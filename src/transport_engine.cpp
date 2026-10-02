#include "transport_engine.h"
#include <string.h>

// =============================================================================
// 1. BOUNDED BYTE-STREAM PARSER (TransportParser)
// =============================================================================

TransportParser::TransportParser(TransportNodeRole role)
    : role_(role), state_(ParserState::SEEK_MAGIC0), last_byte_time_ms_(0),
      header_idx_(0), payload_idx_(0), crc_idx_(0), stats_{} {
    memset(&current_header_, 0, sizeof(current_header_));
    memset(raw_header_, 0, sizeof(raw_header_));
    memset(payload_buffer_, 0, sizeof(payload_buffer_));
    memset(raw_crc_, 0, sizeof(raw_crc_));
}

void TransportParser::reset() {
    state_ = ParserState::SEEK_MAGIC0;
    header_idx_ = 0;
    payload_idx_ = 0;
    crc_idx_ = 0;
    last_byte_time_ms_ = 0;
}

bool TransportParser::feed_byte(uint8_t byte, uint32_t current_time_ms) {
    // Inter-byte gap timeout: reset parser if gap > 50 ms
    if (state_ != ParserState::SEEK_MAGIC0 && last_byte_time_ms_ > 0) {
        if (current_time_ms - last_byte_time_ms_ > 50) {
            stats_.parse_timeouts++;
            reset();
        }
    }
    last_byte_time_ms_ = current_time_ms;

    switch (state_) {
    case ParserState::SEEK_MAGIC0:
        if (byte == TRANSPORT_MAGIC0) {
            raw_header_[0] = byte;
            header_idx_ = 1;
            state_ = ParserState::SEEK_MAGIC1;
        } else {
            stats_.magic_drops++;
        }
        return false;

    case ParserState::SEEK_MAGIC1:
        if (byte == TRANSPORT_MAGIC1) {
            raw_header_[1] = byte;
            header_idx_ = 2;
            state_ = ParserState::READ_HEADER;
        } else {
            stats_.magic_drops++;
            state_ = ParserState::SEEK_MAGIC0;
            if (byte == TRANSPORT_MAGIC0) {
                raw_header_[0] = byte;
                header_idx_ = 1;
                state_ = ParserState::SEEK_MAGIC1;
            }
        }
        return false;

    case ParserState::READ_HEADER:
        raw_header_[header_idx_++] = byte;
        if (header_idx_ == TRANSPORT_HEADER_SIZE) {
            transport_decode_header(raw_header_, &current_header_);

            if (current_header_.version != TRANSPORT_VERSION) {
                stats_.version_errors++;
                state_ = ParserState::SEEK_MAGIC0;
                return false;
            }
            if (!transport_is_valid_channel(current_header_.channel)) {
                stats_.channel_errors++;
                state_ = ParserState::SEEK_MAGIC0;
                return false;
            }
            if (!transport_is_valid_direction(role_, current_header_.channel)) {
                stats_.direction_errors++;
                state_ = ParserState::SEEK_MAGIC0;
                return false;
            }
            if (current_header_.payload_length > TRANSPORT_MAX_SINGLE_BURST_PAYLOAD) {
                stats_.payload_too_large++;
                state_ = ParserState::SEEK_MAGIC0;
                return false;
            }

            if (current_header_.payload_length > 0) {
                payload_idx_ = 0;
                state_ = ParserState::READ_PAYLOAD;
            } else {
                crc_idx_ = 0;
                state_ = ParserState::READ_CRC;
            }
        }
        return false;

    case ParserState::READ_PAYLOAD:
        payload_buffer_[payload_idx_++] = byte;
        if (payload_idx_ == current_header_.payload_length) {
            crc_idx_ = 0;
            state_ = ParserState::READ_CRC;
        }
        return false;

    case ParserState::READ_CRC:
        raw_crc_[crc_idx_++] = byte;
        if (crc_idx_ == TRANSPORT_CRC_SIZE) {
            uint16_t received_crc = transport_read_u16_be(raw_crc_);
            uint16_t computed_crc = transport_crc16(raw_header_, TRANSPORT_HEADER_SIZE);
            if (current_header_.payload_length > 0) {
                computed_crc = transport_crc16(payload_buffer_, current_header_.payload_length, computed_crc);
            }

            state_ = ParserState::SEEK_MAGIC0;
            if (received_crc == computed_crc) {
                stats_.frames_received++;
                return true;
            } else {
                stats_.crc_errors++;
                return false;
            }
        }
        return false;
    }

    return false;
}

size_t TransportParser::feed_buffer(const uint8_t* data, size_t length, uint32_t current_time_ms,
                                    void (*on_frame_cb)(const TransportHeader& hdr, const uint8_t* payload, uint16_t plen, void* ctx),
                                    void* ctx) {
    if (!data || length == 0) return 0;
    size_t frame_count = 0;
    for (size_t i = 0; i < length; ++i) {
        if (feed_byte(data[i], current_time_ms)) {
            frame_count++;
            if (on_frame_cb) {
                on_frame_cb(current_header_, payload_buffer_, current_header_.payload_length, ctx);
            }
        }
    }
    return frame_count;
}

// =============================================================================
// 2. FRAGMENTER (TransportFragmenter)
// =============================================================================

TransportFragmenter::TransportFragmenter()
    : active_(false), reliable_(true), channel_(TransportChannel::MAVLINK_DOWNLINK),
      current_transfer_id_(0), data_ptr_(nullptr), total_length_(0), current_offset_(0),
      total_fragments_(0), current_fragment_idx_(0) {}

void TransportFragmenter::reset() {
    active_ = false;
    data_ptr_ = nullptr;
    total_length_ = 0;
    current_offset_ = 0;
    total_fragments_ = 0;
    current_fragment_idx_ = 0;
}

bool TransportFragmenter::start_transfer(TransportChannel channel, const uint8_t* data, uint16_t length, bool reliable) {
    if (!data || length == 0 || length > TRANSPORT_MAX_TRANSFER_SIZE) {
        return false;
    }
    if (channel != TransportChannel::MAVLINK_DOWNLINK && channel != TransportChannel::MAVLINK_UPLINK) {
        return false;
    }
    active_ = true;
    reliable_ = reliable;
    channel_ = channel;
    data_ptr_ = data;
    total_length_ = length;
    current_offset_ = 0;
    current_fragment_idx_ = 0;
    total_fragments_ = (uint8_t)((length + TRANSPORT_MAX_SINGLE_BURST_PAYLOAD - 1) / TRANSPORT_MAX_SINGLE_BURST_PAYLOAD);
    current_transfer_id_++;
    return true;
}

bool TransportFragmenter::has_next_fragment() const {
    return active_ && (current_offset_ < total_length_);
}

size_t TransportFragmenter::get_next_fragment(uint8_t* out_frame_buf, uint16_t sequence) {
    if (!has_next_fragment() || !out_frame_buf) {
        return 0;
    }

    uint16_t remaining = total_length_ - current_offset_;
    uint16_t chunk_len = (remaining > TRANSPORT_MAX_SINGLE_BURST_PAYLOAD) ? TRANSPORT_MAX_SINGLE_BURST_PAYLOAD : remaining;

    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)channel_;
    hdr.flags = 0;
    if (reliable_) hdr.flags |= TRANSPORT_FLAG_RELIABLE;
    if (current_offset_ == 0) hdr.flags |= TRANSPORT_FLAG_FIRST_FRAG;
    if (current_offset_ + chunk_len >= total_length_) hdr.flags |= TRANSPORT_FLAG_LAST_FRAG;
    hdr.sequence = sequence;
    hdr.transfer_id = current_transfer_id_;
    hdr.fragment_offset = current_offset_;
    hdr.payload_length = chunk_len;

    transport_encode_header(&hdr, out_frame_buf);
    memcpy(&out_frame_buf[TRANSPORT_HEADER_SIZE], &data_ptr_[current_offset_], chunk_len);

    uint16_t crc = transport_crc16(out_frame_buf, TRANSPORT_HEADER_SIZE + chunk_len);
    transport_write_u16_be(&out_frame_buf[TRANSPORT_HEADER_SIZE + chunk_len], crc);

    current_offset_ += chunk_len;
    current_fragment_idx_++;
    if (current_offset_ >= total_length_) {
        active_ = false;
    }

    return TRANSPORT_HEADER_SIZE + chunk_len + TRANSPORT_CRC_SIZE;
}

// =============================================================================
// 3. REASSEMBLER (TransportReassembler)
// =============================================================================

TransportReassembler::TransportReassembler()
    : has_active_transfer_(false), is_complete_(false),
      active_channel_(TransportChannel::MAVLINK_DOWNLINK), active_transfer_id_(0),
      last_activity_time_ms_(0), total_size_(0), range_count_(0), stats_{} {
    memset(reassembly_buffer_, 0, sizeof(reassembly_buffer_));
    memset(ranges_, 0, sizeof(ranges_));
}

void TransportReassembler::reset() {
    has_active_transfer_ = false;
    is_complete_ = false;
    active_transfer_id_ = 0;
    last_activity_time_ms_ = 0;
    total_size_ = 0;
    range_count_ = 0;
}

void TransportReassembler::mark_complete_consumed() {
    has_active_transfer_ = false;
    is_complete_ = false;
    active_transfer_id_ = 0;
    total_size_ = 0;
    range_count_ = 0;
}

bool TransportReassembler::check_coverage() {
    if (total_size_ == 0) return false;
    // Verify every byte from 0 to total_size_ is covered by received ranges
    for (uint16_t b = 0; b < total_size_; ++b) {
        bool covered = false;
        for (uint8_t r = 0; r < range_count_; ++r) {
            if (b >= ranges_[r].start && b < ranges_[r].end) {
                covered = true;
                break;
            }
        }
        if (!covered) return false;
    }
    return true;
}

TransportNackReason TransportReassembler::process_fragment(const TransportHeader& hdr, const uint8_t* payload, uint32_t current_time_ms) {
    if (hdr.payload_length == 0 || (hdr.payload_length > 0 && payload == nullptr)) {
        return TransportNackReason::BAD_CRC;
    }

    if (hdr.channel != (uint8_t)TransportChannel::MAVLINK_DOWNLINK &&
        hdr.channel != (uint8_t)TransportChannel::MAVLINK_UPLINK) {
        return TransportNackReason::BAD_CRC;
    }

    uint32_t start = hdr.fragment_offset;
    uint32_t end = start + hdr.payload_length;

    if (start >= TRANSPORT_MAX_TRANSFER_SIZE ||
        end > TRANSPORT_MAX_TRANSFER_SIZE ||
        end < start) {
        stats_.offset_overruns++;
        return TransportNackReason::OFFSET_OVERRUN;
    }

    uint16_t u16_start = (uint16_t)start;
    uint16_t u16_end = (uint16_t)end;

    if (!has_active_transfer_) {
        // Missing first fragment check when no transfer is active
        if (!(hdr.flags & TRANSPORT_FLAG_FIRST_FRAG)) {
            stats_.gaps_detected++;
            return TransportNackReason::GAP_DETECTED;
        }
        // Initialize new transfer
        has_active_transfer_ = true;
        is_complete_ = false;
        active_channel_ = (TransportChannel)hdr.channel;
        active_transfer_id_ = hdr.transfer_id;
        range_count_ = 0;
        total_size_ = 0;
    } else if (is_complete_) {
        // Previous transfer already completed
        if (hdr.transfer_id == active_transfer_id_ && hdr.channel == (uint8_t)active_channel_) {
            // Retransmitted duplicate of completed transfer: re-ACK idempotently
            for (uint8_t r = 0; r < range_count_; ++r) {
                if (ranges_[r].start == u16_start && ranges_[r].end == u16_end) {
                    stats_.duplicates_received++;
                    return TransportNackReason::NONE;
                }
            }
            stats_.overlaps_detected++;
            return TransportNackReason::OVERLAP_CONFLICT;
        } else {
            // New transfer arriving after previous completed: requires FIRST_FRAG
            if (!(hdr.flags & TRANSPORT_FLAG_FIRST_FRAG)) {
                stats_.gaps_detected++;
                return TransportNackReason::GAP_DETECTED;
            }
            // Atomically initialize new transfer, replacing completed result
            has_active_transfer_ = true;
            is_complete_ = false;
            active_channel_ = (TransportChannel)hdr.channel;
            active_transfer_id_ = hdr.transfer_id;
            range_count_ = 0;
            total_size_ = 0;
        }
    } else {
        // Active transfer in progress: preserve active transfer, reject new transfer_id
        if (hdr.transfer_id != active_transfer_id_ || hdr.channel != (uint8_t)active_channel_) {
            stats_.buffer_full_rejections++;
            return TransportNackReason::BUFFER_FULL;
        }

        // Duplicate vs partial overlap check
        for (uint8_t r = 0; r < range_count_; ++r) {
            if (ranges_[r].start == u16_start && ranges_[r].end == u16_end) {
                stats_.duplicates_received++;
                return TransportNackReason::NONE; // Exact duplicate re-ACKed
            }
            uint16_t max_start = (u16_start > ranges_[r].start) ? u16_start : ranges_[r].start;
            uint16_t min_end = (u16_end < ranges_[r].end) ? u16_end : ranges_[r].end;
            if (max_start < min_end) {
                stats_.overlaps_detected++;
                return TransportNackReason::OVERLAP_CONFLICT;
            }
        }
    }

    if (range_count_ >= TRANSPORT_MAX_FRAGMENTS) {
        stats_.offset_overruns++;
        return TransportNackReason::OFFSET_OVERRUN;
    }

    memcpy(&reassembly_buffer_[u16_start], payload, hdr.payload_length);
    ranges_[range_count_].start = u16_start;
    ranges_[range_count_].end = u16_end;
    range_count_++;
    last_activity_time_ms_ = current_time_ms;

    if (hdr.flags & TRANSPORT_FLAG_LAST_FRAG) {
        total_size_ = u16_end;
    }

    if (total_size_ > 0 && check_coverage()) {
        is_complete_ = true;
        stats_.transfers_completed++;
    }

    return TransportNackReason::NONE;
}

bool TransportReassembler::check_timeout(uint32_t current_time_ms) {
    if (has_active_transfer_ && !is_complete_ && last_activity_time_ms_ > 0) {
        if (current_time_ms - last_activity_time_ms_ >= TRANSPORT_TRANSFER_TIMEOUT_MS) {
            stats_.timeouts++;
            reset();
            return true;
        }
    }
    return false;
}

// =============================================================================
// 4. RETRY STATE MACHINE (TransportRetryManager)
// =============================================================================

TransportRetryManager::TransportRetryManager()
    : state_(RetryState::IDLE), retry_count_(0), active_transfer_id_(0),
      active_fragment_offset_(0), tx_time_ms_(0), pending_frame_len_(0), stats_{} {
    memset(pending_frame_, 0, sizeof(pending_frame_));
}

void TransportRetryManager::reset() {
    state_ = RetryState::IDLE;
    retry_count_ = 0;
    active_transfer_id_ = 0;
    active_fragment_offset_ = 0;
    tx_time_ms_ = 0;
    pending_frame_len_ = 0;
}

void TransportRetryManager::arm(const uint8_t* frame_bytes, size_t frame_len, uint16_t transfer_id,
                               uint16_t fragment_offset, uint32_t current_time_ms) {
    if (!frame_bytes || frame_len == 0 || frame_len > TRANSPORT_MAX_FRAME_SIZE) {
        return;
    }
    state_ = RetryState::WAITING_ACK;
    retry_count_ = 0;
    active_transfer_id_ = transfer_id;
    active_fragment_offset_ = fragment_offset;
    tx_time_ms_ = current_time_ms;
    pending_frame_len_ = frame_len;
    memcpy(pending_frame_, frame_bytes, frame_len);
}

bool TransportRetryManager::process_ack_nack(const TransportLinkAckNack& ack_nack) {
    if (state_ != RetryState::WAITING_ACK) {
        return false;
    }
    if (ack_nack.transfer_id == active_transfer_id_ && ack_nack.fragment_offset == active_fragment_offset_) {
        if (ack_nack.nack_reason == (uint8_t)TransportNackReason::NONE) {
            state_ = RetryState::IDLE;
            stats_.acks_received++;
            return true;
        } else {
            stats_.nacks_received++;
            return false;
        }
    }
    return false;
}

bool TransportRetryManager::check_retry_timeout(uint32_t current_time_ms) {
    if (state_ != RetryState::WAITING_ACK) {
        return false;
    }
    if (current_time_ms - tx_time_ms_ >= TRANSPORT_RETRY_TIMEOUT_MS) {
        if (retry_count_ < TRANSPORT_MAX_RETRIES) {
            retry_count_++;
            tx_time_ms_ = current_time_ms;
            stats_.retries_sent++;
            return true; // Retransmit now
        } else {
            state_ = RetryState::EXHAUSTED;
            stats_.transfer_aborts++;
            return false; // Transfer aborted
        }
    }
    return false;
}

const uint8_t* TransportRetryManager::get_retry_frame(size_t* out_len) const {
    if (out_len) *out_len = pending_frame_len_;
    return pending_frame_;
}

// =============================================================================
// 5. RC LATEST-VALUE MAILBOX (TransportRcMailbox)
// =============================================================================

TransportRcMailbox::TransportRcMailbox() : has_data_(false), sequence_(0), flags_(0) {
    for (size_t i = 0; i < TRANSPORT_RC_NUM_CHANNELS; ++i) {
        channels_[i] = 1024; // Mid-point default
    }
}

bool TransportRcMailbox::update_from_wire(const uint8_t* wire_24bytes) {
    if (!wire_24bytes) return false;
    if (!transport_decode_packed_rc(wire_24bytes, channels_, &sequence_, &flags_)) {
        return false;
    }
    has_data_ = true;
    return true;
}

void TransportRcMailbox::update_from_channels(const uint16_t* channels, uint8_t sequence, uint8_t flags) {
    if (!channels) return;
    for (size_t i = 0; i < TRANSPORT_RC_NUM_CHANNELS; ++i) {
        channels_[i] = channels[i];
    }
    sequence_ = sequence;
    flags_ = flags;
    has_data_ = true;
}

void TransportRcMailbox::read_channels(uint16_t* out_channels, uint8_t* out_sequence, uint8_t* out_flags) const {
    if (out_channels) {
        for (size_t i = 0; i < TRANSPORT_RC_NUM_CHANNELS; ++i) {
            out_channels[i] = channels_[i];
        }
    }
    if (out_sequence) *out_sequence = sequence_;
    if (out_flags) *out_flags = flags_;
}

// =============================================================================
// 6. RC FAILSAFE STATE MACHINE (TransportRcFailsafe)
// =============================================================================

TransportRcFailsafe::TransportRcFailsafe(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms), last_valid_time_ms_(0),
      consecutive_valid_frames_(0), state_(RcFailsafeState::NO_SIGNAL) {}

void TransportRcFailsafe::reset() {
    last_valid_time_ms_ = 0;
    consecutive_valid_frames_ = 0;
    state_ = RcFailsafeState::NO_SIGNAL;
}

void TransportRcFailsafe::record_frame_arrival(uint32_t current_time_ms) {
    last_valid_time_ms_ = current_time_ms;
    consecutive_valid_frames_++;

    if (state_ == RcFailsafeState::FAILSAFE) {
        if (consecutive_valid_frames_ >= TRANSPORT_RC_RESTORE_FRAME_COUNT) {
            state_ = RcFailsafeState::ACTIVE;
        }
    } else {
        state_ = RcFailsafeState::ACTIVE;
    }
}

void TransportRcFailsafe::update(uint32_t current_time_ms) {
    if (last_valid_time_ms_ == 0) {
        state_ = RcFailsafeState::NO_SIGNAL;
        return;
    }

    uint32_t elapsed = current_time_ms - last_valid_time_ms_;
    if (elapsed > timeout_ms_) {
        state_ = RcFailsafeState::FAILSAFE;
        consecutive_valid_frames_ = 0;
    } else if (elapsed > 100 && state_ == RcFailsafeState::ACTIVE) {
        state_ = RcFailsafeState::DEGRADED;
    }
}
