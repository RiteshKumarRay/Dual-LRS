#include "link_transport.h"
#include <algorithm>

LinkTransport::LinkTransport(TransportNodeRole role)
    : role_(role),
      fragmenter_(),
      retry_manager_(),
      reassembler_(),
      ack_queue_count_(0),
      ack_queue_head_(0),
      ack_queue_tail_(0),
      current_reliable_frame_len_(0),
      current_reliable_seq_(0),
      current_reliable_channel_(TransportChannel::LINK_CONTROL),
      current_reliable_transfer_id_(0),
      current_reliable_offset_(0),
      has_current_reliable_frame_(false),
      nack_retransmit_pending_(false),
      timeout_retransmit_pending_(false),
      stats_{} {
    memset(ack_queue_, 0, sizeof(ack_queue_));
    memset(current_reliable_frame_, 0, sizeof(current_reliable_frame_));
}

void LinkTransport::reset() {
    fragmenter_.reset();
    retry_manager_.reset();
    reassembler_.reset();
    ack_queue_count_ = 0;
    ack_queue_head_ = 0;
    ack_queue_tail_ = 0;
    current_reliable_frame_len_ = 0;
    current_reliable_seq_ = 0;
    current_reliable_channel_ = TransportChannel::LINK_CONTROL;
    current_reliable_transfer_id_ = 0;
    current_reliable_offset_ = 0;
    has_current_reliable_frame_ = false;
    nack_retransmit_pending_ = false;
    timeout_retransmit_pending_ = false;
}

bool LinkTransport::start_transfer(TransportChannel channel, const uint8_t* payload, uint16_t length, bool reliable) {
    if (!payload || length == 0 || length > TRANSPORT_MAX_TRANSFER_SIZE) {
        return false;
    }

    // Do not clobber an active outgoing transfer
    if (is_tx_active()) {
        return false;
    }

    if (!fragmenter_.start_transfer(channel, payload, length, reliable)) {
        return false;
    }

    retry_manager_.reset();
    has_current_reliable_frame_ = false;
    nack_retransmit_pending_ = false;
    timeout_retransmit_pending_ = false;
    stats_.transfers_started++;
    return true;
}

void LinkTransport::update(uint32_t current_time_ms) {
    reassembler_.check_timeout(current_time_ms);

    if (retry_manager_.get_state() == RetryState::WAITING_ACK) {
        if (retry_manager_.check_retry_timeout(current_time_ms)) {
            timeout_retransmit_pending_ = true;
            stats_.retries_sent++;
        } else if (retry_manager_.get_state() == RetryState::EXHAUSTED) {
            // Maximum 5 retries exceeded: abort transfer cleanly without blocking future traffic
            stats_.transfer_aborts++;
            fragmenter_.reset();
            retry_manager_.reset();
            has_current_reliable_frame_ = false;
            nack_retransmit_pending_ = false;
            timeout_retransmit_pending_ = false;
        }
    }
}

bool LinkTransport::has_outgoing_frame() const {
    // 1. Pending ACK/NACK has highest data priority
    if (has_pending_ack_nack()) {
        return true;
    }

    // 2. Pending NACK retransmission
    if (nack_retransmit_pending_) {
        return true;
    }

    // 3. Pending retry timeout retransmission
    if (timeout_retransmit_pending_) {
        return true;
    }

    // 4. Next fragment ready (only if retry manager is IDLE)
    if (retry_manager_.get_state() == RetryState::IDLE && fragmenter_.has_next_fragment()) {
        return true;
    }

    return false;
}

bool LinkTransport::queue_ack_nack(uint8_t target_channel, uint16_t transfer_id, uint16_t fragment_offset, TransportNackReason reason) {
    if (ack_queue_count_ >= LINK_TRANSPORT_MAX_ACK_QUEUE) {
        stats_.mack_dropped_queue_full++;
        return false;
    }

    TransportLinkAckNack& an = ack_queue_[ack_queue_tail_];
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = target_channel;
    an.transfer_id = transfer_id;
    an.fragment_offset = fragment_offset;
    an.nack_reason = (uint8_t)reason;
    an.reserved = 0;

    ack_queue_tail_ = (ack_queue_tail_ + 1) % LINK_TRANSPORT_MAX_ACK_QUEUE;
    ack_queue_count_++;
    return true;
}

bool LinkTransport::pop_ack_nack(TransportLinkAckNack* out_an) {
    if (!out_an || ack_queue_count_ == 0) {
        return false;
    }

    *out_an = ack_queue_[ack_queue_head_];
    ack_queue_head_ = (ack_queue_head_ + 1) % LINK_TRANSPORT_MAX_ACK_QUEUE;
    ack_queue_count_--;
    return true;
}

size_t LinkTransport::get_next_tx_frame(uint8_t* out_frame_buf, uint16_t next_seq, uint32_t current_time_ms) {
    if (!out_frame_buf) return 0;

    // Check timers
    update(current_time_ms);

    // -------------------------------------------------------------------------
    // PRIORITY 1: High-Priority ACK / NACK Frame
    // -------------------------------------------------------------------------
    if (has_pending_ack_nack()) {
        TransportLinkAckNack an{};
        if (pop_ack_nack(&an)) {
            TransportHeader hdr{};
            hdr.magic0 = TRANSPORT_MAGIC0;
            hdr.magic1 = TRANSPORT_MAGIC1;
            hdr.version = TRANSPORT_VERSION;
            hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
            hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG; // Single burst, never reliable
            hdr.sequence = next_seq;
            hdr.transfer_id = 0;
            hdr.fragment_offset = 0;
            hdr.payload_length = sizeof(TransportLinkAckNack); // 8 bytes

            transport_encode_header(&hdr, out_frame_buf);
            transport_encode_link_ack_nack(&an, &out_frame_buf[TRANSPORT_HEADER_SIZE]);

            uint16_t crc = transport_crc16(out_frame_buf, TRANSPORT_HEADER_SIZE + sizeof(TransportLinkAckNack));
            transport_write_u16_be(&out_frame_buf[TRANSPORT_HEADER_SIZE + sizeof(TransportLinkAckNack)], crc);

            return TRANSPORT_HEADER_SIZE + sizeof(TransportLinkAckNack) + TRANSPORT_CRC_SIZE; // 23 bytes
        }
    }

    // -------------------------------------------------------------------------
    // PRIORITY 2: Retransmission Frame (NACK arrival or Retry Timeout)
    // -------------------------------------------------------------------------
    if (nack_retransmit_pending_ || timeout_retransmit_pending_) {
        if (nack_retransmit_pending_) {
            stats_.retries_sent++;
        }
        nack_retransmit_pending_ = false;
        timeout_retransmit_pending_ = false;
        size_t retry_len = 0;
        const uint8_t* retry_bytes = retry_manager_.get_retry_frame(&retry_len);
        if (retry_bytes && retry_len > 0 && retry_len <= TRANSPORT_MAX_FRAME_SIZE) {
            memcpy(out_frame_buf, retry_bytes, retry_len);
            return retry_len;
        }
    }

    // -------------------------------------------------------------------------
    // PRIORITY 3: Next Outgoing Data Fragment (only when retry manager is IDLE)
    // -------------------------------------------------------------------------
    if (retry_manager_.get_state() == RetryState::IDLE && fragmenter_.has_next_fragment()) {
        size_t frame_len = fragmenter_.get_next_fragment(out_frame_buf, next_seq);
        if (frame_len > 0) {
            stats_.fragments_sent++;

            TransportHeader hdr{};
            transport_decode_header(out_frame_buf, &hdr);

            if (hdr.flags & TRANSPORT_FLAG_RELIABLE) {
                // Lockstep reliable transfer: arm retry manager with exact frame bytes
                retry_manager_.arm(out_frame_buf, frame_len, hdr.transfer_id, hdr.fragment_offset, current_time_ms);
                has_current_reliable_frame_ = true;
                current_reliable_channel_ = (TransportChannel)hdr.channel;
                current_reliable_transfer_id_ = hdr.transfer_id;
                current_reliable_offset_ = hdr.fragment_offset;
                current_reliable_seq_ = next_seq;
                current_reliable_frame_len_ = frame_len;
                memcpy(current_reliable_frame_, out_frame_buf, frame_len);
            } else {
                // Unreliable stream: does not wait for ACK
                if (!fragmenter_.has_next_fragment()) {
                    stats_.transfers_completed_tx++;
                }
            }
            return frame_len;
        }
    }

    return 0;
}

bool LinkTransport::ingest_transport_frame(const TransportHeader& hdr, const uint8_t* payload, uint16_t length, uint32_t current_time_ms) {
    if (length == 0 || !payload) return false;

    // -------------------------------------------------------------------------
    // CASE A: LINK_CONTROL Frame (ACK / NACK)
    // -------------------------------------------------------------------------
    if (hdr.channel == (uint8_t)TransportChannel::LINK_CONTROL) {
        if (hdr.payload_length >= sizeof(TransportLinkAckNack) && payload[0] == (uint8_t)LinkSubtype::ACK_NACK) {
            TransportLinkAckNack an{};
            if (transport_decode_link_ack_nack(payload, &an)) {
                // Strict validation: ACK/NACK must match the channel, transfer_id, and fragment_offset
                // of the currently pending reliable fragment awaiting ACK.
                bool is_waiting = has_current_reliable_frame_ && (retry_manager_.get_state() == RetryState::WAITING_ACK);
                bool matches = is_waiting &&
                                (an.target_channel == (uint8_t)current_reliable_channel_) &&
                                (an.transfer_id == current_reliable_transfer_id_) &&
                                (an.fragment_offset == current_reliable_offset_);

                if (!matches) {
                    // Stale, out-of-window, or mismatched ACK/NACK: safely ignored
                    return false;
                }

                if (an.nack_reason == (uint8_t)TransportNackReason::NONE) {
                    // Valid matching ACK
                    if (retry_manager_.process_ack_nack(an)) {
                        stats_.acks_received++;
                        has_current_reliable_frame_ = false;
                        nack_retransmit_pending_ = false;
                        timeout_retransmit_pending_ = false;

                        // Check if the acknowledged fragment was the last fragment of the transfer
                        if (!fragmenter_.has_next_fragment()) {
                            stats_.transfers_completed_tx++;
                        }
                        return true;
                    }
                } else {
                    // Valid matching NACK: trigger immediate retransmission of stored fragment
                    stats_.nacks_received++;
                    nack_retransmit_pending_ = true;
                    return true;
                }
            }
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // CASE B: MAVLink Data Fragment (Downlink or Uplink)
    // -------------------------------------------------------------------------
    if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK ||
        hdr.channel == (uint8_t)TransportChannel::MAVLINK_UPLINK) {

        // Validate direction according to node role
        if (role_ == TransportNodeRole::GROUND && hdr.channel != (uint8_t)TransportChannel::MAVLINK_DOWNLINK) {
            return false;
        }
        if (role_ == TransportNodeRole::AIR && hdr.channel != (uint8_t)TransportChannel::MAVLINK_UPLINK) {
            return false;
        }

        stats_.fragments_received++;
        bool is_reliable = (hdr.flags & TRANSPORT_FLAG_RELIABLE) != 0;

        // Ingest into reassembler
        TransportNackReason reason = reassembler_.process_fragment(hdr, payload, current_time_ms);

        // Generate ACK or NACK for reliable streams (never send ACK for unreliable routine telemetry)
        if (is_reliable) {
            queue_ack_nack(hdr.channel, hdr.transfer_id, hdr.fragment_offset, reason);
            if (reason == TransportNackReason::NONE) {
                stats_.acks_sent++;
            } else {
                stats_.nacks_sent++;
            }
        }

        if (reassembler_.is_complete()) {
            stats_.transfers_completed_rx++;
        }

        return true;
    }

    return false;
}

size_t LinkTransport::get_received_message(uint8_t* out_buf, size_t max_len) {
    if (!out_buf || max_len == 0 || !has_received_message()) {
        return 0;
    }

    uint16_t reasm_len = reassembler_.get_reassembled_length();
    size_t copy_len = (reasm_len > max_len) ? max_len : reasm_len;
    memcpy(out_buf, reassembler_.get_reassembled_data(), copy_len);
    return copy_len;
}

void LinkTransport::pop_received_message() {
    reassembler_.mark_complete_consumed();
}
