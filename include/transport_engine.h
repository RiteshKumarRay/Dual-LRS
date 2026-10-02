#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "transport_protocol.h"

#ifdef ARDUINO
#include <Arduino.h>
static inline uint32_t transport_get_time_ms() { return millis(); }
#else
#include <chrono>
static inline uint32_t transport_get_time_ms() {
    using namespace std::chrono;
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
#endif

// =============================================================================
// 1. BOUNDED BYTE-STREAM PARSER (TransportParser)
// =============================================================================
enum class ParserState : uint8_t {
    SEEK_MAGIC0,
    SEEK_MAGIC1,
    READ_HEADER,
    READ_PAYLOAD,
    READ_CRC
};

struct TransportParserStats {
    uint32_t frames_received;
    uint32_t crc_errors;
    uint32_t magic_drops;
    uint32_t version_errors;
    uint32_t channel_errors;
    uint32_t direction_errors;
    uint32_t payload_too_large;
    uint32_t parse_timeouts;
};

class TransportParser {
public:
    explicit TransportParser(TransportNodeRole role);
    void reset();

    // Feed a single byte into the parser state machine
    // Returns true when a complete, verified frame has been assembled
    bool feed_byte(uint8_t byte, uint32_t current_time_ms);

    // Feed a block of bytes
    // Calls on_frame_cb for every complete, verified frame decoded
    size_t feed_buffer(const uint8_t* data, size_t length, uint32_t current_time_ms,
                       void (*on_frame_cb)(const TransportHeader& hdr, const uint8_t* payload, uint16_t plen, void* ctx),
                       void* ctx);

    const TransportHeader& get_header() const { return current_header_; }
    const uint8_t* get_payload() const { return payload_buffer_; }
    uint16_t get_payload_length() const { return current_header_.payload_length; }
    const TransportParserStats& get_stats() const { return stats_; }

private:
    TransportNodeRole role_;
    ParserState state_;
    uint32_t last_byte_time_ms_;
    uint8_t raw_header_[TRANSPORT_HEADER_SIZE];
    uint8_t header_idx_;
    uint8_t payload_buffer_[TRANSPORT_MAX_SINGLE_BURST_PAYLOAD];
    uint16_t payload_idx_;
    uint8_t raw_crc_[TRANSPORT_CRC_SIZE];
    uint8_t crc_idx_;
    TransportHeader current_header_;
    TransportParserStats stats_;
};

// =============================================================================
// 2. FRAGMENTER (TransportFragmenter)
// =============================================================================
class TransportFragmenter {
public:
    TransportFragmenter();
    void reset();

    // Start fragmenting a new outgoing multi-fragment message
    bool start_transfer(TransportChannel channel, const uint8_t* data, uint16_t length, bool reliable = true);

    // Check if more fragments are pending
    bool has_next_fragment() const;

    // Pull next fragment into wire frame buffer (Header + Payload + CRC)
    // Returns total wire frame size (<= 64 bytes), or 0 if no fragment pending
    size_t get_next_fragment(uint8_t* out_frame_buf, uint16_t sequence);

    bool is_transfer_active() const { return active_; }
    uint16_t get_transfer_id() const { return current_transfer_id_; }
    uint16_t get_total_length() const { return total_length_; }
    uint8_t get_fragment_count() const { return total_fragments_; }
    uint8_t get_fragment_index() const { return current_fragment_idx_; }

private:
    bool active_;
    bool reliable_;
    TransportChannel channel_;
    uint16_t current_transfer_id_;
    const uint8_t* data_ptr_;
    uint16_t total_length_;
    uint16_t current_offset_;
    uint8_t total_fragments_;
    uint8_t current_fragment_idx_;
};

// =============================================================================
// 3. REASSEMBLER (TransportReassembler)
// =============================================================================
struct TransportReassemblerStats {
    uint32_t transfers_completed;
    uint32_t duplicates_received;
    uint32_t gaps_detected;
    uint32_t overlaps_detected;
    uint32_t buffer_full_rejections;
    uint32_t offset_overruns;
    uint32_t timeouts;
};

class TransportReassembler {
public:
    TransportReassembler();
    void reset();

    // Ingests an incoming fragment.
    // Returns:
    //   TransportNackReason::NONE if accepted (or exact duplicate re-ACKed)
    //   TransportNackReason::* if rejected
    TransportNackReason process_fragment(const TransportHeader& hdr, const uint8_t* payload, uint32_t current_time_ms);

    // Periodic tick to handle 1000ms inactivity timeout
    bool check_timeout(uint32_t current_time_ms);

    // Releases completed transfer buffer once consumed by application
    void mark_complete_consumed();

    bool is_complete() const { return is_complete_; }
    bool has_active_transfer() const { return has_active_transfer_; }
    uint16_t get_active_transfer_id() const { return active_transfer_id_; }
    TransportChannel get_active_channel() const { return active_channel_; }
    const uint8_t* get_reassembled_data() const { return reassembly_buffer_; }
    uint16_t get_reassembled_length() const { return total_size_; }
    const TransportReassemblerStats& get_stats() const { return stats_; }

private:
    bool has_active_transfer_;
    bool is_complete_;
    TransportChannel active_channel_;
    uint16_t active_transfer_id_;
    uint32_t last_activity_time_ms_;
    uint16_t total_size_;
    uint8_t reassembly_buffer_[TRANSPORT_MAX_TRANSFER_SIZE];

    // Range tracking: max 11 fragments
    struct ReceivedRange {
        uint16_t start;
        uint16_t end;
    };
    ReceivedRange ranges_[TRANSPORT_MAX_FRAGMENTS];
    uint8_t range_count_;
    TransportReassemblerStats stats_;

    bool check_coverage();
};

// =============================================================================
// 4. RETRY STATE MACHINE (TransportRetryManager)
// =============================================================================
enum class RetryState : uint8_t {
    IDLE,
    WAITING_ACK,
    EXHAUSTED
};

struct TransportRetryStats {
    uint32_t retries_sent;
    uint32_t acks_received;
    uint32_t nacks_received;
    uint32_t transfer_aborts;
};

class TransportRetryManager {
public:
    TransportRetryManager();
    void reset();

    // Arms the retry manager with the last transmitted reliable frame
    void arm(const uint8_t* frame_bytes, size_t frame_len, uint16_t transfer_id,
             uint16_t fragment_offset, uint32_t current_time_ms);

    // Processes an incoming ACK/NACK
    // Returns true if ACK matches and clears retry state
    bool process_ack_nack(const TransportLinkAckNack& ack_nack);

    // Ticks the retry timer
    // Returns true if a retransmission is required now
    bool check_retry_timeout(uint32_t current_time_ms);

    // Retrieves frame bytes to retransmit
    const uint8_t* get_retry_frame(size_t* out_len) const;

    RetryState get_state() const { return state_; }
    uint8_t get_retry_count() const { return retry_count_; }
    const TransportRetryStats& get_stats() const { return stats_; }

private:
    RetryState state_;
    uint8_t retry_count_;
    uint16_t active_transfer_id_;
    uint16_t active_fragment_offset_;
    uint32_t tx_time_ms_;
    uint8_t pending_frame_[TRANSPORT_MAX_FRAME_SIZE];
    size_t pending_frame_len_;
    TransportRetryStats stats_;
};

// =============================================================================
// 5. RC LATEST-VALUE MAILBOX (TransportRcMailbox)
// =============================================================================
class TransportRcMailbox {
public:
    TransportRcMailbox();

    // Updates mailbox with newest packed RC frame (overwrites older values)
    bool update_from_wire(const uint8_t* wire_24bytes);

    // Updates mailbox directly from unpacked channel array
    void update_from_channels(const uint16_t* channels, uint8_t sequence, uint8_t flags);

    // Reads latest channels into destination buffer
    void read_channels(uint16_t* out_channels, uint8_t* out_sequence, uint8_t* out_flags) const;

    bool has_data() const { return has_data_; }
    uint8_t get_sequence() const { return sequence_; }
    uint8_t get_flags() const { return flags_; }

private:
    bool has_data_;
    uint16_t channels_[TRANSPORT_RC_NUM_CHANNELS];
    uint8_t sequence_;
    uint8_t flags_;
};

// =============================================================================
// 6. RC FAILSAFE STATE MACHINE (TransportRcFailsafe)
// =============================================================================
enum class RcFailsafeState : uint8_t {
    NO_SIGNAL,
    ACTIVE,
    DEGRADED,
    FAILSAFE
};

class TransportRcFailsafe {
public:
    explicit TransportRcFailsafe(uint32_t timeout_ms = TRANSPORT_RC_FAILSAFE_TIMEOUT_MS);
    void reset();

    // Called on arrival of every valid RC_CONTROL frame
    void record_frame_arrival(uint32_t current_time_ms);

    // Ticks the failsafe state machine
    void update(uint32_t current_time_ms);

    bool is_failsafe_active() const { return state_ == RcFailsafeState::FAILSAFE; }
    RcFailsafeState get_state() const { return state_; }
    uint32_t get_consecutive_valid_frames() const { return consecutive_valid_frames_; }

private:
    uint32_t timeout_ms_;
    uint32_t last_valid_time_ms_;
    uint32_t consecutive_valid_frames_;
    RcFailsafeState state_;
};
