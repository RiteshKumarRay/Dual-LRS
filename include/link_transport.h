#pragma once

#include "transport_protocol.h"
#include "transport_engine.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// =============================================================================
// LINK TRANSPORT LAYER (Dual-LRS Reliable Transfer State Machine)
//
// Responsibilities:
//   - Manages reliable and unreliable MAVLink fragment transport.
//   - Coordinates TransportFragmenter, TransportRetryManager, and TransportReassembler.
//   - Generates and schedules high-priority ACK/NACK frames.
//   - Guarantees lockstep fragment progression (waits for ACK before next fragment).
//   - Handles retransmissions on NACK or timeout, with 5-retry abort limit.
//   - Ensures ACK frames are never recursively ACKed.
//   - Enforces strict priority: RC > ACK/NACK > Data > Routine Telemetry.
// =============================================================================

#define LINK_TRANSPORT_MAX_ACK_QUEUE 4

struct LinkTransportStats {
    uint32_t transfers_started;
    uint32_t transfers_completed_tx;
    uint32_t transfers_completed_rx;
    uint32_t fragments_sent;
    uint32_t fragments_received;
    uint32_t retries_sent;
    uint32_t acks_sent;
    uint32_t acks_received;
    uint32_t nacks_sent;
    uint32_t nacks_received;
    uint32_t duplicates_received;
    uint32_t transfer_aborts;
    uint32_t mack_dropped_queue_full;
};

class LinkTransport {
public:
    explicit LinkTransport(TransportNodeRole role);
    void reset();

    // -------------------------------------------------------------------------
    // TRANSMIT PATH (Sender)
    // -------------------------------------------------------------------------

    // Start a new message transfer (uplink or downlink).
    // Returns true if transfer admitted, false if a transfer is already in progress.
    bool start_transfer(TransportChannel channel, const uint8_t* payload, uint16_t length, bool reliable = true);

    // Periodic time tick (handles retry timers, transfer timeouts).
    void update(uint32_t current_time_ms);

    // Checks if an outgoing frame is ready to send (ACK/NACK, retry, or new fragment).
    bool has_outgoing_frame() const;

    // Checks if an ACK or NACK is pending transmission (high priority).
    bool has_pending_ack_nack() const { return ack_queue_count_ > 0; }

    // Pulls the next frame to send in the assigned TDM slot.
    // Order of precedence:
    //   1. High-priority ACK/NACK frame.
    //   2. Retransmission frame (if retry timer expired or NACK received).
    //   3. New data fragment (if retry manager is IDLE and fragmenter has next).
    // Returns total wire bytes written (<= 64 bytes), or 0 if nothing to send.
    size_t get_next_tx_frame(uint8_t* out_frame_buf, uint16_t next_seq, uint32_t current_time_ms);

    // -------------------------------------------------------------------------
    // RECEIVE PATH (Receiver)
    // -------------------------------------------------------------------------

    // Ingests an incoming verified transport frame.
    // Handles ACK/NACK frames (LINK_CONTROL) or data fragments (MAVLINK_DOWNLINK/UPLINK).
    // Generates ACK/NACK responses for incoming reliable data fragments.
    bool ingest_transport_frame(const TransportHeader& hdr, const uint8_t* payload, uint16_t length, uint32_t current_time_ms);

    // Checks if a complete incoming message has been reassembled.
    bool has_received_message() const { return reassembler_.is_complete(); }

    // Retrieves the reassembled complete message.
    size_t get_received_message(uint8_t* out_buf, size_t max_len);

    // Marks the reassembled complete message as consumed.
    void pop_received_message();

    // -------------------------------------------------------------------------
    // STATE & DIAGNOSTICS
    // -------------------------------------------------------------------------
    bool is_tx_active() const { return fragmenter_.is_transfer_active() || retry_manager_.get_state() == RetryState::WAITING_ACK; }
    bool is_rx_active() const { return reassembler_.has_active_transfer(); }
    uint16_t get_tx_transfer_id() const { return fragmenter_.get_transfer_id(); }
    uint16_t get_rx_transfer_id() const { return reassembler_.get_active_transfer_id(); }
    const LinkTransportStats& get_stats() const { return stats_; }

private:
    TransportNodeRole role_;
    TransportFragmenter fragmenter_;
    TransportRetryManager retry_manager_;
    TransportReassembler reassembler_;

    // Outgoing ACK/NACK queue
    TransportLinkAckNack ack_queue_[LINK_TRANSPORT_MAX_ACK_QUEUE];
    uint8_t ack_queue_count_;
    uint8_t ack_queue_head_;
    uint8_t ack_queue_tail_;

    bool queue_ack_nack(uint8_t target_channel, uint16_t transfer_id, uint16_t fragment_offset, TransportNackReason reason);
    bool pop_ack_nack(TransportLinkAckNack* out_an);

    // Last reliable frame waiting for ACK (cached so retry uses identical sequence)
    uint8_t  current_reliable_frame_[TRANSPORT_MAX_FRAME_SIZE];
    size_t   current_reliable_frame_len_;
    uint16_t current_reliable_seq_;
    bool     has_current_reliable_frame_;
    bool     nack_retransmit_pending_;
    bool     timeout_retransmit_pending_;

    LinkTransportStats stats_;
};
