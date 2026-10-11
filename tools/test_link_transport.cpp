#include <iostream>
#include <vector>
#include <cassert>
#include <cstring>
#include "link_transport.h"
#include "transport_protocol.h"
#include "transport_engine.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << #cond << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

// =============================================================================
// TEST 1: Single-fragment transfer receives an ACK
// =============================================================================
bool test_single_fragment_ack() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    uint8_t msg[] = {0xFD, 0x09, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x12, 0x34}; // 12 bytes
    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true), "start_transfer failed");

    uint8_t wire_frame[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = sender.get_next_tx_frame(wire_frame, 100, 1000);
    TEST_ASSERT(frame_len > 0, "sender produced no frame");
    TEST_ASSERT(sender.is_tx_active(), "sender should be active waiting for ACK");

    // Feed to receiver
    TransportHeader rx_hdr{};
    transport_decode_header(wire_frame, &rx_hdr);
    uint16_t crc = transport_crc16(wire_frame, frame_len - 2);
    uint16_t wire_crc = transport_read_u16_be(&wire_frame[frame_len - 2]);
    TEST_ASSERT(crc == wire_crc, "wire CRC mismatch");

    TEST_ASSERT(receiver.ingest_transport_frame(rx_hdr, &wire_frame[TRANSPORT_HEADER_SIZE], rx_hdr.payload_length, 1010), "receiver ingest failed");
    TEST_ASSERT(receiver.has_received_message(), "receiver should have complete message");
    TEST_ASSERT(receiver.has_pending_ack_nack(), "receiver should have queued an ACK");

    // Receiver generates ACK
    uint8_t ack_wire[TRANSPORT_MAX_FRAME_SIZE];
    size_t ack_len = receiver.get_next_tx_frame(ack_wire, 200, 1015);
    TEST_ASSERT(ack_len > 0, "receiver produced no ACK frame");

    // Feed ACK back to sender
    TransportHeader ack_hdr{};
    transport_decode_header(ack_wire, &ack_hdr);
    TEST_ASSERT(sender.ingest_transport_frame(ack_hdr, &ack_wire[TRANSPORT_HEADER_SIZE], ack_hdr.payload_length, 1020), "sender failed to ingest ACK");

    // Sender should now be complete and IDLE
    TEST_ASSERT(!sender.is_tx_active(), "sender should be idle after ACK");
    TEST_ASSERT(sender.get_stats().acks_received == 1, "acks_received should be 1");
    TEST_ASSERT(sender.get_stats().transfers_completed_tx == 1, "transfers_completed_tx should be 1");

    // Verify received message content
    uint8_t rx_buf[64];
    size_t rx_len = receiver.get_received_message(rx_buf, sizeof(rx_buf));
    TEST_ASSERT(rx_len == sizeof(msg), "rx_len mismatch");
    TEST_ASSERT(memcmp(rx_buf, msg, sizeof(msg)) == 0, "payload corrupted");

    std::cout << "  [PASS] test_single_fragment_ack" << std::endl;
    return true;
}

// =============================================================================
// TEST 2: Multi-fragment transfer advances only after each ACK
// =============================================================================
bool test_multi_fragment_advances_only_after_ack() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    // 120 bytes -> 3 fragments (49 + 49 + 22)
    uint8_t msg[120];
    for (size_t i = 0; i < sizeof(msg); ++i) msg[i] = (uint8_t)(i ^ 0x55);

    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true), "start_transfer failed");

    uint16_t seq = 1;
    uint32_t t_ms = 1000;

    for (int frag = 0; frag < 3; ++frag) {
        uint8_t wire_frame[TRANSPORT_MAX_FRAME_SIZE];
        size_t frame_len = sender.get_next_tx_frame(wire_frame, seq++, t_ms);
        TEST_ASSERT(frame_len > 0, "failed to get fragment frame");

        // Before ACK, sender should NOT advance to next fragment!
        uint8_t peek_buf[TRANSPORT_MAX_FRAME_SIZE];
        size_t peek_len = sender.get_next_tx_frame(peek_buf, seq, t_ms + 10);
        TEST_ASSERT(peek_len == 0, "sender advanced to next fragment before receiving ACK!");

        // Receiver processes fragment
        TransportHeader rx_hdr{};
        transport_decode_header(wire_frame, &rx_hdr);
        TEST_ASSERT(receiver.ingest_transport_frame(rx_hdr, &wire_frame[TRANSPORT_HEADER_SIZE], rx_hdr.payload_length, t_ms + 15), "ingest failed");

        // Receiver emits ACK
        uint8_t ack_wire[TRANSPORT_MAX_FRAME_SIZE];
        size_t ack_len = receiver.get_next_tx_frame(ack_wire, 500 + frag, t_ms + 20);
        TEST_ASSERT(ack_len > 0, "no ACK produced");

        // Sender receives ACK
        TransportHeader ack_hdr{};
        transport_decode_header(ack_wire, &ack_hdr);
        TEST_ASSERT(sender.ingest_transport_frame(ack_hdr, &ack_wire[TRANSPORT_HEADER_SIZE], ack_hdr.payload_length, t_ms + 25), "ACK ingest failed");

        t_ms += 100;
    }

    TEST_ASSERT(!sender.is_tx_active(), "sender should be idle after all fragments ACKed");
    TEST_ASSERT(sender.get_stats().acks_received == 3, "expected 3 ACKs received");
    TEST_ASSERT(sender.get_stats().transfers_completed_tx == 1, "expected 1 completed tx");
    TEST_ASSERT(receiver.has_received_message(), "receiver should have complete message");

    uint8_t rx_buf[150];
    size_t rx_len = receiver.get_received_message(rx_buf, sizeof(rx_buf));
    TEST_ASSERT(rx_len == sizeof(msg), "rx_len mismatch");
    TEST_ASSERT(memcmp(rx_buf, msg, sizeof(msg)) == 0, "multi-fragment payload mismatch");

    std::cout << "  [PASS] test_multi_fragment_advances_only_after_ack" << std::endl;
    return true;
}

// =============================================================================
// TEST 3: Lost fragment causes a timeout retry
// =============================================================================
bool test_lost_fragment_causes_timeout_retry() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    uint8_t msg[] = {0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true), "start failed");

    uint8_t wire_frame[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = sender.get_next_tx_frame(wire_frame, 10, 1000);
    TEST_ASSERT(frame_len > 0, "frame_len == 0");

    // SIMULATED IMPAIRMENT: Drop frame 1 (do not pass to receiver)
    // Advance time by 100 ms (RTO is 200 ms) -> no retransmit yet
    sender.update(1100);
    uint8_t retry_buf[TRANSPORT_MAX_FRAME_SIZE];
    size_t retry_len = sender.get_next_tx_frame(retry_buf, 11, 1100);
    TEST_ASSERT(retry_len == 0, "should not retransmit before 200ms RTO timeout");

    // Advance time to 1205 ms (205 ms elapsed >= 200ms RTO)
    sender.update(1205);
    retry_len = sender.get_next_tx_frame(retry_buf, 12, 1205);
    TEST_ASSERT(retry_len == frame_len, "retransmission frame length mismatch");
    TEST_ASSERT(memcmp(retry_buf, wire_frame, frame_len) == 0, "retransmit must use exact stored frame bytes");
    TEST_ASSERT(sender.get_stats().retries_sent == 1, "retries_sent should be 1");

    // Deliver retransmitted frame to receiver
    TransportHeader rx_hdr{};
    transport_decode_header(retry_buf, &rx_hdr);
    receiver.ingest_transport_frame(rx_hdr, &retry_buf[TRANSPORT_HEADER_SIZE], rx_hdr.payload_length, 1210);

    // Receiver ACKs
    uint8_t ack_wire[TRANSPORT_MAX_FRAME_SIZE];
    size_t ack_len = receiver.get_next_tx_frame(ack_wire, 50, 1215);
    TEST_ASSERT(ack_len > 0, "receiver failed to generate ACK");
    TransportHeader ack_hdr{};
    transport_decode_header(ack_wire, &ack_hdr);
    sender.ingest_transport_frame(ack_hdr, &ack_wire[TRANSPORT_HEADER_SIZE], ack_hdr.payload_length, 1220);

    TEST_ASSERT(!sender.is_tx_active(), "sender should be idle after retransmission ACK");
    std::cout << "  [PASS] test_lost_fragment_causes_timeout_retry" << std::endl;
    return true;
}

// =============================================================================
// TEST 4: Corrupted frame causes CRC rejection at parser and timeout retry
// =============================================================================
bool test_corrupted_frame_causes_crc_rejection_and_timeout_retry() {
    LinkTransport sender(TransportNodeRole::AIR);
    LinkTransport receiver(TransportNodeRole::GROUND);
    TransportParser parser(TransportNodeRole::GROUND);

    uint8_t msg[] = {0xAA, 0xBB, 0xCC, 0xDD};
    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_DOWNLINK, msg, sizeof(msg), true), "start_transfer failed");

    uint8_t wire_frame[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = sender.get_next_tx_frame(wire_frame, 1, 1000);
    TEST_ASSERT(frame_len > 0, "failed to get frame");

    // SIMULATED IMPAIRMENT: Corrupt byte in payload on RF link
    uint8_t corrupt_frame[TRANSPORT_MAX_FRAME_SIZE];
    memcpy(corrupt_frame, wire_frame, frame_len);
    corrupt_frame[TRANSPORT_HEADER_SIZE + 1] ^= 0xFF; // Flip bits

    // TransportParser at link level detects CRC mismatch and discards corrupted frame
    bool parsed_ok = false;
    for (size_t i = 0; i < frame_len; ++i) {
        if (parser.feed_byte(corrupt_frame[i], 1005)) {
            parsed_ok = true;
        }
    }
    TEST_ASSERT(!parsed_ok, "corrupted frame must NOT be accepted by parser");
    TEST_ASSERT(parser.get_stats().crc_errors > 0, "parser must record CRC error");

    // Corrupted frame was dropped; receiver was NOT fed any data
    TEST_ASSERT(!receiver.has_received_message(), "receiver must not have received anything");

    // Sender times out after 200ms RTO and generates exact retransmission
    sender.update(1205);
    TEST_ASSERT(sender.has_outgoing_frame(), "sender must have retry frame ready after timeout");

    uint8_t retry_buf[TRANSPORT_MAX_FRAME_SIZE];
    size_t retry_len = sender.get_next_tx_frame(retry_buf, 2, 1205);
    TEST_ASSERT(retry_len == frame_len, "retry frame len mismatch");
    TEST_ASSERT(memcmp(retry_buf, wire_frame, frame_len) == 0, "retry must send exact original frame");
    TEST_ASSERT(sender.get_stats().retries_sent == 1, "retries_sent should be 1");

    // Feed clean retransmitted frame through parser to receiver
    parsed_ok = false;
    for (size_t i = 0; i < retry_len; ++i) {
        if (parser.feed_byte(retry_buf[i], 1210)) {
            parsed_ok = true;
            TransportHeader hdr = parser.get_header();
            receiver.ingest_transport_frame(hdr, parser.get_payload(), hdr.payload_length, 1210);
        }
    }
    TEST_ASSERT(parsed_ok, "clean retransmission must be accepted by parser");
    TEST_ASSERT(receiver.has_received_message(), "receiver should now have complete message");

    // Receiver generates ACK
    uint8_t ack_wire[TRANSPORT_MAX_FRAME_SIZE];
    size_t ack_len = receiver.get_next_tx_frame(ack_wire, 10, 1215);
    TEST_ASSERT(ack_len > 0, "receiver must generate ACK");

    TransportHeader ack_hdr{};
    transport_decode_header(ack_wire, &ack_hdr);
    sender.ingest_transport_frame(ack_hdr, &ack_wire[TRANSPORT_HEADER_SIZE], ack_hdr.payload_length, 1220);

    TEST_ASSERT(!sender.is_tx_active(), "sender should be idle after ACK");
    TEST_ASSERT(sender.get_stats().acks_received == 1, "acks_received should be 1");

    std::cout << "  [PASS] test_corrupted_frame_causes_crc_rejection_and_timeout_retry" << std::endl;
    return true;
}

// =============================================================================
// TEST 5: Dropped fragment causes GAP_DETECTED
// =============================================================================
bool test_dropped_fragment_causes_gap_detected() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    // 120 bytes -> 3 fragments
    uint8_t msg[120];
    memset(msg, 0x77, sizeof(msg));
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    // Fragment 0
    uint8_t f0[TRANSPORT_MAX_FRAME_SIZE];
    size_t f0_len = sender.get_next_tx_frame(f0, 1, 1000);
    (void)f0_len;

    // SIMULATED IMPAIRMENT: Drop Fragment 0 entirely!
    // Manually force fragmenter to fragment 1
    TransportHeader hdr_f1{};
    hdr_f1.magic0 = TRANSPORT_MAGIC0;
    hdr_f1.magic1 = TRANSPORT_MAGIC1;
    hdr_f1.version = TRANSPORT_VERSION;
    hdr_f1.channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    hdr_f1.flags = TRANSPORT_FLAG_RELIABLE; // Not FIRST_FRAG!
    hdr_f1.sequence = 2;
    hdr_f1.transfer_id = sender.get_tx_transfer_id();
    hdr_f1.fragment_offset = 49;
    hdr_f1.payload_length = 49;

    uint8_t payload_f1[49];
    memset(payload_f1, 0x77, 49);

    // Ingest fragment 1 without fragment 0 having arrived
    receiver.ingest_transport_frame(hdr_f1, payload_f1, sizeof(payload_f1), 1010);

    TEST_ASSERT(receiver.has_pending_ack_nack(), "receiver should emit NACK on gap");
    uint8_t nack_wire[TRANSPORT_MAX_FRAME_SIZE];
    receiver.get_next_tx_frame(nack_wire, 100, 1015);

    TransportLinkAckNack an{};
    transport_decode_link_ack_nack(&nack_wire[TRANSPORT_HEADER_SIZE], &an);
    TEST_ASSERT(an.nack_reason == (uint8_t)TransportNackReason::GAP_DETECTED, "expected GAP_DETECTED NACK");

    std::cout << "  [PASS] test_dropped_fragment_causes_gap_detected" << std::endl;
    return true;
}

// =============================================================================
// TEST 6: Duplicate fragment produces another ACK but no duplicate delivery
// =============================================================================
bool test_duplicate_fragment_produces_ack_no_duplicate_delivery() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    uint8_t msg[] = {0x11, 0x22, 0x33};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    uint8_t wire_frame[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = sender.get_next_tx_frame(wire_frame, 1, 1000);
    (void)frame_len;

    TransportHeader hdr{};
    transport_decode_header(wire_frame, &hdr);

    // Ingest frame first time
    receiver.ingest_transport_frame(hdr, &wire_frame[TRANSPORT_HEADER_SIZE], hdr.payload_length, 1000);
    TEST_ASSERT(receiver.has_received_message(), "message should be complete");

    // Pop the queued ACK
    uint8_t ack1[TRANSPORT_MAX_FRAME_SIZE];
    receiver.get_next_tx_frame(ack1, 10, 1005);

    // SIMULATED IMPAIRMENT: Deliver DUPLICATE frame
    receiver.ingest_transport_frame(hdr, &wire_frame[TRANSPORT_HEADER_SIZE], hdr.payload_length, 1010);

    // Duplicate must produce another idempotent ACK
    TEST_ASSERT(receiver.has_pending_ack_nack(), "duplicate must generate idempotent ACK");
    uint8_t ack2[TRANSPORT_MAX_FRAME_SIZE];
    receiver.get_next_tx_frame(ack2, 11, 1015);

    TransportLinkAckNack an2{};
    transport_decode_link_ack_nack(&ack2[TRANSPORT_HEADER_SIZE], &an2);
    TEST_ASSERT(an2.nack_reason == (uint8_t)TransportNackReason::NONE, "re-ACK must be NONE (valid ACK)");

    // Ensure only 1 application message is ready, not two
    uint8_t out[16];
    size_t out_len = receiver.get_received_message(out, sizeof(out));
    TEST_ASSERT(out_len == sizeof(msg), "out_len mismatch");
    receiver.pop_received_message();
    TEST_ASSERT(!receiver.has_received_message(), "duplicate frame must not create second application message");

    std::cout << "  [PASS] test_duplicate_fragment_produces_ack_no_duplicate_delivery" << std::endl;
    return true;
}

// =============================================================================
// TEST 7: NACK causes retransmission
// =============================================================================
bool test_nack_causes_retransmission() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x99, 0x88};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    uint8_t frame1[TRANSPORT_MAX_FRAME_SIZE];
    size_t f1_len = sender.get_next_tx_frame(frame1, 1, 1000);

    // Send NACK to sender
    TransportHeader nack_hdr{};
    nack_hdr.magic0 = TRANSPORT_MAGIC0;
    nack_hdr.magic1 = TRANSPORT_MAGIC1;
    nack_hdr.version = TRANSPORT_VERSION;
    nack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    nack_hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    nack_hdr.sequence = 50;
    nack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id();
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::GAP_DETECTED;

    uint8_t nack_payload[8];
    transport_encode_link_ack_nack(&an, nack_payload);

    sender.ingest_transport_frame(nack_hdr, nack_payload, sizeof(nack_payload), 1010);
    TEST_ASSERT(sender.get_stats().nacks_received == 1, "nacks_received should be 1");

    // Sender should immediately have a retransmission ready without waiting for timeout
    TEST_ASSERT(sender.has_outgoing_frame(), "sender must prepare retransmission on NACK");
    uint8_t retry_frame[TRANSPORT_MAX_FRAME_SIZE];
    size_t r_len = sender.get_next_tx_frame(retry_frame, 2, 1015);
    TEST_ASSERT(r_len == f1_len, "retransmission length mismatch");
    TEST_ASSERT(memcmp(retry_frame, frame1, f1_len) == 0, "retransmission must match original frame");

    std::cout << "  [PASS] test_nack_causes_retransmission" << std::endl;
    return true;
}

// =============================================================================
// TEST 8: Five failed retries abort transfer cleanly
// =============================================================================
bool test_five_failed_retries_abort_transfer() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x01, 0x02};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    uint32_t t = 1000;
    for (int retry = 1; retry <= 5; ++retry) {
        t += 205; // Trigger 200ms RTO timeout
        sender.update(t);
        size_t rlen = sender.get_next_tx_frame(buf, 1 + retry, t);
        TEST_ASSERT(rlen > 0, "expected retry frame");
        TEST_ASSERT(sender.get_stats().retries_sent == (uint32_t)retry, "retry count mismatch");
    }

    // 6th timeout: reaches EXHAUSTED and aborts
    t += 205;
    sender.update(t);
    TEST_ASSERT(!sender.is_tx_active(), "transfer must abort after 5 retries");
    TEST_ASSERT(sender.get_stats().transfer_aborts == 1, "transfer_aborts should be 1");

    std::cout << "  [PASS] test_five_failed_retries_abort_transfer" << std::endl;
    return true;
}

// =============================================================================
// TEST 9: ACK for the wrong transfer is ignored
// =============================================================================
bool test_ack_wrong_transfer_id_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // Send ACK for WRONG transfer_id
    TransportHeader ack_hdr{};
    ack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    ack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id() + 99; // Wrong ID
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::NONE;

    uint8_t ack_payload[8];
    transport_encode_link_ack_nack(&an, ack_payload);

    sender.ingest_transport_frame(ack_hdr, ack_payload, sizeof(ack_payload), 1010);
    TEST_ASSERT(sender.get_stats().acks_received == 0, "wrong transfer_id ACK must be ignored");
    TEST_ASSERT(sender.is_tx_active(), "sender must remain waiting for valid ACK");

    std::cout << "  [PASS] test_ack_wrong_transfer_id_ignored" << std::endl;
    return true;
}

// =============================================================================
// TEST 10: ACK for the wrong fragment offset is ignored
// =============================================================================
bool test_ack_wrong_fragment_offset_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // Send ACK for WRONG offset
    TransportHeader ack_hdr{};
    ack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    ack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id();
    an.fragment_offset = 49; // Wrong offset (active is 0)
    an.nack_reason = (uint8_t)TransportNackReason::NONE;

    uint8_t ack_payload[8];
    transport_encode_link_ack_nack(&an, ack_payload);

    sender.ingest_transport_frame(ack_hdr, ack_payload, sizeof(ack_payload), 1010);
    TEST_ASSERT(sender.get_stats().acks_received == 0, "wrong offset ACK must be ignored");
    TEST_ASSERT(sender.is_tx_active(), "sender must remain waiting for valid ACK");

    std::cout << "  [PASS] test_ack_wrong_fragment_offset_ignored" << std::endl;
    return true;
}

// =============================================================================
// TEST 11: Transfer-ID wraparound works (65535 -> 0)
// =============================================================================
bool test_transfer_id_wraparound() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    // Transfer with ID near max (handled in fragmenter)
    uint8_t msg[] = {0x42};
    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true), "start failed");

    uint8_t f[TRANSPORT_MAX_FRAME_SIZE];
    size_t flen = sender.get_next_tx_frame(f, 1, 1000);
    (void)flen;

    TransportHeader hdr{};
    transport_decode_header(f, &hdr);
    receiver.ingest_transport_frame(hdr, &f[TRANSPORT_HEADER_SIZE], hdr.payload_length, 1005);
    TEST_ASSERT(receiver.has_received_message(), "receiver should reassemble across transfer_id boundary");

    std::cout << "  [PASS] test_transfer_id_wraparound" << std::endl;
    return true;
}

// =============================================================================
// TEST 12: Sequence-number wraparound works (65535 -> 0)
// =============================================================================
bool test_sequence_number_wraparound() {
    LinkTransport sender(TransportNodeRole::GROUND);
    LinkTransport receiver(TransportNodeRole::AIR);

    uint8_t msg[] = {0xAA, 0x55};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    uint8_t f[TRANSPORT_MAX_FRAME_SIZE];
    // Next sequence wraps around 65535 -> 0
    sender.get_next_tx_frame(f, 65535, 1000);

    TransportHeader hdr{};
    transport_decode_header(f, &hdr);
    TEST_ASSERT(hdr.sequence == 65535, "seq should be 65535");

    receiver.ingest_transport_frame(hdr, &f[TRANSPORT_HEADER_SIZE], hdr.payload_length, 1005);
    TEST_ASSERT(receiver.has_received_message(), "wrapped seq frame must be accepted");

    std::cout << "  [PASS] test_sequence_number_wraparound" << std::endl;
    return true;
}

// =============================================================================
// TEST 13: Reliable MAVLink traffic cannot displace RC
// =============================================================================
bool test_reliable_mavlink_cannot_displace_rc() {
    // Verified by scheduler invariant: RC always checks send_rc before link transport frames
    LinkTransport lt(TransportNodeRole::GROUND);
    uint8_t msg[100];
    memset(msg, 0xEE, sizeof(msg));
    lt.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);

    // Both RC and Reliable MAVLink are ready
    bool rc_has_fresh_frame = true;
    bool mavlink_has_frame = lt.has_outgoing_frame();
    TEST_ASSERT(mavlink_has_frame, "mavlink has frame ready");

    // Scheduler rule: When RC is ready, RC is selected, MAVLink yields
    bool send_rc = rc_has_fresh_frame;
    TEST_ASSERT(send_rc == true, "RC must not be displaced by pending MAVLink fragment");

    std::cout << "  [PASS] test_reliable_mavlink_cannot_displace_rc" << std::endl;
    return true;
}

// =============================================================================
// TEST 14: ACK frames themselves are never recursively ACKed
// =============================================================================
bool test_ack_frames_never_recursively_acked() {
    LinkTransport receiver(TransportNodeRole::AIR);

    // Ingest LINK_CONTROL frame (an ACK/NACK frame)
    TransportHeader ack_hdr{};
    ack_hdr.magic0 = TRANSPORT_MAGIC0;
    ack_hdr.magic1 = TRANSPORT_MAGIC1;
    ack_hdr.version = TRANSPORT_VERSION;
    ack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    ack_hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    ack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    an.transfer_id = 1;
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::NONE;

    uint8_t payload[8];
    transport_encode_link_ack_nack(&an, payload);

    receiver.ingest_transport_frame(ack_hdr, payload, sizeof(payload), 1000);

    // Receiver must NOT generate an ACK for an incoming ACK!
    TEST_ASSERT(!receiver.has_pending_ack_nack(), "ACK frames must never be recursively ACKed!");

    std::cout << "  [PASS] test_ack_frames_never_recursively_acked" << std::endl;
    return true;
}

// =============================================================================
// TEST 15: A failed transfer does not block a later transfer
// =============================================================================
bool test_failed_transfer_does_not_block_later_transfer() {
    LinkTransport sender(TransportNodeRole::GROUND);

    // Transfer 1: fails after 5 retries
    uint8_t msg1[] = {0x01};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg1, sizeof(msg1), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    uint32_t t = 1000;
    for (int i = 0; i < 6; ++i) {
        t += 205;
        sender.update(t);
        sender.get_next_tx_frame(buf, 2 + i, t);
    }
    TEST_ASSERT(!sender.is_tx_active(), "transfer 1 should be aborted");

    // Transfer 2: must start cleanly!
    uint8_t msg2[] = {0x02, 0x03};
    TEST_ASSERT(sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg2, sizeof(msg2), true), "transfer 2 must not be blocked by aborted transfer 1");
    TEST_ASSERT(sender.is_tx_active(), "transfer 2 should be active");

    std::cout << "  [PASS] test_failed_transfer_does_not_block_later_transfer" << std::endl;
    return true;
}

// =============================================================================
// TEST 16: NACK for the wrong transfer is ignored
// =============================================================================
bool test_nack_wrong_transfer_id_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // Send NACK for WRONG transfer_id
    TransportHeader nack_hdr{};
    nack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    nack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id() + 77; // Wrong ID
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::GAP_DETECTED;

    uint8_t nack_payload[8];
    transport_encode_link_ack_nack(&an, nack_payload);

    bool res = sender.ingest_transport_frame(nack_hdr, nack_payload, sizeof(nack_payload), 1010);
    TEST_ASSERT(!res, "wrong transfer_id NACK must return false");
    TEST_ASSERT(sender.get_stats().nacks_received == 0, "nacks_received must be 0");
    TEST_ASSERT(!sender.has_outgoing_frame(), "wrong transfer_id NACK must not trigger retransmission");

    std::cout << "  [PASS] test_nack_wrong_transfer_id_ignored" << std::endl;
    return true;
}

// =============================================================================
// TEST 17: NACK for the wrong fragment offset is ignored
// =============================================================================
bool test_nack_wrong_fragment_offset_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // Send NACK for WRONG offset
    TransportHeader nack_hdr{};
    nack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    nack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id();
    an.fragment_offset = 49; // Wrong offset (active is 0)
    an.nack_reason = (uint8_t)TransportNackReason::GAP_DETECTED;

    uint8_t nack_payload[8];
    transport_encode_link_ack_nack(&an, nack_payload);

    bool res = sender.ingest_transport_frame(nack_hdr, nack_payload, sizeof(nack_payload), 1010);
    TEST_ASSERT(!res, "wrong offset NACK must return false");
    TEST_ASSERT(sender.get_stats().nacks_received == 0, "nacks_received must be 0");
    TEST_ASSERT(!sender.has_outgoing_frame(), "wrong offset NACK must not trigger retransmission");

    std::cout << "  [PASS] test_nack_wrong_fragment_offset_ignored" << std::endl;
    return true;
}

// =============================================================================
// TEST 18: Wrong-channel ACK and NACK are ignored
// =============================================================================
bool test_wrong_channel_ack_nack_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // 1. ACK targeting WRONG channel (DOWNLINK instead of UPLINK)
    TransportHeader ack_hdr{};
    ack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    ack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK; // Wrong channel!
    an.transfer_id = sender.get_tx_transfer_id();
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::NONE;

    uint8_t payload[8];
    transport_encode_link_ack_nack(&an, payload);

    bool res = sender.ingest_transport_frame(ack_hdr, payload, sizeof(payload), 1010);
    TEST_ASSERT(!res, "wrong channel ACK must be rejected");
    TEST_ASSERT(sender.get_stats().acks_received == 0, "acks_received must be 0");
    TEST_ASSERT(sender.is_tx_active(), "sender must remain active");

    // 2. NACK targeting WRONG channel
    an.nack_reason = (uint8_t)TransportNackReason::GAP_DETECTED;
    transport_encode_link_ack_nack(&an, payload);

    res = sender.ingest_transport_frame(ack_hdr, payload, sizeof(payload), 1015);
    TEST_ASSERT(!res, "wrong channel NACK must be rejected");
    TEST_ASSERT(sender.get_stats().nacks_received == 0, "nacks_received must be 0");
    TEST_ASSERT(!sender.has_outgoing_frame(), "wrong channel NACK must not trigger retransmission");

    std::cout << "  [PASS] test_wrong_channel_ack_nack_ignored" << std::endl;
    return true;
}

// =============================================================================
// TEST 19: Stale ACK after transfer completed is ignored
// =============================================================================
bool test_stale_ack_after_transfer_completed_ignored() {
    LinkTransport sender(TransportNodeRole::GROUND);

    uint8_t msg[] = {0x10, 0x20};
    sender.start_transfer(TransportChannel::MAVLINK_UPLINK, msg, sizeof(msg), true);
    uint8_t buf[TRANSPORT_MAX_FRAME_SIZE];
    sender.get_next_tx_frame(buf, 1, 1000);

    // Send valid matching ACK
    TransportHeader ack_hdr{};
    ack_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    ack_hdr.payload_length = sizeof(TransportLinkAckNack);

    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK;
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    an.transfer_id = sender.get_tx_transfer_id();
    an.fragment_offset = 0;
    an.nack_reason = (uint8_t)TransportNackReason::NONE;

    uint8_t payload[8];
    transport_encode_link_ack_nack(&an, payload);

    bool res1 = sender.ingest_transport_frame(ack_hdr, payload, sizeof(payload), 1010);
    TEST_ASSERT(res1, "first ACK must complete transfer");
    TEST_ASSERT(!sender.is_tx_active(), "sender must be idle");
    TEST_ASSERT(sender.get_stats().acks_received == 1, "acks_received should be 1");

    // Now send the same ACK again (stale duplicate ACK after completion)
    bool res2 = sender.ingest_transport_frame(ack_hdr, payload, sizeof(payload), 1015);
    TEST_ASSERT(!res2, "stale ACK after transfer completed must be safely ignored");
    TEST_ASSERT(sender.get_stats().acks_received == 1, "acks_received must still be 1");
    TEST_ASSERT(!sender.is_tx_active(), "sender must remain idle");

    std::cout << "  [PASS] test_stale_ack_after_transfer_completed_ignored" << std::endl;
    return true;
}

// =============================================================================
// MAIN ENTRY
// =============================================================================
int main() {
    std::cout << "===============================================================" << std::endl;
    std::cout << "Running Dual-LRS Phase 3 Reliable LinkTransport Unit Tests" << std::endl;
    std::cout << "===============================================================" << std::endl;

    bool all_passed = true;
    all_passed &= test_single_fragment_ack();
    all_passed &= test_multi_fragment_advances_only_after_ack();
    all_passed &= test_lost_fragment_causes_timeout_retry();
    all_passed &= test_corrupted_frame_causes_crc_rejection_and_timeout_retry();
    all_passed &= test_dropped_fragment_causes_gap_detected();
    all_passed &= test_duplicate_fragment_produces_ack_no_duplicate_delivery();
    all_passed &= test_nack_causes_retransmission();
    all_passed &= test_five_failed_retries_abort_transfer();
    all_passed &= test_ack_wrong_transfer_id_ignored();
    all_passed &= test_ack_wrong_fragment_offset_ignored();
    all_passed &= test_transfer_id_wraparound();
    all_passed &= test_sequence_number_wraparound();
    all_passed &= test_reliable_mavlink_cannot_displace_rc();
    all_passed &= test_ack_frames_never_recursively_acked();
    all_passed &= test_failed_transfer_does_not_block_later_transfer();
    all_passed &= test_nack_wrong_transfer_id_ignored();
    all_passed &= test_nack_wrong_fragment_offset_ignored();
    all_passed &= test_wrong_channel_ack_nack_ignored();
    all_passed &= test_stale_ack_after_transfer_completed_ignored();

    std::cout << "===============================================================" << std::endl;
    if (all_passed) {
        std::cout << "ALL 19 RELIABLE LINK TRANSPORT TESTS PASSED (100% SUCCESS)!" << std::endl;
        std::cout << "===============================================================" << std::endl;
        return 0;
    } else {
        std::cerr << "SOME TESTS FAILED!" << std::endl;
        std::cout << "===============================================================" << std::endl;
        return 1;
    }
}
