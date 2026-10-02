// Comprehensive Host-Side C++ Unit Tests for Dual-LRS Isolated TransportEngine
// Exercises Parser, Fragmenter, Reassembler, RetryManager, RcMailbox, and RcFailsafe.

#include <iostream>
#include <cassert>
#include <cstring>
#include <vector>
#include "transport_engine.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

// =============================================================================
// 1. TEST PARSER
// =============================================================================
bool test_parser_valid_frame() {
    TransportParser parser(TransportNodeRole::AIR);

    // Build valid MAVLINK_UPLINK frame
    uint8_t payload[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    hdr.sequence = 100;
    hdr.transfer_id = 1;
    hdr.fragment_offset = 0;
    hdr.payload_length = 10;

    uint8_t wire[64];
    transport_encode_header(&hdr, wire);
    memcpy(&wire[13], payload, 10);
    uint16_t crc = transport_crc16(wire, 23);
    transport_write_u16_be(&wire[23], crc);

    // Feed bytes one by one
    bool got_frame = false;
    uint32_t t = 1000;
    for (size_t i = 0; i < 25; ++i) {
        if (parser.feed_byte(wire[i], t)) {
            got_frame = true;
        }
        t += 1;
    }

    TEST_ASSERT(got_frame, "Parser must emit valid frame");
    TEST_ASSERT(parser.get_header().channel == (uint8_t)TransportChannel::MAVLINK_UPLINK, "Channel mismatch");
    TEST_ASSERT(parser.get_payload_length() == 10, "Payload len mismatch");
    TEST_ASSERT(memcmp(parser.get_payload(), payload, 10) == 0, "Payload data mismatch");
    TEST_ASSERT(parser.get_stats().frames_received == 1, "frames_received stat mismatch");

    std::cout << "  [PASS] test_parser_valid_frame\n";
    return true;
}

bool test_parser_invalid_direction_and_crc() {
    TransportParser air_parser(TransportNodeRole::AIR);

    // MAVLINK_DOWNLINK sent to Air unit must be rejected by direction check
    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK; // Invalid for Air
    hdr.flags = 0;
    hdr.sequence = 1;
    hdr.transfer_id = 1;
    hdr.fragment_offset = 0;
    hdr.payload_length = 5;

    uint8_t wire[64];
    transport_encode_header(&hdr, wire);
    memset(&wire[13], 'A', 5);
    uint16_t crc = transport_crc16(wire, 18);
    transport_write_u16_be(&wire[18], crc);

    bool frame = false;
    for (size_t i = 0; i < 20; ++i) {
        if (air_parser.feed_byte(wire[i], 100)) frame = true;
    }
    TEST_ASSERT(!frame, "Air parser must reject MAVLINK_DOWNLINK");
    TEST_ASSERT(air_parser.get_stats().direction_errors == 1, "direction_errors stat mismatch");

    // CRC mismatch check
    TransportParser ground_parser(TransportNodeRole::GROUND);
    hdr.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK; // Valid for Ground
    transport_encode_header(&hdr, wire);
    transport_write_u16_be(&wire[18], crc ^ 0xFFFF); // Corrupt CRC

    frame = false;
    for (size_t i = 0; i < 20; ++i) {
        if (ground_parser.feed_byte(wire[i], 200)) frame = true;
    }
    TEST_ASSERT(!frame, "Parser must reject corrupted CRC");
    TEST_ASSERT(ground_parser.get_stats().crc_errors == 1, "crc_errors stat mismatch");

    std::cout << "  [PASS] test_parser_invalid_direction_and_crc\n";
    return true;
}

// =============================================================================
// 2. TEST FRAGMENTER & REASSEMBLER ROUND-TRIP
// =============================================================================
bool test_fragmentation_and_reassembly_roundtrip() {
    // 128 bytes test message
    uint8_t original_message[128];
    for (size_t i = 0; i < 128; ++i) {
        original_message[i] = (uint8_t)(i ^ 0x5A);
    }

    TransportFragmenter fragmenter;
    TEST_ASSERT(fragmenter.start_transfer(TransportChannel::MAVLINK_DOWNLINK, original_message, 128, true), "start_transfer failed");
    TEST_ASSERT(fragmenter.get_fragment_count() == 3, "128B message must produce 3 fragments (49 + 49 + 30)");

    TransportReassembler reassembler;
    TransportParser parser(TransportNodeRole::GROUND);

    uint16_t seq = 1;
    uint32_t t = 1000;
    while (fragmenter.has_next_fragment()) {
        uint8_t frame_buf[64];
        size_t frame_len = fragmenter.get_next_fragment(frame_buf, seq++);
        TEST_ASSERT(frame_len > 0 && frame_len <= 64, "Frame len out of single-burst bounds");

        // Parse wire frame
        bool parsed = false;
        for (size_t b = 0; b < frame_len; ++b) {
            if (parser.feed_byte(frame_buf[b], t)) {
                parsed = true;
            }
        }
        TEST_ASSERT(parsed, "Parser must decode fragment frame");

        TransportNackReason nack = reassembler.process_fragment(parser.get_header(), parser.get_payload(), t);
        TEST_ASSERT(nack == TransportNackReason::NONE, "Fragment must be accepted");
        t += 10;
    }

    TEST_ASSERT(reassembler.is_complete(), "Reassembler must be complete");
    TEST_ASSERT(reassembler.get_reassembled_length() == 128, "Reassembled length must be 128");
    TEST_ASSERT(memcmp(reassembler.get_reassembled_data(), original_message, 128) == 0, "Reassembled data mismatch");

    std::cout << "  [PASS] test_fragmentation_and_reassembly_roundtrip (128B message -> 3 fragments -> reassembly)\n";
    return true;
}

bool test_reassembler_out_of_order() {
    TransportReassembler reassembler;

    // Chunk 0: [0..10)
    TransportHeader h0{};
    h0.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h0.transfer_id = 42;
    h0.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h0.fragment_offset = 0;
    h0.payload_length = 10;
    const uint8_t p0[] = "0123456789";

    // Chunk 2: [20..25), LAST_FRAG
    TransportHeader h2{};
    h2.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h2.transfer_id = 42;
    h2.flags = TRANSPORT_FLAG_LAST_FRAG;
    h2.fragment_offset = 20;
    h2.payload_length = 5;
    const uint8_t p2[] = "ABCDE";

    // Chunk 1: [10..20)
    TransportHeader h1{};
    h1.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h1.transfer_id = 42;
    h1.flags = 0;
    h1.fragment_offset = 10;
    h1.payload_length = 10;
    const uint8_t p1[] = "KLMNOPQRST";

    // Feed chunk 0
    TEST_ASSERT(reassembler.process_fragment(h0, p0, 100) == TransportNackReason::NONE, "h0 failed");
    TEST_ASSERT(!reassembler.is_complete(), "h0 should not complete");

    // Feed chunk 2 out of order (chunk 1 still missing)
    TEST_ASSERT(reassembler.process_fragment(h2, p2, 110) == TransportNackReason::NONE, "h2 failed");
    TEST_ASSERT(!reassembler.is_complete(), "h2 should not complete without chunk 1");

    // Feed chunk 1
    TEST_ASSERT(reassembler.process_fragment(h1, p1, 120) == TransportNackReason::NONE, "h1 failed");
    TEST_ASSERT(reassembler.is_complete(), "Should complete after chunk 1 arrives");
    TEST_ASSERT(reassembler.get_reassembled_length() == 25, "Length mismatch");

    const char* expected = "0123456789KLMNOPQRSTABCDE";
    TEST_ASSERT(memcmp(reassembler.get_reassembled_data(), expected, 25) == 0, "Data mismatch");

    std::cout << "  [PASS] test_reassembler_out_of_order\n";
    return true;
}

bool test_reassembler_overlap_and_active_preservation() {
    TransportReassembler reassembler;

    // Start transfer 10: chunk [0..20)
    TransportHeader h10{};
    h10.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h10.transfer_id = 10;
    h10.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h10.fragment_offset = 0;
    h10.payload_length = 20;
    uint8_t p10[20];
    memset(p10, 'A', 20);
    TEST_ASSERT(reassembler.process_fragment(h10, p10, 100) == TransportNackReason::NONE, "h10 failed");

    // Attempt to preempt with transfer 20: must be rejected with BUFFER_FULL
    TransportHeader h20 = h10;
    h20.transfer_id = 20;
    TransportNackReason r_preempt = reassembler.process_fragment(h20, p10, 110);
    TEST_ASSERT(r_preempt == TransportNackReason::BUFFER_FULL, "Must reject new transfer with BUFFER_FULL");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 10, "Transfer 10 must be preserved");

    // Partial overlap within transfer 10: chunk [15..35)
    TransportHeader h_ov = h10;
    h_ov.flags = 0;
    h_ov.fragment_offset = 15;
    h_ov.payload_length = 20;
    uint8_t p_ov[20];
    memset(p_ov, 'Z', 20);
    TransportNackReason r_ov = reassembler.process_fragment(h_ov, p_ov, 120);
    TEST_ASSERT(r_ov == TransportNackReason::OVERLAP_CONFLICT, "Must reject partial overlap with OVERLAP_CONFLICT");
    TEST_ASSERT(reassembler.get_reassembled_data()[15] == 'A', "Buffer bytes must not be overwritten");

    // Exact duplicate: chunk [0..20) again -> NONE (re-ACKed)
    TransportNackReason r_dup = reassembler.process_fragment(h10, p10, 130);
    TEST_ASSERT(r_dup == TransportNackReason::NONE, "Exact duplicate must be accepted idempotently");

    std::cout << "  [PASS] test_reassembler_overlap_and_active_preservation\n";
    return true;
}

bool test_reassembler_bounds_and_overflow_safety() {
    TransportReassembler reassembler;

    // Verify reassembly buffer is completely zeroed initially
    for (size_t i = 0; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Buffer not initialized to 0");
    }

    // 1. Boundary case: fragment_offset = 512, payload_length = 1
    // Arithmetic: 512 + 1 = 513 > 512 -> OFFSET_OVERRUN
    TransportHeader h_512{};
    h_512.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_512.transfer_id = 1;
    h_512.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_512.fragment_offset = 512;
    h_512.payload_length = 1;
    uint8_t p_canary = 0xAA;
    TransportNackReason r1 = reassembler.process_fragment(h_512, &p_canary, 100);
    TEST_ASSERT(r1 == TransportNackReason::OFFSET_OVERRUN, "offset=512, len=1 must return OFFSET_OVERRUN");
    TEST_ASSERT(!reassembler.has_active_transfer(), "Must not activate transfer on overrun");
    for (size_t i = 0; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Buffer corrupted by offset=512");
    }

    // 2. Boundary case: fragment_offset = 65535, payload_length = 1 (16-bit wrapping attack)
    // Arithmetic: 65535 + 1 = 65536 (wraps to 0 in 16-bit) -> must be caught by 32-bit math as OFFSET_OVERRUN
    TransportHeader h_wrap{};
    h_wrap.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_wrap.transfer_id = 1;
    h_wrap.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_wrap.fragment_offset = 65535;
    h_wrap.payload_length = 1;
    TransportNackReason r2 = reassembler.process_fragment(h_wrap, &p_canary, 100);
    TEST_ASSERT(r2 == TransportNackReason::OFFSET_OVERRUN, "offset=65535, len=1 must return OFFSET_OVERRUN");
    TEST_ASSERT(!reassembler.has_active_transfer(), "Must not activate transfer on 16-bit wrap overrun");
    for (size_t i = 0; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Buffer corrupted by offset=65535");
    }

    // 3. Boundary case: fragment_offset = 500, payload_length = 13
    // Arithmetic: 500 + 13 = 513 > 512 -> OFFSET_OVERRUN
    TransportHeader h_500{};
    h_500.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_500.transfer_id = 1;
    h_500.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_500.fragment_offset = 500;
    h_500.payload_length = 13;
    uint8_t p_13[13];
    memset(p_13, 0xBB, sizeof(p_13));
    TransportNackReason r3 = reassembler.process_fragment(h_500, p_13, 100);
    TEST_ASSERT(r3 == TransportNackReason::OFFSET_OVERRUN, "offset=500, len=13 must return OFFSET_OVERRUN");
    TEST_ASSERT(!reassembler.has_active_transfer(), "Must not activate transfer on 500+13 overrun");
    for (size_t i = 0; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Buffer corrupted by offset=500, len=13");
    }

    // 4. Null payload rejection with payload_length > 0
    TransportHeader h_null{};
    h_null.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_null.transfer_id = 1;
    h_null.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_null.fragment_offset = 0;
    h_null.payload_length = 20;
    TransportNackReason r_null = reassembler.process_fragment(h_null, nullptr, 100);
    TEST_ASSERT(r_null == TransportNackReason::BAD_CRC, "Null payload with len>0 must be rejected");
    TEST_ASSERT(!reassembler.has_active_transfer(), "Must not activate transfer on null payload");
    for (size_t i = 0; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Buffer corrupted by null payload");
    }

    // 5. Zero payload_length rejection
    TransportHeader h_zero{};
    h_zero.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_zero.transfer_id = 1;
    h_zero.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_zero.fragment_offset = 0;
    h_zero.payload_length = 0;
    TransportNackReason r_zero = reassembler.process_fragment(h_zero, p_13, 100);
    TEST_ASSERT(r_zero == TransportNackReason::BAD_CRC, "Zero payload_length must be rejected");

    // 6. Valid boundary case: fragment_offset = 0, payload_length = 49
    // Must succeed, activate transfer, write exactly bytes [0..49), leaving [49..512) zeroed
    TransportHeader h_valid{};
    h_valid.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h_valid.transfer_id = 1;
    h_valid.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h_valid.fragment_offset = 0;
    h_valid.payload_length = 49;
    uint8_t p_49[49];
    memset(p_49, 0xCC, sizeof(p_49));
    TransportNackReason r_valid = reassembler.process_fragment(h_valid, p_49, 100);
    TEST_ASSERT(r_valid == TransportNackReason::NONE, "offset=0, len=49 must be accepted");
    TEST_ASSERT(reassembler.has_active_transfer(), "Transfer must be active");
    TEST_ASSERT(memcmp(reassembler.get_reassembled_data(), p_49, 49) == 0, "Payload data [0..49) mismatch");
    for (size_t i = 49; i < TRANSPORT_MAX_TRANSFER_SIZE; ++i) {
        TEST_ASSERT(reassembler.get_reassembled_data()[i] == 0, "Trailing buffer [49..512) must remain 0");
    }

    // 7. Fragmenter channel restriction validation
    TransportFragmenter frag;
    uint8_t dummy[50] = {0};
    TEST_ASSERT(!frag.start_transfer(TransportChannel::RC_CONTROL, dummy, sizeof(dummy)),
                "Fragmenter must reject RC_CONTROL");
    TEST_ASSERT(!frag.start_transfer(TransportChannel::LINK_CONTROL, dummy, sizeof(dummy)),
                "Fragmenter must reject LINK_CONTROL");
    TEST_ASSERT(frag.start_transfer(TransportChannel::MAVLINK_DOWNLINK, dummy, sizeof(dummy)),
                "Fragmenter must accept MAVLINK_DOWNLINK");
    frag.reset();
    TEST_ASSERT(frag.start_transfer(TransportChannel::MAVLINK_UPLINK, dummy, sizeof(dummy)),
                "Fragmenter must accept MAVLINK_UPLINK");

    std::cout << "  [PASS] test_reassembler_bounds_and_overflow_safety\n";
    return true;
}

bool test_reassembler_sequential_transfers_and_lifecycle() {
    TransportReassembler reassembler;

    // --- Part A: Single-fragment sequential transfers ---
    // 1. Send transfer ID 1 (FIRST_FRAG | LAST_FRAG, 10 bytes)
    TransportHeader h1{};
    h1.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h1.transfer_id = 1;
    h1.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    h1.fragment_offset = 0;
    h1.payload_length = 10;
    const uint8_t p1[] = "HELLO_0001";
    TEST_ASSERT(reassembler.process_fragment(h1, p1, 100) == TransportNackReason::NONE, "Transfer 1 failed");
    TEST_ASSERT(reassembler.is_complete(), "Transfer 1 must be complete");
    TEST_ASSERT(reassembler.has_active_transfer(), "Transfer 1 must be active");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 1, "Transfer 1 ID must match");
    TEST_ASSERT(reassembler.get_reassembled_length() == 10, "Transfer 1 length mismatch");
    TEST_ASSERT(memcmp(reassembler.get_reassembled_data(), p1, 10) == 0, "Transfer 1 data mismatch");
    TEST_ASSERT(reassembler.get_stats().transfers_completed == 1, "Must have 1 completed transfer");

    // 2. Duplicate fragment of completed transfer 1: must be re-ACKed idempotently without modifying state
    TEST_ASSERT(reassembler.process_fragment(h1, p1, 110) == TransportNackReason::NONE, "Duplicate of completed transfer failed");
    TEST_ASSERT(reassembler.get_stats().transfers_completed == 1, "Completed count must remain 1");

    // 3. Send transfer ID 2 (FIRST_FRAG | LAST_FRAG, 10 bytes): must atomically replace completed transfer 1
    TransportHeader h2 = h1;
    h2.transfer_id = 2;
    const uint8_t p2[] = "WORLD_0002";
    TEST_ASSERT(reassembler.process_fragment(h2, p2, 120) == TransportNackReason::NONE, "Transfer 2 failed");
    TEST_ASSERT(reassembler.is_complete(), "Transfer 2 must be complete");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 2, "Transfer 2 ID must match");
    TEST_ASSERT(reassembler.get_reassembled_length() == 10, "Transfer 2 length mismatch");
    TEST_ASSERT(memcmp(reassembler.get_reassembled_data(), p2, 10) == 0, "Transfer 2 data mismatch");
    TEST_ASSERT(reassembler.get_stats().transfers_completed == 2, "Must have 2 completed transfers");

    // 4. Test explicit mark_complete_consumed()
    reassembler.mark_complete_consumed();
    TEST_ASSERT(!reassembler.has_active_transfer(), "has_active_transfer must be false after consumption");
    TEST_ASSERT(!reassembler.is_complete(), "is_complete must be false after consumption");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 0, "active_transfer_id must be 0 after consumption");

    // --- Part B: Multi-fragment sequential transfers ---
    // 5. Transfer 3: Fragment 0 of 2 (49 bytes, FIRST_FRAG)
    TransportHeader h3_0{};
    h3_0.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h3_0.transfer_id = 3;
    h3_0.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h3_0.fragment_offset = 0;
    h3_0.payload_length = 49;
    uint8_t p3_0[49];
    memset(p3_0, '3', sizeof(p3_0));
    TEST_ASSERT(reassembler.process_fragment(h3_0, p3_0, 200) == TransportNackReason::NONE, "Transfer 3 frag 0 failed");
    TEST_ASSERT(reassembler.has_active_transfer(), "Transfer 3 must be active");
    TEST_ASSERT(!reassembler.is_complete(), "Transfer 3 must not be complete yet");

    // 6. Preemption attempt with Transfer 4 while Transfer 3 is incomplete: must be rejected with BUFFER_FULL
    TransportHeader h4_preempt = h3_0;
    h4_preempt.transfer_id = 4;
    TEST_ASSERT(reassembler.process_fragment(h4_preempt, p3_0, 210) == TransportNackReason::BUFFER_FULL,
                "Incomplete transfer 3 must reject transfer 4 with BUFFER_FULL");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 3, "Transfer 3 must remain active");

    // 7. Transfer 3: Fragment 1 of 2 (20 bytes, LAST_FRAG) -> completes Transfer 3 (total 69 bytes)
    TransportHeader h3_1{};
    h3_1.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h3_1.transfer_id = 3;
    h3_1.flags = TRANSPORT_FLAG_LAST_FRAG;
    h3_1.fragment_offset = 49;
    h3_1.payload_length = 20;
    uint8_t p3_1[20];
    memset(p3_1, 'X', sizeof(p3_1));
    TEST_ASSERT(reassembler.process_fragment(h3_1, p3_1, 220) == TransportNackReason::NONE, "Transfer 3 frag 1 failed");
    TEST_ASSERT(reassembler.is_complete(), "Transfer 3 must now be complete");
    TEST_ASSERT(reassembler.get_reassembled_length() == 69, "Transfer 3 length must be 69");
    TEST_ASSERT(reassembler.get_stats().transfers_completed == 3, "Must have 3 completed transfers");

    // 8. Transfer 4: Fragment 0 arrives without explicit mark_complete_consumed()
    // Since Transfer 3 is complete, Transfer 4 with FIRST_FRAG must atomically take over
    TransportHeader h4_0{};
    h4_0.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h4_0.transfer_id = 4;
    h4_0.flags = TRANSPORT_FLAG_FIRST_FRAG;
    h4_0.fragment_offset = 0;
    h4_0.payload_length = 49;
    uint8_t p4_0[49];
    memset(p4_0, '4', sizeof(p4_0));
    TEST_ASSERT(reassembler.process_fragment(h4_0, p4_0, 230) == TransportNackReason::NONE, "Transfer 4 frag 0 failed");
    TEST_ASSERT(reassembler.get_active_transfer_id() == 4, "Transfer 4 must be active ID");
    TEST_ASSERT(!reassembler.is_complete(), "Transfer 4 must not be complete yet");

    // 9. Transfer 4: Fragment 1 (30 bytes, LAST_FRAG) -> completes Transfer 4 (total 79 bytes)
    TransportHeader h4_1{};
    h4_1.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h4_1.transfer_id = 4;
    h4_1.flags = TRANSPORT_FLAG_LAST_FRAG;
    h4_1.fragment_offset = 49;
    h4_1.payload_length = 30;
    uint8_t p4_1[30];
    memset(p4_1, 'Y', sizeof(p4_1));
    TEST_ASSERT(reassembler.process_fragment(h4_1, p4_1, 240) == TransportNackReason::NONE, "Transfer 4 frag 1 failed");
    TEST_ASSERT(reassembler.is_complete(), "Transfer 4 must be complete");
    TEST_ASSERT(reassembler.get_reassembled_length() == 79, "Transfer 4 length must be 79");
    TEST_ASSERT(reassembler.get_stats().transfers_completed == 4, "Must have 4 completed transfers");

    // 10. Transfer 5 arrives without FIRST_FRAG (e.g. middle fragment): must return GAP_DETECTED
    TransportHeader h5_mid{};
    h5_mid.channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK;
    h5_mid.transfer_id = 5;
    h5_mid.flags = 0;
    h5_mid.fragment_offset = 49;
    h5_mid.payload_length = 20;
    TEST_ASSERT(reassembler.process_fragment(h5_mid, p4_1, 250) == TransportNackReason::GAP_DETECTED,
                "Transfer 5 missing FIRST_FRAG must return GAP_DETECTED");

    std::cout << "  [PASS] test_reassembler_sequential_transfers_and_lifecycle\n";
    return true;
}

// =============================================================================
// 3. TEST RETRY STATE MACHINE
// =============================================================================
bool test_retry_manager() {
    TransportRetryManager retry_mgr;
    uint8_t dummy_frame[30] = {1, 2, 3};

    // Arm with frame at t = 1000 ms
    retry_mgr.arm(dummy_frame, 30, 77, 0, 1000);
    TEST_ASSERT(retry_mgr.get_state() == RetryState::WAITING_ACK, "Must be WAITING_ACK");

    // At t = 1150 ms (dt = 150 ms < 200 ms RTO): no timeout
    TEST_ASSERT(!retry_mgr.check_retry_timeout(1150), "Must not timeout before 200ms");

    // At t = 1201 ms (dt = 201 ms >= 200 ms RTO): retry 1 triggered
    TEST_ASSERT(retry_mgr.check_retry_timeout(1201), "Must trigger retry 1 at 200ms");
    TEST_ASSERT(retry_mgr.get_retry_count() == 1, "retry_count must be 1");

    // Trigger retries 2, 3, 4, 5
    TEST_ASSERT(retry_mgr.check_retry_timeout(1402), "Retry 2");
    TEST_ASSERT(retry_mgr.check_retry_timeout(1603), "Retry 3");
    TEST_ASSERT(retry_mgr.check_retry_timeout(1804), "Retry 4");
    TEST_ASSERT(retry_mgr.check_retry_timeout(2005), "Retry 5");
    TEST_ASSERT(retry_mgr.get_retry_count() == 5, "retry_count must be 5");

    // 6th timeout: exhausts retries and aborts
    TEST_ASSERT(!retry_mgr.check_retry_timeout(2206), "Retry 6 must abort");
    TEST_ASSERT(retry_mgr.get_state() == RetryState::EXHAUSTED, "Must be EXHAUSTED");
    TEST_ASSERT(retry_mgr.get_stats().transfer_aborts == 1, "transfer_aborts must be 1");

    // Test ACK arrival clears retry
    retry_mgr.arm(dummy_frame, 30, 88, 49, 3000);
    TransportLinkAckNack ack{};
    ack.transfer_id = 88;
    ack.fragment_offset = 49;
    ack.nack_reason = (uint8_t)TransportNackReason::NONE;
    TEST_ASSERT(retry_mgr.process_ack_nack(ack), "Matching ACK must clear retry");
    TEST_ASSERT(retry_mgr.get_state() == RetryState::IDLE, "Must be IDLE after ACK");

    std::cout << "  [PASS] test_retry_manager (200ms RTO, 5-retry limit, ACK clearing)\n";
    return true;
}

// =============================================================================
// 4. TEST RC MAILBOX & FAILSAFE STATE MACHINE
// =============================================================================
bool test_rc_mailbox_and_failsafe() {
    TransportRcMailbox mailbox;
    TransportRcFailsafe failsafe(500); // 500 ms timeout

    TEST_ASSERT(!mailbox.has_data(), "Mailbox starts empty");
    TEST_ASSERT(failsafe.get_state() == RcFailsafeState::NO_SIGNAL, "Failsafe starts NO_SIGNAL");

    // Pack and update RC frame at t = 1000 ms
    uint16_t in_channels[16];
    for (int i = 0; i < 16; ++i) in_channels[i] = 1000 + i * 50;
    uint8_t wire_rc[24];
    TEST_ASSERT(transport_encode_packed_rc(in_channels, 5, 0, wire_rc), "encode packed rc failed");

    TEST_ASSERT(mailbox.update_from_wire(wire_rc), "mailbox update failed");
    TEST_ASSERT(mailbox.has_data(), "mailbox must have data");

    uint16_t out_channels[16];
    uint8_t seq = 0, flags = 0;
    mailbox.read_channels(out_channels, &seq, &flags);
    TEST_ASSERT(seq == 5, "seq mismatch");
    TEST_ASSERT(memcmp(in_channels, out_channels, sizeof(in_channels)) == 0, "channels mismatch");

    // Record frame arrival at failsafe
    failsafe.record_frame_arrival(1000);
    failsafe.update(1050);
    TEST_ASSERT(failsafe.get_state() == RcFailsafeState::ACTIVE, "Must be ACTIVE at dt=50ms");

    // dt = 150 ms: DEGRADED
    failsafe.update(1150);
    TEST_ASSERT(failsafe.get_state() == RcFailsafeState::DEGRADED, "Must be DEGRADED at dt=150ms");

    // dt = 501 ms: FAILSAFE ASSERTED
    failsafe.update(1501);
    TEST_ASSERT(failsafe.is_failsafe_active(), "Must be FAILSAFE at dt=501ms");

    // Recovery requires 3 consecutive valid frames
    failsafe.record_frame_arrival(1600); // Frame 1
    failsafe.update(1601);
    TEST_ASSERT(failsafe.is_failsafe_active(), "Must remain in failsafe after 1 frame");

    failsafe.record_frame_arrival(1690); // Frame 2
    failsafe.update(1691);
    TEST_ASSERT(failsafe.is_failsafe_active(), "Must remain in failsafe after 2 frames");

    failsafe.record_frame_arrival(1780); // Frame 3
    failsafe.update(1781);
    TEST_ASSERT(!failsafe.is_failsafe_active(), "Failsafe must clear after 3 consecutive frames");
    TEST_ASSERT(failsafe.get_state() == RcFailsafeState::ACTIVE, "Must be ACTIVE");

    std::cout << "  [PASS] test_rc_mailbox_and_failsafe (mailbox overwrite, 500ms timeout, 3-frame restore)\n";
    return true;
}

int main() {
    std::cout << "=== Running Comprehensive TransportEngine Unit Tests ===\n";
    bool ok = true;
    ok &= test_parser_valid_frame();
    ok &= test_parser_invalid_direction_and_crc();
    ok &= test_fragmentation_and_reassembly_roundtrip();
    ok &= test_reassembler_out_of_order();
    ok &= test_reassembler_overlap_and_active_preservation();
    ok &= test_reassembler_bounds_and_overflow_safety();
    ok &= test_reassembler_sequential_transfers_and_lifecycle();
    ok &= test_retry_manager();
    ok &= test_rc_mailbox_and_failsafe();

    if (ok) {
        std::cout << "=== ALL TRANSPORT ENGINE TESTS PASSED ===\n";
        return 0;
    } else {
        std::cerr << "=== SOME TRANSPORT ENGINE TESTS FAILED ===\n";
        return 1;
    }
}
