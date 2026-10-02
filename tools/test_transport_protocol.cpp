// Direct C++ Host-Side Verification Suite for Dual-LRS Transport Protocol Helpers
// Validates C++ big-endian wire serialization, RC bit packing, NACK codes, and CRC.

#include <iostream>
#include <cassert>
#include <cstring>
#include <vector>
#include "transport_protocol.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

bool test_header_serialization() {
    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)TransportChannel::MAVLINK_UPLINK;
    hdr.flags = TRANSPORT_FLAG_RELIABLE | TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG; // 0x19
    hdr.sequence = 0x1234;
    hdr.transfer_id = 0x5678;
    hdr.fragment_offset = 0x0031;
    hdr.payload_length = 0x002A;

    uint8_t wire[13] = {0};
    transport_encode_header(&hdr, wire);

    // Assert exact big-endian wire bytes
    const uint8_t expected[13] = {
        0x44, 0x4C,       // magic0, magic1
        0x01,             // version
        0x03,             // channel
        0x19,             // flags
        0x12, 0x34,       // sequence (MSB, LSB)
        0x56, 0x78,       // transfer_id (MSB, LSB)
        0x00, 0x31,       // fragment_offset (MSB, LSB)
        0x00, 0x2A        // payload_length (MSB, LSB)
    };
    TEST_ASSERT(std::memcmp(wire, expected, 13) == 0, "Header wire bytes mismatch");

    // Round-trip decode
    TransportHeader dec{};
    bool ok = transport_decode_header(wire, &dec);
    TEST_ASSERT(ok, "transport_decode_header failed");
    TEST_ASSERT(dec.magic0 == 0x44, "magic0 mismatch");
    TEST_ASSERT(dec.magic1 == 0x4C, "magic1 mismatch");
    TEST_ASSERT(dec.version == 0x01, "version mismatch");
    TEST_ASSERT(dec.channel == 0x03, "channel mismatch");
    TEST_ASSERT(dec.flags == 0x19, "flags mismatch");
    TEST_ASSERT(dec.sequence == 0x1234, "sequence mismatch");
    TEST_ASSERT(dec.transfer_id == 0x5678, "transfer_id mismatch");
    TEST_ASSERT(dec.fragment_offset == 0x0031, "fragment_offset mismatch");
    TEST_ASSERT(dec.payload_length == 0x002A, "payload_length mismatch");

    std::cout << "  [PASS] test_header_serialization (exact byte verification)\n";
    return true;
}

bool test_heartbeat_serialization() {
    TransportLinkHeartbeat hb{};
    hb.subtype = (uint8_t)LinkSubtype::HEARTBEAT; // 0x01
    hb.link_quality = 98;                          // 0x62
    hb.rssi = -75;                                 // 0xB5
    hb.snr = 10;                                   // 0x0A
    hb.rtt_ms = 0x01A4;                            // 420 ms

    uint8_t wire[6] = {0};
    transport_encode_link_heartbeat(&hb, wire);

    const uint8_t expected[6] = {
        0x01,       // subtype
        0x62,       // link_quality
        0xB5,       // rssi (-75 in 2's complement)
        0x0A,       // snr
        0x01, 0xA4  // rtt_ms (MSB, LSB)
    };
    TEST_ASSERT(std::memcmp(wire, expected, 6) == 0, "Heartbeat wire bytes mismatch");

    TransportLinkHeartbeat dec{};
    bool ok = transport_decode_link_heartbeat(wire, &dec);
    TEST_ASSERT(ok, "transport_decode_link_heartbeat failed");
    TEST_ASSERT(dec.subtype == 0x01, "hb subtype mismatch");
    TEST_ASSERT(dec.link_quality == 98, "hb link_quality mismatch");
    TEST_ASSERT(dec.rssi == -75, "hb rssi mismatch");
    TEST_ASSERT(dec.snr == 10, "hb snr mismatch");
    TEST_ASSERT(dec.rtt_ms == 420, "hb rtt_ms mismatch");

    std::cout << "  [PASS] test_heartbeat_serialization (exact byte verification)\n";
    return true;
}

bool test_ack_nack_serialization() {
    TransportLinkAckNack an{};
    an.subtype = (uint8_t)LinkSubtype::ACK_NACK; // 0x02
    an.target_channel = (uint8_t)TransportChannel::MAVLINK_DOWNLINK; // 0x04
    an.transfer_id = 0xA1B2;
    an.fragment_offset = 0x0092;
    an.nack_reason = (uint8_t)TransportNackReason::OVERLAP_CONFLICT; // 0x06
    an.reserved = 0x00;

    uint8_t wire[8] = {0};
    transport_encode_link_ack_nack(&an, wire);

    const uint8_t expected[8] = {
        0x02,       // subtype
        0x04,       // target_channel
        0xA1, 0xB2, // transfer_id (MSB, LSB)
        0x00, 0x92, // fragment_offset (MSB, LSB)
        0x06,       // nack_reason (OVERLAP_CONFLICT)
        0x00        // reserved
    };
    TEST_ASSERT(std::memcmp(wire, expected, 8) == 0, "ACK/NACK wire bytes mismatch");

    TransportLinkAckNack dec{};
    bool ok = transport_decode_link_ack_nack(wire, &dec);
    TEST_ASSERT(ok, "transport_decode_link_ack_nack failed");
    TEST_ASSERT(dec.subtype == 0x02, "an subtype mismatch");
    TEST_ASSERT(dec.target_channel == 0x04, "an target_channel mismatch");
    TEST_ASSERT(dec.transfer_id == 0xA1B2, "an transfer_id mismatch");
    TEST_ASSERT(dec.fragment_offset == 0x0092, "an fragment_offset mismatch");
    TEST_ASSERT(dec.nack_reason == (uint8_t)TransportNackReason::OVERLAP_CONFLICT, "an nack_reason mismatch");
    TEST_ASSERT(dec.reserved == 0x00, "an reserved mismatch");

    TEST_ASSERT(transport_is_valid_nack_reason((uint8_t)TransportNackReason::OVERLAP_CONFLICT), "OVERLAP_CONFLICT should be valid");

    std::cout << "  [PASS] test_ack_nack_serialization (exact byte verification with OVERLAP_CONFLICT)\n";
    return true;
}

bool test_rc_bit_packing() {
    // 1. All zero
    uint16_t ch_zero[16] = {0};
    uint8_t wire_zero[22] = {0};
    bool ok0 = transport_pack_rc_channels(ch_zero, wire_zero);
    TEST_ASSERT(ok0, "pack zero failed");
    for (int i = 0; i < 22; ++i) {
        TEST_ASSERT(wire_zero[i] == 0x00, "wire_zero must be all 0x00");
    }
    uint16_t out_zero[16] = {0};
    TEST_ASSERT(transport_unpack_rc_channels(wire_zero, out_zero), "unpack zero failed");
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT(out_zero[i] == 0, "out_zero channel must be 0");
    }

    // 2. All maximum (2047 = 0x7FF)
    uint16_t ch_max[16];
    for (int i = 0; i < 16; ++i) ch_max[i] = 2047;
    uint8_t wire_max[22] = {0};
    TEST_ASSERT(transport_pack_rc_channels(ch_max, wire_max), "pack max failed");
    for (int i = 0; i < 22; ++i) {
        TEST_ASSERT(wire_max[i] == 0xFF, "wire_max must be all 0xFF");
    }
    uint16_t out_max[16] = {0};
    TEST_ASSERT(transport_unpack_rc_channels(wire_max, out_max), "unpack max failed");
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT(out_max[i] == 2047, "out_max channel must be 2047");
    }

    // 3. Alternating 0 and 2047
    uint16_t ch_alt[16];
    for (int i = 0; i < 16; ++i) ch_alt[i] = (i % 2 == 0) ? 0 : 2047;
    uint8_t wire_alt[22] = {0};
    TEST_ASSERT(transport_pack_rc_channels(ch_alt, wire_alt), "pack alt failed");
    uint16_t out_alt[16] = {0};
    TEST_ASSERT(transport_unpack_rc_channels(wire_alt, out_alt), "unpack alt failed");
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT(out_alt[i] == ch_alt[i], "out_alt channel mismatch");
    }

    // 4. Boundaries
    // Channel 0 only at 2047 -> Byte 0 = 0xFF, Byte 1 = 0x07, rest 0x00
    uint16_t ch_b0[16] = {0};
    ch_b0[0] = 2047;
    uint8_t wire_b0[22] = {0};
    TEST_ASSERT(transport_pack_rc_channels(ch_b0, wire_b0), "pack b0 failed");
    TEST_ASSERT(wire_b0[0] == 0xFF, "b0 byte 0 mismatch");
    TEST_ASSERT(wire_b0[1] == 0x07, "b0 byte 1 mismatch");
    for (int i = 2; i < 22; ++i) {
        TEST_ASSERT(wire_b0[i] == 0x00, "b0 remaining bytes mismatch");
    }

    // Channel 1 only at 2047 -> Byte 1 = 0xF8, Byte 2 = 0x3F, rest 0x00
    uint16_t ch_b1[16] = {0};
    ch_b1[1] = 2047;
    uint8_t wire_b1[22] = {0};
    TEST_ASSERT(transport_pack_rc_channels(ch_b1, wire_b1), "pack b1 failed");
    TEST_ASSERT(wire_b1[0] == 0x00, "b1 byte 0 mismatch");
    TEST_ASSERT(wire_b1[1] == 0xF8, "b1 byte 1 mismatch");
    TEST_ASSERT(wire_b1[2] == 0x3F, "b1 byte 2 mismatch");

    // Channel 15 only at 2047 -> Byte 20 = 0xE0, Byte 21 = 0xFF, rest 0x00
    uint16_t ch_b15[16] = {0};
    ch_b15[15] = 2047;
    uint8_t wire_b15[22] = {0};
    TEST_ASSERT(transport_pack_rc_channels(ch_b15, wire_b15), "pack b15 failed");
    for (int i = 0; i < 20; ++i) {
        TEST_ASSERT(wire_b15[i] == 0x00, "b15 byte 0..19 mismatch");
    }
    TEST_ASSERT(wire_b15[20] == 0xE0, "b15 byte 20 mismatch");
    TEST_ASSERT(wire_b15[21] == 0xFF, "b15 byte 21 mismatch");

    // 5. Clamping vs Rejection
    uint16_t ch_out_of_range[16] = {1000};
    ch_out_of_range[5] = 3000; // > 2047
    uint8_t wire_oor[22] = {0};

    // clamp = false -> must reject
    bool ok_reject = transport_pack_rc_channels(ch_out_of_range, wire_oor, false);
    TEST_ASSERT(!ok_reject, "pack with clamp=false must reject >2047");

    // clamp = true -> must clamp to 2047
    bool ok_clamp = transport_pack_rc_channels(ch_out_of_range, wire_oor, true);
    TEST_ASSERT(ok_clamp, "pack with clamp=true must succeed");
    uint16_t out_oor[16] = {0};
    TEST_ASSERT(transport_unpack_rc_channels(wire_oor, out_oor), "unpack oor failed");
    TEST_ASSERT(out_oor[5] == 2047, "channel 5 must be clamped to 2047");

    std::cout << "  [PASS] test_rc_bit_packing (exact byte verification, boundaries, clamping)\n";
    return true;
}

bool test_crc16_and_direction() {
    const uint8_t test_data[] = { 'D', 'L', 0x01, 0x03, 0x00, 0x00, 0x01 };
    uint16_t crc = transport_crc16(test_data, sizeof(test_data));
    TEST_ASSERT(crc != 0, "CRC should not be zero");

    // Direction validation
    TEST_ASSERT(transport_is_valid_direction(TransportNodeRole::AIR, (uint8_t)TransportChannel::RC_CONTROL), "Air must accept RC");
    TEST_ASSERT(transport_is_valid_direction(TransportNodeRole::AIR, (uint8_t)TransportChannel::MAVLINK_UPLINK), "Air must accept MAVLINK_UPLINK");
    TEST_ASSERT(!transport_is_valid_direction(TransportNodeRole::AIR, (uint8_t)TransportChannel::MAVLINK_DOWNLINK), "Air must reject MAVLINK_DOWNLINK");

    TEST_ASSERT(transport_is_valid_direction(TransportNodeRole::GROUND, (uint8_t)TransportChannel::MAVLINK_DOWNLINK), "Ground must accept MAVLINK_DOWNLINK");
    TEST_ASSERT(!transport_is_valid_direction(TransportNodeRole::GROUND, (uint8_t)TransportChannel::RC_CONTROL), "Ground must reject RC_CONTROL");
    TEST_ASSERT(!transport_is_valid_direction(TransportNodeRole::GROUND, (uint8_t)TransportChannel::MAVLINK_UPLINK), "Ground must reject MAVLINK_UPLINK");

    std::cout << "  [PASS] test_crc16_and_direction\n";
    return true;
}

int main() {
    std::cout << "=== Running C++ Dual-LRS Transport Protocol Unit Tests ===\n";
    bool all_ok = true;
    all_ok &= test_header_serialization();
    all_ok &= test_heartbeat_serialization();
    all_ok &= test_ack_nack_serialization();
    all_ok &= test_rc_bit_packing();
    all_ok &= test_crc16_and_direction();

    if (all_ok) {
        std::cout << "=== ALL C++ TRANSPORT TESTS PASSED ===\n";
        return 0;
    } else {
        std::cerr << "=== SOME C++ TRANSPORT TESTS FAILED ===\n";
        return 1;
    }
}
