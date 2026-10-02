#include <iostream>
#include <vector>
#include <cassert>
#include <cstring>
#include "crsf_protocol.h"
#include "transport_protocol.h"
#include "transport_engine.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

// Test 1: CRC-8 DVB-S2 computation against verified vector
bool test_crsf_crc8() {
    // Type 0x16 followed by 22 zeros
    uint8_t payload[23];
    payload[0] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memset(&payload[1], 0, 22);

    uint8_t crc = crsf_crc8(payload, 23);
    TEST_ASSERT(crc == 0xEF, "CRSF CRC-8 mismatch on all-zero vector");

    // All 0xFF channels
    memset(&payload[1], 0xFF, 22);
    uint8_t crc_ff = crsf_crc8(payload, 23);
    TEST_ASSERT(crc_ff != 0, "CRC for 0xFF vector should not be 0");

    std::cout << "  [PASS] test_crsf_crc8\n";
    return true;
}

// Test 2: Channel encoding and decoding roundtrip
bool test_crsf_channel_roundtrip() {
    uint16_t in_channels[16];
    // Fill with varied typical CRSF values (172 to 1811, 992 mid)
    for (size_t i = 0; i < 16; ++i) {
        in_channels[i] = 172 + (i * 100); // 172, 272, 372...
    }
    in_channels[0] = CRSF_CHANNEL_VALUE_MIN;  // 172
    in_channels[1] = CRSF_CHANNEL_VALUE_MID;  // 992
    in_channels[2] = CRSF_CHANNEL_VALUE_MAX;  // 1811

    uint8_t crsf_frame[26];
    size_t sz = crsf_encode_rc_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, in_channels, crsf_frame);
    TEST_ASSERT(sz == 26, "Encoded size must be 26");
    TEST_ASSERT(crsf_frame[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "Sync byte mismatch");
    TEST_ASSERT(crsf_frame[1] == 24, "Length mismatch");
    TEST_ASSERT(crsf_frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "Type mismatch");

    // Verify CRC of the generated frame
    uint8_t calc_crc = crsf_crc8(&crsf_frame[2], 23);
    TEST_ASSERT(crsf_frame[25] == calc_crc, "Frame CRC must match computed CRC");

    // Unpack and verify channel fidelity
    uint16_t out_channels[16];
    bool ok = crsf_unpack_channels22(&crsf_frame[3], out_channels);
    TEST_ASSERT(ok, "Unpack channels failed");

    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(in_channels[i] == out_channels[i], "Channel value mismatch");
    }

    std::cout << "  [PASS] test_crsf_channel_roundtrip\n";
    return true;
}

// Test 3: Streaming byte parser
bool test_crsf_parser() {
    CrsfParser parser;

    uint16_t in_channels[16];
    for (size_t i = 0; i < 16; ++i) in_channels[i] = 992;
    in_channels[3] = 1500; // Stick displaced

    uint8_t frame[26];
    crsf_encode_rc_frame(CRSF_ADDRESS_TRANSMITTER, in_channels, frame);

    // Feed leading garbage/noise bytes
    uint8_t noise[] = {0x00, 0x55, 0xAA, 0x12, 0xFF};
    for (uint8_t b : noise) {
        TEST_ASSERT(!parser.feed_byte(b), "Noise should not complete frame");
    }

    // Feed valid frame byte-by-byte
    bool completed = false;
    for (size_t i = 0; i < 26; ++i) {
        if (parser.feed_byte(frame[i])) {
            completed = true;
            TEST_ASSERT(i == 25, "Frame must complete on final CRC byte");
        }
    }
    TEST_ASSERT(completed, "Parser must complete on valid frame");
    TEST_ASSERT(parser.get_dest_addr() == CRSF_ADDRESS_TRANSMITTER, "Dest addr mismatch");
    TEST_ASSERT(parser.get_frame_type() == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "Type mismatch");
    TEST_ASSERT(parser.get_valid_frames() == 1, "Valid frames count must be 1");
    TEST_ASSERT(parser.get_crc_errors() == 0, "CRC errors must be 0");

    uint16_t decoded[16];
    TEST_ASSERT(parser.get_channels(decoded), "get_channels must succeed");
    TEST_ASSERT(decoded[3] == 1500, "Channel 3 mismatch");
    TEST_ASSERT(decoded[0] == 992, "Channel 0 mismatch");

    // Corrupted frame: flip one bit in payload
    frame[10] ^= 0x01;
    completed = false;
    for (size_t i = 0; i < 26; ++i) {
        if (parser.feed_byte(frame[i])) completed = true;
    }
    TEST_ASSERT(!completed, "Corrupted frame must be rejected");
    TEST_ASSERT(parser.get_crc_errors() == 1, "CRC error counter must increment");

    std::cout << "  [PASS] test_crsf_parser\n";
    return true;
}

// Test 4: End-to-end Ground -> RF -> Air -> FC RC pipeline roundtrip
bool test_crsf_transport_integration_roundtrip() {
    // 1. Handset generates CRSF frame
    uint16_t handset_channels[16] = {172, 992, 1811, 1000, 1500, 1600, 1700, 1800,
                                     992, 992, 992, 992, 992, 992, 992, 992};
    uint8_t handset_frame[26];
    crsf_encode_rc_frame(CRSF_ADDRESS_TRANSMITTER, handset_channels, handset_frame);

    // 2. Ground CrsfParser parses it
    CrsfParser ground_parser;
    for (size_t i = 0; i < 26; ++i) {
        ground_parser.feed_byte(handset_frame[i]);
    }
    TEST_ASSERT(ground_parser.get_valid_frames() == 1, "Ground parser failed to parse handset frame");

    // 3. Ground packs it into TransportPackedRc (39-byte RF frame)
    TransportPackedRc packed_rc{};
    // Direct zero-copy 22-byte transfer from CRSF payload to TransportPackedRc payload
    memcpy(packed_rc.channels, ground_parser.get_raw_channels22(), 22);
    packed_rc.rc_sequence = 42;
    packed_rc.flags = 0;

    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)TransportChannel::RC_CONTROL;
    hdr.flags = 0;
    hdr.sequence = 101;
    hdr.transfer_id = 0;
    hdr.fragment_offset = 0;
    hdr.payload_length = sizeof(TransportPackedRc); // 24 bytes

    uint8_t wire_frame[64];
    transport_encode_header(&hdr, wire_frame);
    memcpy(&wire_frame[TRANSPORT_HEADER_SIZE], &packed_rc, sizeof(TransportPackedRc));
    uint16_t crc = transport_crc16(wire_frame, TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc));
    transport_write_u16_be(&wire_frame[TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc)], crc);
    size_t total_wire_len = TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc) + TRANSPORT_CRC_SIZE; // 13 + 24 + 2 = 39

    TEST_ASSERT(total_wire_len == 39, "Total wire length must be exactly 39 bytes");

    // 4. Air TransportParser receives RF frame
    TransportParser air_parser(TransportNodeRole::AIR);
    size_t frames_decoded = air_parser.feed_buffer(wire_frame, total_wire_len, 1000, nullptr, nullptr);
    TEST_ASSERT(frames_decoded == 1, "Air parser failed to decode 39B RC frame");
    TEST_ASSERT(air_parser.get_header().channel == (uint8_t)TransportChannel::RC_CONTROL, "Channel must be RC_CONTROL");

    // 5. Air unpacks TransportPackedRc
    const TransportPackedRc* air_rc = reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload());
    TEST_ASSERT(air_rc->rc_sequence == 42, "rc_sequence mismatch");

    // 6. Air formats standard CRSF frame for Flight Controller
    uint16_t fc_channels[16];
    TEST_ASSERT(crsf_unpack_channels22(air_rc->channels, fc_channels), "Failed to unpack channels on Air");

    uint8_t fc_frame[26];
    size_t fc_frame_sz = crsf_encode_rc_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, fc_channels, fc_frame);
    TEST_ASSERT(fc_frame_sz == 26, "FC frame size must be 26");

    // 7. Verify Flight Controller receives identical channels
    CrsfParser fc_mock_parser;
    for (size_t i = 0; i < 26; ++i) {
        fc_mock_parser.feed_byte(fc_frame[i]);
    }
    TEST_ASSERT(fc_mock_parser.get_valid_frames() == 1, "FC failed to parse CRSF frame");

    uint16_t final_channels[16];
    TEST_ASSERT(fc_mock_parser.get_channels(final_channels), "FC failed to unpack channels");

    for (size_t ch = 0; ch < 16; ++ch) {
        TEST_ASSERT(final_channels[ch] == handset_channels[ch], "End-to-end channel fidelity mismatch");
    }

    std::cout << "  [PASS] test_crsf_transport_integration_roundtrip\n";
    return true;
}

// Test 5: Exhaustive 2,048-value CH1/CH2 decoupling and isolation test
bool test_crsf_exhaustive_ch1_ch2_independence() {
    // 5A: Sweep CH1 from 0 to 2047 while holding CH2 at neutral 992
    for (uint32_t val = 0; val <= 2047; ++val) {
        uint16_t in_ch[16];
        in_ch[0] = (uint16_t)val;
        in_ch[1] = 992; // Neutral
        for (size_t i = 2; i < 16; ++i) in_ch[i] = 1000 + i;

        uint8_t payload22[22];
        crsf_pack_channels22(in_ch, payload22);

        uint16_t out_ch[16];
        TEST_ASSERT(crsf_unpack_channels22(payload22, out_ch), "unpack failed in CH1 sweep");
        TEST_ASSERT(out_ch[0] == val, "CH1 value altered during pack/unpack");
        TEST_ASSERT(out_ch[1] == 992, "CH2 coupled/altered during CH1 sweep");
        for (size_t i = 2; i < 16; ++i) {
            TEST_ASSERT(out_ch[i] == in_ch[i], "Aux channel coupled during CH1 sweep");
        }
    }

    // 5B: Sweep CH2 from 0 to 2047 while holding CH1 at neutral 992
    for (uint32_t val = 0; val <= 2047; ++val) {
        uint16_t in_ch[16];
        in_ch[0] = 992; // Neutral
        in_ch[1] = (uint16_t)val;
        for (size_t i = 2; i < 16; ++i) in_ch[i] = 1000 + i;

        uint8_t payload22[22];
        crsf_pack_channels22(in_ch, payload22);

        uint16_t out_ch[16];
        TEST_ASSERT(crsf_unpack_channels22(payload22, out_ch), "unpack failed in CH2 sweep");
        TEST_ASSERT(out_ch[0] == 992, "CH1 coupled/altered during CH2 sweep");
        TEST_ASSERT(out_ch[1] == val, "CH2 value altered during pack/unpack");
        for (size_t i = 2; i < 16; ++i) {
            TEST_ASSERT(out_ch[i] == in_ch[i], "Aux channel coupled during CH2 sweep");
        }
    }

    std::cout << "  [PASS] test_crsf_exhaustive_ch1_ch2_independence (4,096 total sweeps verified)\n";
    return true;
}

int main() {
    std::cout << "=== Running Comprehensive CRSF Protocol & Adapter Tests ===\n";
    bool ok = true;
    ok &= test_crsf_crc8();
    ok &= test_crsf_channel_roundtrip();
    ok &= test_crsf_parser();
    ok &= test_crsf_transport_integration_roundtrip();
    ok &= test_crsf_exhaustive_ch1_ch2_independence();

    if (ok) {
        std::cout << "=== ALL CRSF PROTOCOL TESTS PASSED ===\n";
        return 0;
    } else {
        std::cerr << "=== CRSF PROTOCOL TESTS FAILED ===\n";
        return 1;
    }
}
