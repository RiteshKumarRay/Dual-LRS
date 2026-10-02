#include <iostream>
#include <vector>
#include <cstring>
#include "rc_adapter.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

bool test_rc_adapter_pipeline() {
    RcGroundAdapter ground_adapter;
    RcAirAdapter air_adapter;

    // 1. Simulate handset sending 16 channels
    uint16_t handset_ch[16];
    for (size_t i = 0; i < 16; ++i) handset_ch[i] = 1000 + (i * 50);
    uint8_t crsf_in[26];
    crsf_encode_rc_frame(CRSF_ADDRESS_TRANSMITTER, handset_ch, crsf_in);

    // Feed handset bytes to Ground adapter
    for (size_t i = 0; i < 26; ++i) {
        ground_adapter.feed_byte(crsf_in[i]);
    }
    TEST_ASSERT(ground_adapter.get_stats().crsf_frames_in == 1, "Ground failed to parse CRSF frame");
    TEST_ASSERT(ground_adapter.has_new_frame(), "has_new_frame should be true");

    // 2. Ground generates 39-byte uplink RF frame
    uint8_t rf_frame[64];
    size_t rf_len = ground_adapter.get_uplink_frame(rf_frame, 1);
    TEST_ASSERT(rf_len == 39, "RF frame length must be 39 bytes");
    TEST_ASSERT(ground_adapter.get_stats().rf_frames_sent == 1, "rf_frames_sent mismatch");

    // 3. Air parser receives RF frame
    TransportParser air_parser(TransportNodeRole::AIR);
    size_t decoded = air_parser.feed_buffer(rf_frame, rf_len, 1000, nullptr, nullptr);
    TEST_ASSERT(decoded == 1, "Air failed to decode RF frame");

    const TransportPackedRc* packed_rc = reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload());
    TEST_ASSERT(air_adapter.ingest_rc_frame(*packed_rc, 1000), "Air failed to ingest RC frame");

    // Verify channel fidelity on Air
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(air_adapter.get_channels()[i] == handset_ch[i], "Channel value mismatch on Air");
    }

    // 4. Initial frame arrives from NO_SIGNAL: transitions to ACTIVE
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "First valid frame must activate link from NO_SIGNAL");

    // Air generates 26-byte CRSF frame for FC
    uint8_t fc_out[26];
    size_t fc_len = air_adapter.get_fc_frame(fc_out);
    TEST_ASSERT(fc_len == 26, "FC frame length must be 26 when link is ACTIVE");
    TEST_ASSERT(fc_out[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "FC sync byte mismatch");

    // Unpack FC frame and verify fidelity
    uint16_t fc_ch[16];
    TEST_ASSERT(crsf_unpack_channels22(&fc_out[3], fc_ch), "Failed to unpack FC channels");
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(fc_ch[i] == handset_ch[i], "FC channel mismatch");
    }

    // 5. Test Failsafe timeout: simulate RF link loss (> 500 ms)
    air_adapter.update(1550); // t = 1550 ms (dt = 550 ms > 500 ms)
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must be active after 500ms timeout");
    TEST_ASSERT(air_adapter.get_stats().failsafe_events == 1, "Failsafe event count mismatch");

    // FC pulses must be suppressed ("No Pulses" policy)
    TEST_ASSERT(air_adapter.get_fc_frame(fc_out) == 0, "FC frame must be suppressed during failsafe");

    // 6. Test 3-frame restoration from FAILSAFE
    // Restore frame 1 at t = 1600 ms: still in failsafe (consecutive = 1 < 3)
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 2);
    air_parser.feed_buffer(rf_frame, rf_len, 1600, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1600);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe should remain active after 1st restore frame");

    // Restore frame 2 at t = 1620 ms: still in failsafe (consecutive = 2 < 3)
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 3);
    air_parser.feed_buffer(rf_frame, rf_len, 1620, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1620);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe should remain active after 2nd restore frame");

    // Restore frame 3 at t = 1640 ms: failsafe cleared! (consecutive = 3 >= 3)
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 4);
    air_parser.feed_buffer(rf_frame, rf_len, 1640, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1640);
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Failsafe must clear after 3 consecutive frames");
    TEST_ASSERT(air_adapter.get_stats().restore_events == 1, "Restore event count mismatch");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_out) == 26, "FC pulses must resume once link is restored");

    std::cout << "  [PASS] test_rc_adapter_pipeline\n";
    return true;
}

bool test_sbus_ingestion() {
    RcGroundAdapter ground_adapter;
    ground_adapter.enable_sbus(true);
    RcAirAdapter air_adapter;

    // Simulate 25-byte SBUS frame: 0x0F, 22 bytes packed channels, 0x00 flags, 0x00 end
    uint16_t handset_ch[16];
    for (size_t i = 0; i < 16; ++i) handset_ch[i] = 992 + (i * 20); // ~1500 us centered
    uint8_t raw22[22];
    transport_pack_rc_channels(handset_ch, raw22);

    uint8_t sbus_frame[25];
    sbus_frame[0] = 0x0F;
    memcpy(&sbus_frame[1], raw22, 22);
    sbus_frame[23] = 0x00; // flags
    sbus_frame[24] = 0x00; // end byte

    // Feed to ground adapter
    for (size_t i = 0; i < 25; ++i) {
        ground_adapter.feed_byte(sbus_frame[i]);
    }

    TEST_ASSERT(ground_adapter.get_stats().crsf_frames_in == 1, "Ground failed to parse SBUS frame");
    TEST_ASSERT(ground_adapter.is_using_sbus(), "is_using_sbus must be true");
    TEST_ASSERT(ground_adapter.has_new_frame(), "has_new_frame must be true");

    // Uplink to Air
    uint8_t rf_frame[64];
    size_t rf_len = ground_adapter.get_uplink_frame(rf_frame, 10);
    TEST_ASSERT(rf_len == 39, "RF frame length must be 39 bytes");

    TransportParser air_parser(TransportNodeRole::AIR);
    size_t decoded = air_parser.feed_buffer(rf_frame, rf_len, 2000, nullptr, nullptr);
    TEST_ASSERT(decoded == 1, "Air failed to decode RF frame from SBUS ingest");

    const TransportPackedRc* prc = reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload());
    TEST_ASSERT(air_adapter.ingest_rc_frame(*prc, 2000), "Air failed to ingest RC frame");

    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(air_adapter.get_channels()[i] == handset_ch[i], "Channel value mismatch on Air from SBUS");
    }

    std::cout << "  [PASS] test_sbus_ingestion\n";
    return true;
}

bool test_openi6x_frame_ingestion() {
    RcGroundAdapter ground_adapter;
    ground_adapter.enable_sbus(true);
    RcAirAdapter air_adapter;

    // Simulate 26-byte OpenI6X TX2 frame: 0x0F, 0x64, 22B channels, 0x80 (flags), 0x00 (end byte)
    uint16_t handset_ch[16];
    handset_ch[0] = 772; // OpenI6X wire value (~1360 us) -> corrected to 992 (1500 us)
    for (size_t i = 1; i < 16; ++i) handset_ch[i] = 992; // Centered 1500 us

    uint8_t raw22[22];
    transport_pack_rc_channels(handset_ch, raw22);

    uint8_t openi6x_frame[26];
    openi6x_frame[0] = 0x0F;
    openi6x_frame[1] = 0x64;
    memcpy(&openi6x_frame[2], raw22, 22);
    openi6x_frame[24] = 0x80;
    openi6x_frame[25] = 0x00;

    // Feed some initial junk bytes to verify sliding window synchronization
    ground_adapter.feed_byte(0xAA);
    ground_adapter.feed_byte(0x55);

    // Feed to ground adapter
    for (size_t i = 0; i < 26; ++i) {
        ground_adapter.feed_byte(openi6x_frame[i]);
    }

    TEST_ASSERT(ground_adapter.get_stats().crsf_frames_in == 1, "Ground failed to parse OpenI6X 26B frame");
    TEST_ASSERT(ground_adapter.has_new_frame(), "has_new_frame must be true");

    // Uplink to Air
    uint8_t rf_frame[64];
    size_t rf_len = ground_adapter.get_uplink_frame(rf_frame, 20);
    TEST_ASSERT(rf_len == 39, "RF frame length must be 39 bytes");

    TransportParser air_parser(TransportNodeRole::AIR);
    size_t decoded = air_parser.feed_buffer(rf_frame, rf_len, 3000, nullptr, nullptr);
    TEST_ASSERT(decoded == 1, "Air failed to decode RF frame from OpenI6X");

    const TransportPackedRc* prc = reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload());
    TEST_ASSERT(air_adapter.ingest_rc_frame(*prc, 3000), "Air failed to ingest RC frame");

    uint16_t expected_ch[16];
    for (size_t i = 0; i < 16; ++i) expected_ch[i] = handset_ch[i];
    expected_ch[0] = handset_ch[5]; // Ch1 gets Ch6 (Roll)
    expected_ch[5] = 992;           // Ch6 gets parked at 992 (1500 us)
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(air_adapter.get_channels()[i] == expected_ch[i], "Channel mismatch on Air from OpenI6X frame");
    }

    std::cout << "  [PASS] test_openi6x_frame_ingestion\n";
    return true;
}

int main() {
    std::cout << "=== Running Controlled RC Adapter Unit Tests ===\n";
    if (test_rc_adapter_pipeline() && test_sbus_ingestion() && test_openi6x_frame_ingestion()) {
        std::cout << "=== ALL RC ADAPTER TESTS PASSED ===\n";
        return 0;
    } else {
        std::cerr << "=== RC ADAPTER TESTS FAILED ===\n";
        return 1;
    }
}
