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

    // 6. Test baseline plus 3 advancing frames for restoration from FAILSAFE
    // Restore baseline at t = 1600 ms: still in failsafe
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 2);
    air_parser.feed_buffer(rf_frame, rf_len, 1600, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1600);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe should remain active after 1st restore frame");

    // Restore advancing frame 1 at t = 1620 ms: still in failsafe
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 3);
    air_parser.feed_buffer(rf_frame, rf_len, 1620, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1620);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe should remain active after 2nd restore frame");

    // Restore advancing frame 2 at t = 1640 ms: still in failsafe
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 4);
    air_parser.feed_buffer(rf_frame, rf_len, 1640, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1640);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must remain after only 2 advancing frames");

    // Restore advancing frame 3 at t = 1660 ms: failsafe cleared!
    rf_len = ground_adapter.get_uplink_frame(rf_frame, 5);
    air_parser.feed_buffer(rf_frame, rf_len, 1660, nullptr, nullptr);
    air_adapter.ingest_rc_frame(*reinterpret_cast<const TransportPackedRc*>(air_parser.get_payload()), 1660);
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Failsafe must clear after 3 advancing frames");
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

bool test_crsf_ping_detection() {
    RcGroundAdapter ground_adapter;
    uint8_t ping_frame[4] = {
        CRSF_ADDRESS_TRANSMITTER,
        2,
        CRSF_FRAMETYPE_DEVICE_PING,
        0
    };
    ping_frame[3] = crsf_crc8(&ping_frame[2], 1);

    for (size_t i = 0; i < sizeof(ping_frame); ++i) {
        ground_adapter.feed_byte(ping_frame[i]);
    }

    TEST_ASSERT(ground_adapter.pop_ping_request(), "Valid CRSF DEVICE_PING must be reported");
    TEST_ASSERT(!ground_adapter.pop_ping_request(), "Ping request must be consumed exactly once");
    TEST_ASSERT(!ground_adapter.has_handset_signal(), "A ping alone must not count as an RC handset frame");

    std::cout << "  [PASS] test_crsf_ping_detection\n";
    return true;
}

bool test_failsafe_lifecycle_and_suppression() {
    RcAirAdapter air_adapter;
    uint8_t fc_buf[CRSF_FRAME_RC_TOTAL_SIZE];
    memset(fc_buf, 0, sizeof(fc_buf));

    // 1. Null output buffer returns 0
    TEST_ASSERT(air_adapter.get_fc_frame(nullptr) == 0, "Null buffer must return 0");

    // 2. Boot with no RC returns 0 (NO_SIGNAL state)
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "Boot with no RC must return 0");
    TEST_ASSERT(!air_adapter.has_new_frame(), "Boot state should not have new frame");

    // 3. A long no-signal wait still returns 0
    air_adapter.update(10000);
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "Long wait in NO_SIGNAL must still return 0");

    // 4. Valid initial frame produces correct CRSF output
    uint16_t init_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        init_ch[i] = 1000 + i * 50; // 1000, 1050, 1100, ...
    }
    TransportPackedRc packed1{};
    TEST_ASSERT(transport_pack_rc_channels(init_ch, packed1.channels), "Packing initial channels failed");
    packed1.rc_sequence = 1;
    packed1.flags = 0;

    // Ingest at t = 10000 ms
    TEST_ASSERT(air_adapter.ingest_rc_frame(packed1, 10000), "Initial frame ingest failed");
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Link should be active after initial frame");

    // Null buffer must return 0 while output is otherwise eligible
    TEST_ASSERT(air_adapter.get_fc_frame(nullptr) == 0, "Null buffer must return 0 when output is eligible");

    size_t fc_len = air_adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "Initial frame output length must be 26");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "Header sync byte mismatch (expected 0xC8)");
    TEST_ASSERT(fc_buf[1] == CRSF_FRAME_RC_LENGTH, "Frame length mismatch (expected 24)");
    TEST_ASSERT(fc_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "Frame type mismatch (expected 0x16)");
    // CRC check: CRC-8 over type (byte 2) + payload (bytes 3..24) = 23 bytes
    uint8_t expected_crc = crsf_crc8(&fc_buf[2], 23);
    TEST_ASSERT(fc_buf[25] == expected_crc, "CRSF frame CRC mismatch");

    // Channel fidelity check
    uint16_t unpacked_ch[CRSF_NUM_CHANNELS];
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_ch), "Failed to unpack CRSF channels");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_ch[i] == init_ch[i], "Channel value mismatch in initial frame");
    }

    // 5. DEGRADED output preserves all 16 accepted channels
    // Advance time by 250 ms (t = 10250 ms, dt = 250 ms > 100 ms and <= 500 ms)
    air_adapter.update(10250);
    memset(fc_buf, 0, sizeof(fc_buf));
    fc_len = air_adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "DEGRADED link must still produce 26-byte frame");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "DEGRADED header sync byte mismatch");
    TEST_ASSERT(fc_buf[1] == CRSF_FRAME_RC_LENGTH, "DEGRADED frame length mismatch");
    TEST_ASSERT(fc_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "DEGRADED frame type mismatch");
    TEST_ASSERT(fc_buf[25] == crsf_crc8(&fc_buf[2], 23), "DEGRADED CRSF frame CRC mismatch");
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_ch), "Failed to unpack DEGRADED channels");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_ch[i] == init_ch[i], "Channel mismatch during DEGRADED state");
    }

    // 6. Exactly 500 ms after last accepted frame remains eligible
    // (t = 10500 ms, dt = 500 ms)
    air_adapter.update(10500);
    memset(fc_buf, 0, sizeof(fc_buf));
    fc_len = air_adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "Output must remain eligible at exactly 500 ms timeout");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "500ms boundary header sync byte mismatch");
    TEST_ASSERT(fc_buf[1] == CRSF_FRAME_RC_LENGTH, "500ms boundary frame length mismatch");
    TEST_ASSERT(fc_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "500ms boundary frame type mismatch");
    TEST_ASSERT(fc_buf[25] == crsf_crc8(&fc_buf[2], 23), "500ms boundary CRSF frame CRC mismatch");
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_ch), "Failed to unpack 500ms boundary channels");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_ch[i] == init_ch[i], "Channel mismatch at 500ms boundary");
    }

    // 7. At 501 ms output is suppressed
    // (t = 10501 ms, dt = 501 ms > 500 ms -> FAILSAFE)
    air_adapter.update(10501);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must be active at 501 ms");
    memset(fc_buf, 0xFF, sizeof(fc_buf));
    fc_len = air_adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == 0, "Frame output must be suppressed at 501 ms (FAILSAFE)");

    // 8. Recovery arrivals with a distinct channel vector
    uint16_t recovery_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        recovery_ch[i] = 1800 - i * 40; // 1800, 1760, 1720, ...
    }

    // First recovery arrival establishes the candidate baseline -> suppressed
    TransportPackedRc rec_packed1{};
    TEST_ASSERT(transport_pack_rc_channels(recovery_ch, rec_packed1.channels), "Packing recovery frame 1 channels failed");
    rec_packed1.rc_sequence = 2;
    rec_packed1.flags = 0;
    TEST_ASSERT(air_adapter.ingest_rc_frame(rec_packed1, 10600), "Ingest 1st recovery frame failed");
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Link must remain in failsafe after 1st recovery frame");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "Output must remain suppressed after 1st recovery frame");

    // First advancing recovery arrival (t = 10620 ms, seq = 3) -> suppressed
    TransportPackedRc rec_packed2{};
    TEST_ASSERT(transport_pack_rc_channels(recovery_ch, rec_packed2.channels), "Packing recovery frame 2 channels failed");
    rec_packed2.rc_sequence = 3;
    rec_packed2.flags = 0;
    TEST_ASSERT(air_adapter.ingest_rc_frame(rec_packed2, 10620), "Ingest 2nd recovery frame failed");
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Link must remain in failsafe after first advancing recovery frame");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "Output must remain suppressed after first advancing recovery frame");

    // Second advancing recovery arrival (t = 10640 ms, seq = 4) -> still suppressed
    TransportPackedRc rec_packed3{};
    TEST_ASSERT(transport_pack_rc_channels(recovery_ch, rec_packed3.channels), "Packing recovery frame 3 channels failed");
    rec_packed3.rc_sequence = 4;
    rec_packed3.flags = 0;
    TEST_ASSERT(air_adapter.ingest_rc_frame(rec_packed3, 10640), "Ingest 3rd recovery frame failed");
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Link must remain in failsafe after second advancing recovery frame");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "Output must remain suppressed after second advancing recovery frame");

    // Third advancing recovery arrival (t = 10660 ms, seq = 5) -> output resumes
    TransportPackedRc rec_packed4{};
    TEST_ASSERT(transport_pack_rc_channels(recovery_ch, rec_packed4.channels), "Packing recovery frame 4 channels failed");
    rec_packed4.rc_sequence = 5;
    rec_packed4.flags = 0;
    TEST_ASSERT(air_adapter.ingest_rc_frame(rec_packed4, 10660), "Ingest 4th recovery frame failed");
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Link must recover after 3 advancing frames");

    memset(fc_buf, 0, sizeof(fc_buf));
    fc_len = air_adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "Output must resume (26 bytes) after 3rd advancing recovery frame");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "Recovered header sync byte mismatch");
    TEST_ASSERT(fc_buf[1] == CRSF_FRAME_RC_LENGTH, "Recovered frame length mismatch");
    TEST_ASSERT(fc_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "Recovered frame type mismatch");
    TEST_ASSERT(fc_buf[25] == crsf_crc8(&fc_buf[2], 23), "Recovered CRSF frame CRC mismatch");
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_ch), "Failed to unpack recovered channels");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_ch[i] == recovery_ch[i], "Channel mismatch with recovery channels");
    }

    std::cout << "  [PASS] test_failsafe_lifecycle_and_suppression\n";
    return true;
}

bool test_rc_sequence_active_stream_modular_rules() {
    RcAirAdapter adapter;

    // 1. Initial acquisition at timestamp 0
    TEST_ASSERT(!adapter.has_committed_seq(), "Starts without committed seq");
    TEST_ASSERT(!adapter.has_valid_time(), "Starts without valid time");
    uint16_t ch0[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch0[i] = 1000 + i * 20;
    TransportPackedRc p0{};
    TEST_ASSERT(transport_pack_rc_channels(ch0, p0.channels), "pack ch0 failed");
    p0.rc_sequence = 10;
    p0.flags = 0;

    TEST_ASSERT(adapter.ingest_rc_frame(p0, 0), "Initial acquisition at t=0 failed");
    TEST_ASSERT(adapter.has_committed_seq(), "Committed seq must be set at t=0");
    TEST_ASSERT(adapter.get_committed_seq() == 10, "Committed seq must be 10");
    TEST_ASSERT(adapter.has_valid_time(), "has_valid_time must be true at t=0");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 0, "last_valid_time_ms must be 0");
    TEST_ASSERT(!adapter.is_failsafe_active(), "Link must be ACTIVE at t=0");
    uint8_t fc_buf[CRSF_FRAME_RC_TOTAL_SIZE];
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == CRSF_FRAME_RC_TOTAL_SIZE, "Output must be eligible at t=0");
    uint16_t unpacked_ch[CRSF_NUM_CHANNELS];
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_ch), "unpack failed");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_ch[i] == ch0[i], "Channel mismatch at t=0");
    }

    // Timestamp-zero validity: dt = 50ms still active, dt = 501ms failsafe
    adapter.update(50);
    TEST_ASSERT(!adapter.is_failsafe_active(), "Still active at t=50 from t=0");

    // 2. Initial acquisition at nonzero timestamp
    RcAirAdapter adapter_nz;
    uint16_t ch_nz[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_nz[i] = 1100 + i * 10;
    TransportPackedRc p_nz{};
    TEST_ASSERT(transport_pack_rc_channels(ch_nz, p_nz.channels), "pack ch_nz failed");
    p_nz.rc_sequence = 42;
    p_nz.flags = 0;
    TEST_ASSERT(adapter_nz.ingest_rc_frame(p_nz, 5000), "Initial acquisition at t=5000 failed");
    TEST_ASSERT(adapter_nz.has_committed_seq(), "Committed seq must be set at t=5000");
    TEST_ASSERT(adapter_nz.get_committed_seq() == 42, "Committed seq mismatch at t=5000");
    TEST_ASSERT(adapter_nz.get_last_valid_time_ms() == 5000, "Valid time mismatch at t=5000");
    TEST_ASSERT(!adapter_nz.is_failsafe_active(), "Link must be active at t=5000");

    // 3. Active delta 1 accepted
    // Continue on adapter (current seq = 10 at t = 0)
    uint16_t ch_11[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_11[i] = 1200 + i * 15;
    TransportPackedRc p_11{};
    TEST_ASSERT(transport_pack_rc_channels(ch_11, p_11.channels), "pack p_11 failed");
    p_11.rc_sequence = 11; // delta = (11 - 10) = 1
    p_11.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_11, 70), "Active delta 1 must be accepted");
    TEST_ASSERT(adapter.get_committed_seq() == 11, "Committed seq must advance to 11");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 70, "Timestamp must refresh to 70");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_11, sizeof(ch_11)) == 0, "Channels must update to ch_11");

    // 4. Active duplicate delta 0 rejected without timestamp/channel change
    uint16_t bad_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) bad_ch[i] = 1999;
    TransportPackedRc p_dup{};
    TEST_ASSERT(transport_pack_rc_channels(bad_ch, p_dup.channels), "pack p_dup failed");
    p_dup.rc_sequence = 11; // delta = (11 - 11) = 0
    p_dup.flags = 0;
    TEST_ASSERT(!adapter.ingest_rc_frame(p_dup, 100), "Active duplicate delta 0 must be rejected");
    TEST_ASSERT(adapter.get_committed_seq() == 11, "Committed seq must remain 11");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 70, "Timestamp must NOT refresh (remains 70)");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_11, sizeof(ch_11)) == 0, "Channels must NOT be modified by duplicate");

    // 5. Active delta 127 accepted
    uint16_t ch_138[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_138[i] = 1300 + i * 10;
    TransportPackedRc p_127{};
    TEST_ASSERT(transport_pack_rc_channels(ch_138, p_127.channels), "pack p_127 failed");
    p_127.rc_sequence = (11 + 127) & 0xFF; // 138, delta = 127
    p_127.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_127, 140), "Active delta 127 must be accepted");
    TEST_ASSERT(adapter.get_committed_seq() == 138, "Committed seq must advance to 138");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 140, "Timestamp must refresh to 140");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_138, sizeof(ch_138)) == 0, "Channels must update to ch_138");

    // 6. Active delta 128 rejected
    TransportPackedRc p_128{};
    TEST_ASSERT(transport_pack_rc_channels(bad_ch, p_128.channels), "pack p_128 failed");
    p_128.rc_sequence = (138 + 128) & 0xFF; // 10, delta = 128
    p_128.flags = 0;
    TEST_ASSERT(!adapter.ingest_rc_frame(p_128, 200), "Active delta 128 must be rejected");
    TEST_ASSERT(adapter.get_committed_seq() == 138, "Committed seq must remain 138");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 140, "Timestamp must NOT refresh");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_138, sizeof(ch_138)) == 0, "Channels must NOT change on delta 128");

    // 7. Active delta 255 rejected
    TransportPackedRc p_255{};
    TEST_ASSERT(transport_pack_rc_channels(bad_ch, p_255.channels), "pack p_255 failed");
    p_255.rc_sequence = (138 - 1) & 0xFF; // 137, delta = (uint8_t)(137 - 138) = 255
    p_255.flags = 0;
    TEST_ASSERT(!adapter.ingest_rc_frame(p_255, 210), "Active delta 255 must be rejected");
    TEST_ASSERT(adapter.get_committed_seq() == 138, "Committed seq must remain 138");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 140, "Timestamp must NOT refresh");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_138, sizeof(ch_138)) == 0, "Channels must NOT change on delta 255");

    // 8. 255 -> 0 rollover accepted
    // First advance to 255: delta = (255 - 138) = 117 in [1, 127]
    uint16_t ch_255[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_255[i] = 1400 + i * 5;
    TransportPackedRc p_to_255{};
    TEST_ASSERT(transport_pack_rc_channels(ch_255, p_to_255.channels), "pack p_to_255 failed");
    p_to_255.rc_sequence = 255;
    p_to_255.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_to_255, 280), "Advance to 255 must be accepted");
    TEST_ASSERT(adapter.get_committed_seq() == 255, "Committed seq must be 255");

    // Now roll over from 255 to 0: delta = (uint8_t)(0 - 255) = 1 in [1, 127]
    uint16_t ch_roll[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_roll[i] = 1500 + i * 5;
    TransportPackedRc p_roll{};
    TEST_ASSERT(transport_pack_rc_channels(ch_roll, p_roll.channels), "pack p_roll failed");
    p_roll.rc_sequence = 0;
    p_roll.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_roll, 350), "255->0 rollover must be accepted");
    TEST_ASSERT(adapter.get_committed_seq() == 0, "Committed seq must roll over to 0");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 350, "Timestamp must refresh to 350");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_roll, sizeof(ch_roll)) == 0, "Channels must update to ch_roll");

    // 9. Gap acceptance
    // Jump from 0 to 5: delta = 5 in [1, 127]
    uint16_t ch_gap[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) ch_gap[i] = 1600 + i * 5;
    TransportPackedRc p_gap{};
    TEST_ASSERT(transport_pack_rc_channels(ch_gap, p_gap.channels), "pack p_gap failed");
    p_gap.rc_sequence = 5;
    p_gap.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_gap, 420), "Gap of 5 must be accepted");
    TEST_ASSERT(adapter.get_committed_seq() == 5, "Committed seq must advance to 5");
    TEST_ASSERT(adapter.get_last_valid_time_ms() == 420, "Timestamp must refresh to 420");
    TEST_ASSERT(memcmp(adapter.get_channels(), ch_gap, sizeof(ch_gap)) == 0, "Channels must update to ch_gap");

    std::cout << "  [PASS] test_rc_sequence_active_stream_modular_rules\n";
    return true;
}

bool test_rc_failsafe_reacquisition_and_candidate_rules() {
    RcAirAdapter adapter;
    uint8_t fc_buf[CRSF_FRAME_RC_TOTAL_SIZE];

    // Establish active stream with committed channels at t = 1000 ms
    uint16_t active_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) active_ch[i] = 1000 + i * 30;
    TransportPackedRc p_init{};
    TEST_ASSERT(transport_pack_rc_channels(active_ch, p_init.channels), "pack init failed");
    p_init.rc_sequence = 10;
    p_init.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_init, 1000), "Initial ingest failed");
    TEST_ASSERT(!adapter.is_failsafe_active(), "Link must be ACTIVE");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 26, "Output must be 26 bytes");

    // 1. Arrival after timeout without prior update()
    // Advance time by 600 ms (t = 1600 ms, elapsed = 600 > 500 ms) WITHOUT calling adapter.update()
    uint16_t cand1_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) cand1_ch[i] = 1100 + i * 30;
    TransportPackedRc p_c1{};
    TEST_ASSERT(transport_pack_rc_channels(cand1_ch, p_c1.channels), "pack p_c1 failed");
    p_c1.rc_sequence = 11;
    p_c1.flags = 0;

    // Ingest at t = 1600: Pre-evaluation MUST trigger failsafe and treat frame as candidate 1
    TEST_ASSERT(adapter.ingest_rc_frame(p_c1, 1600), "Candidate frame 1 accepted in failsafe");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link MUST enter FAILSAFE due to elapsed > 500 ms");
    TEST_ASSERT(!adapter.has_committed_seq(), "Committed sequence must be invalidated");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Candidate frame 1 output MUST be suppressed (0 bytes)");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Restore count must be 0 for the candidate baseline");
    TEST_ASSERT(adapter.get_candidate_seq() == 11, "Candidate seq must be 11");

    // Committed channels must remain UNCHANGED during candidate frame 1!
    TEST_ASSERT(memcmp(adapter.get_channels(), active_ch, sizeof(active_ch)) == 0,
                "Committed channels must remain unchanged during candidate frame 1");
    // Candidate channels are separate
    TEST_ASSERT(memcmp(adapter.get_candidate_channels(), cand1_ch, sizeof(cand1_ch)) == 0,
                "Candidate channels must hold candidate 1 values in staging buffer");

    // 2. Candidate delta 1 accepted but suppressed (Candidate Frame 2)
    uint16_t cand2_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) cand2_ch[i] = 1200 + i * 30;
    TransportPackedRc p_c2{};
    TEST_ASSERT(transport_pack_rc_channels(cand2_ch, p_c2.channels), "pack p_c2 failed");
    p_c2.rc_sequence = 12; // delta = (12 - 11) = 1 in [1, 127]
    p_c2.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_c2, 1670), "Candidate delta 1 must be accepted");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link must remain in failsafe after candidate 2");
    TEST_ASSERT(adapter.get_restore_count() == 1, "Restore count must advance to 1 after the first advancing candidate");
    TEST_ASSERT(adapter.get_candidate_seq() == 12, "Candidate seq must be 12");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Candidate frame 2 output MUST be suppressed (0 bytes)");
    TEST_ASSERT(memcmp(adapter.get_channels(), active_ch, sizeof(active_ch)) == 0,
                "Committed channels must still remain unchanged after candidate frame 2");

    // 3. Candidate duplicate resets candidate
    uint16_t bad_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) bad_ch[i] = 1999;
    TransportPackedRc p_c_dup{};
    TEST_ASSERT(transport_pack_rc_channels(bad_ch, p_c_dup.channels), "pack p_c_dup failed");
    p_c_dup.rc_sequence = 12; // delta = (12 - 12) = 0
    p_c_dup.flags = 0;
    TEST_ASSERT(!adapter.ingest_rc_frame(p_c_dup, 1700), "Candidate duplicate must be rejected");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Candidate duplicate MUST reset restore count to 0");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link must remain in failsafe");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Output remains suppressed");
    TEST_ASSERT(memcmp(adapter.get_channels(), active_ch, sizeof(active_ch)) == 0,
                "Committed channels must still remain unchanged");

    // 4. Candidate delta 127 accepted but suppressed
    // Start fresh candidate 1 at t = 1750 ms, seq = 20
    TransportPackedRc p_c_base{};
    TEST_ASSERT(transport_pack_rc_channels(cand1_ch, p_c_base.channels), "pack p_c_base failed");
    p_c_base.rc_sequence = 20;
    p_c_base.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_c_base, 1750), "Fresh candidate 1 accepted");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Restore count is 0 for the candidate baseline");

    // Advance with delta 127: seq = (20 + 127) = 147
    TransportPackedRc p_c_127{};
    TEST_ASSERT(transport_pack_rc_channels(cand2_ch, p_c_127.channels), "pack p_c_127 failed");
    p_c_127.rc_sequence = 147;
    p_c_127.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_c_127, 1800), "Candidate delta 127 accepted");
    TEST_ASSERT(adapter.get_restore_count() == 1, "Restore count is 1 after delta 127");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link remains in failsafe");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Output remains suppressed");

    // 5. Candidate delta 128 resets candidate
    // Current candidate seq is 147. delta 128: seq = (147 + 128) & 0xFF = 19
    TransportPackedRc p_c_128{};
    TEST_ASSERT(transport_pack_rc_channels(bad_ch, p_c_128.channels), "pack p_c_128 failed");
    p_c_128.rc_sequence = (147 + 128) & 0xFF;
    p_c_128.flags = 0;
    TEST_ASSERT(!adapter.ingest_rc_frame(p_c_128, 1850), "Candidate delta 128 must be rejected");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Candidate delta 128 MUST reset restore count to 0");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link remains in failsafe");

    // 6. Candidate timeout causes new candidate baseline
    // Candidate 1 at t = 1900 ms, seq = 60
    TransportPackedRc p_c_to{};
    TEST_ASSERT(transport_pack_rc_channels(cand1_ch, p_c_to.channels), "pack p_c_to failed");
    p_c_to.rc_sequence = 60;
    p_c_to.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_c_to, 1900), "Candidate 1 accepted at t=1900");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Restore count is 0 for the candidate baseline");

    // Wait 501 ms (t = 2401 ms, elapsed > 500 ms). Candidate expires!
    // Arrival at t = 2401 with seq = 61: treated as NEW candidate baseline
    TransportPackedRc p_c_new_base{};
    TEST_ASSERT(transport_pack_rc_channels(cand2_ch, p_c_new_base.channels), "pack p_c_new_base failed");
    p_c_new_base.rc_sequence = 61;
    p_c_new_base.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_c_new_base, 2401), "Arrival after candidate timeout accepted");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Must establish new candidate baseline (restore count = 0)");
    TEST_ASSERT(adapter.get_candidate_seq() == 61, "Candidate seq must be 61");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Output still suppressed");

    // 7. Three accepted advancing candidate frames commit atomically & output resumes
    // First advancing frame at t = 2470 ms: seq = 62, delta = 1
    TransportPackedRc p_rec2{};
    TEST_ASSERT(transport_pack_rc_channels(cand2_ch, p_rec2.channels), "pack p_rec2 failed");
    p_rec2.rc_sequence = 62;
    p_rec2.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_rec2, 2470), "Restore candidate 2 accepted");
    TEST_ASSERT(adapter.get_restore_count() == 1, "Restore count is 1 after first advancing frame");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Output still suppressed after first advancing frame");
    TEST_ASSERT(memcmp(adapter.get_channels(), active_ch, sizeof(active_ch)) == 0,
                "Committed channels still unchanged before frame 3 commit");

    // Second advancing frame at t = 2540 ms: seq = 63, delta = 1 -> still suppressed
    TransportPackedRc p_rec3{};
    TEST_ASSERT(transport_pack_rc_channels(cand2_ch, p_rec3.channels), "pack p_rec3 failed");
    p_rec3.rc_sequence = 63;
    p_rec3.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_rec3, 2540), "Restore candidate 3 accepted");
    TEST_ASSERT(adapter.is_failsafe_active(), "Link must remain in failsafe after two advancing frames");
    TEST_ASSERT(adapter.get_restore_count() == 2, "Restore count is 2 after second advancing frame");
    TEST_ASSERT(adapter.get_fc_frame(fc_buf) == 0, "Output still suppressed after second advancing frame");
    TEST_ASSERT(memcmp(adapter.get_channels(), active_ch, sizeof(active_ch)) == 0,
                "Committed channels still unchanged before final commit");

    // Third advancing frame at t = 2610 ms: seq = 64, delta = 1 -> ATOMIC COMMIT!
    uint16_t restored_ch[CRSF_NUM_CHANNELS];
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) restored_ch[i] = 1800 - i * 15;
    TransportPackedRc p_rec4{};
    TEST_ASSERT(transport_pack_rc_channels(restored_ch, p_rec4.channels), "pack p_rec4 failed");
    p_rec4.rc_sequence = 64;
    p_rec4.flags = 0;
    TEST_ASSERT(adapter.ingest_rc_frame(p_rec4, 2610), "Restore candidate 4 accepted");
    TEST_ASSERT(!adapter.is_failsafe_active(), "Link MUST clear failsafe on the third advancing frame!");
    TEST_ASSERT(adapter.has_committed_seq(), "Committed seq must be true");
    TEST_ASSERT(adapter.get_committed_seq() == 64, "Committed seq must be 64");
    TEST_ASSERT(adapter.get_restore_count() == 0, "Restore count reset to 0 after commit");

    // Output must now resume and deliver restored channels
    size_t fc_len = adapter.get_fc_frame(fc_buf);
    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "Output must resume (26 bytes)");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "Header sync byte mismatch");
    TEST_ASSERT(fc_buf[1] == CRSF_FRAME_RC_LENGTH, "Frame length mismatch");
    TEST_ASSERT(fc_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "Frame type mismatch");
    TEST_ASSERT(fc_buf[25] == crsf_crc8(&fc_buf[2], 23), "CRSF frame CRC mismatch");
    uint16_t unpacked_res[CRSF_NUM_CHANNELS];
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], unpacked_res), "Failed to unpack restored channels");
    for (size_t i = 0; i < CRSF_NUM_CHANNELS; ++i) {
        TEST_ASSERT(unpacked_res[i] == restored_ch[i], "Channel value mismatch with committed channels");
    }
    TEST_ASSERT(memcmp(adapter.get_channels(), restored_ch, sizeof(restored_ch)) == 0,
                "Committed channels must atomically match restored_ch");

    std::cout << "  [PASS] test_rc_failsafe_reacquisition_and_candidate_rules\n";
    return true;
}

int main() {
    std::cout << "=== Running Controlled RC Adapter Unit Tests ===\n";
    if (test_rc_adapter_pipeline() &&
        test_sbus_ingestion() &&
        test_openi6x_frame_ingestion() &&
        test_crsf_ping_detection() &&
        test_failsafe_lifecycle_and_suppression() &&
        test_rc_sequence_active_stream_modular_rules() &&
        test_rc_failsafe_reacquisition_and_candidate_rules()) {
        std::cout << "=== ALL RC ADAPTER TESTS PASSED ===\n";
        return 0;
    } else {
        std::cerr << "=== RC ADAPTER TESTS FAILED ===\n";
        return 1;
    }
}
