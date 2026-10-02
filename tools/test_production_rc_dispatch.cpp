// =============================================================================
// Dual-LRS Step 3.1.2: Production RC End-to-End Dispatch Host Test Suite
// Verifies:
// 1. Handset Ingest (CRSF & SBUS) -> Ground Adapter packing
// 2. Handset Disconnect / Timeout (500ms) -> Ground fallback to LINK_CONTROL beacon
// 3. Ground TDM Slot 1 RF Transmission (39B RC_CONTROL frame)
// 4. Over-The-Air RF Transfer to Air Unit via TdmEngine
// 5. Air Unit TdmEngine Frame Validation (Magic, Channel, CRC16) & Demux
// 6. Air RcAirAdapter Ingestion, Mailbox Update, & 16-Channel Fidelity
// 7. Paced Air FC CRSF Output Generation (26B frame @ 420k baud format)
// 8. Air Failsafe Watchdog & "No Pulses" FC Output Suppression on Link Loss (>500ms)
// 9. 3-Frame Link Restoration from Failsafe
// 10. Corrupted RF Frame Rejection & Link Resilience
// =============================================================================

#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>

#include "Arduino.h"
#include "e22_driver.h"
#include "tdm_engine.h"
#include "transport_protocol.h"
#include "transport_engine.h"
#include "crsf_protocol.h"
#include "rc_adapter.h"

// ---------------------------------------------------------------------------
// Simulated Time Management
// ---------------------------------------------------------------------------
uint32_t g_mock_micros = 0;
uint32_t g_mock_millis = 0;

void set_mock_time_ms(uint32_t ms) {
    g_mock_millis = ms;
    g_mock_micros = ms * 1000;
}

void advance_mock_time_ms(uint32_t delta_ms) {
    g_mock_millis += delta_ms;
    g_mock_micros = g_mock_millis * 1000;
}

// ---------------------------------------------------------------------------
// Dual-Node Simulated E22 RF Bus
// ---------------------------------------------------------------------------
static std::vector<uint8_t> s_rf_ground_to_air;
static std::vector<uint8_t> s_rf_air_to_ground;
static bool s_rf_drop_air_rx = false;

HardwareSerial dummy_serial_ground;
HardwareSerial dummy_serial_air;

E22Driver::E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux)
    : _serial(serialPort), _pinM0(pinM0), _pinM1(pinM1), _pinAux(pinAux), _currentMode(E22Mode::NORMAL) {}

void E22Driver::begin(uint32_t) {}
bool E22Driver::beginPassive(uint32_t) { return true; }
void E22Driver::setMode(E22Mode) {}
bool E22Driver::configureRadio(uint8_t, uint8_t) { return true; }
E22RegReadStatus E22Driver::readRegistersReadOnly(uint8_t*, size_t, size_t&) { return E22RegReadStatus::OK; }
bool E22Driver::isBusy() const { return false; }
bool E22Driver::waitForReady(uint32_t) { return true; }

size_t E22Driver::write(const uint8_t* data, size_t length) {
    if (&_serial == &dummy_serial_ground) {
        if (!s_rf_drop_air_rx) {
            s_rf_ground_to_air.insert(s_rf_ground_to_air.end(), data, data + length);
        }
    } else {
        s_rf_air_to_ground.insert(s_rf_air_to_ground.end(), data, data + length);
    }
    return length;
}

int E22Driver::available() {
    if (&_serial == &dummy_serial_air) {
        return (int)s_rf_ground_to_air.size();
    } else {
        return (int)s_rf_air_to_ground.size();
    }
}

int E22Driver::read() {
    std::vector<uint8_t>& q = (&_serial == &dummy_serial_air) ? s_rf_ground_to_air : s_rf_air_to_ground;
    if (q.empty()) return -1;
    uint8_t b = q.front();
    q.erase(q.begin());
    return b;
}

size_t E22Driver::readBytes(uint8_t* buffer, size_t length) {
    std::vector<uint8_t>& q = (&_serial == &dummy_serial_air) ? s_rf_ground_to_air : s_rf_air_to_ground;
    size_t count = 0;
    while (count < length && !q.empty()) {
        buffer[count++] = q.front();
        q.erase(q.begin());
    }
    return count;
}

void E22Driver::flush() {}

void reset_rf_bus() {
    s_rf_ground_to_air.clear();
    s_rf_air_to_ground.clear();
    s_rf_drop_air_rx = false;
}

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

// Context for Air callbacks
static struct {
    uint32_t rc_frames_received;
    uint32_t link_frames_received;
    uint8_t last_channel;
    TransportPackedRc last_packed_rc;
} s_air_rx_stats;

void test_air_transport_callback(const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
    s_air_rx_stats.last_channel = hdr.channel;
    if (hdr.channel == (uint8_t)TransportChannel::RC_CONTROL) {
        s_air_rx_stats.rc_frames_received++;
        if (length >= sizeof(TransportPackedRc) && payload != nullptr) {
            memcpy(&s_air_rx_stats.last_packed_rc, payload, sizeof(TransportPackedRc));
        }
    } else if (hdr.channel == (uint8_t)TransportChannel::LINK_CONTROL) {
        s_air_rx_stats.link_frames_received++;
    }
}

// =============================================================================
// TEST 1: END-TO-END RC DISPATCH & CHANNEL FIDELITY
// Ingest CRSF handset frame on Ground -> Ground slot transmit -> Air TDM parse ->
// Air RcAirAdapter decode -> 26B CRSF frame generation to FC -> Channel unpack
// =============================================================================
bool test_end_to_end_rc_dispatch_fidelity() {
    reset_rf_bus();
    memset(&s_air_rx_stats, 0, sizeof(s_air_rx_stats));
    set_mock_time_ms(100);

    E22Driver ground_radio(dummy_serial_ground, 0, 0, 0);
    E22Driver air_radio(dummy_serial_air, 0, 0, 0);

    TdmEngine ground_tdm(ground_radio, NodeRole::GROUND);
    TdmEngine air_tdm(air_radio, NodeRole::AIR);

    air_tdm.onTransportFrameReceived(test_air_transport_callback);

    ground_tdm.begin();
    air_tdm.begin();

    RcGroundAdapter ground_adapter;
    RcAirAdapter air_adapter;

    // 1. Simulate FS-i6X Handset emitting 16 channels
    uint16_t handset_channels[16];
    for (size_t i = 0; i < 16; ++i) {
        handset_channels[i] = 1000 + (uint16_t)(i * 60); // 1000 to 1900 us range
    }

    uint8_t handset_crsf_wire[CRSF_FRAME_RC_TOTAL_SIZE];
    size_t crsf_wire_len = crsf_encode_rc_frame(CRSF_ADDRESS_TRANSMITTER, handset_channels, handset_crsf_wire);
    TEST_ASSERT(crsf_wire_len == CRSF_FRAME_RC_TOTAL_SIZE, "CRSF encode length must be 26B");

    // Feed bytes into Ground adapter at t = 100ms
    for (size_t i = 0; i < crsf_wire_len; ++i) {
        ground_adapter.feed_byte(handset_crsf_wire[i], g_mock_millis);
    }
    TEST_ASSERT(ground_adapter.get_stats().crsf_frames_in == 1, "Ground adapter crsf_frames_in must be 1");
    TEST_ASSERT(ground_adapter.has_handset_signal(g_mock_millis), "Ground adapter must report active handset");

    // 2. Ground slot 1 transmission: pack and send 39-byte RF frame
    TransportPackedRc packed_rc{};
    bool has_rc = ground_adapter.get_packed_rc(&packed_rc, g_mock_millis);
    TEST_ASSERT(has_rc, "get_packed_rc must return true when handset is active");
    TEST_ASSERT(packed_rc.rc_sequence == 0, "Initial rc_sequence must be 0");

    bool sent = ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                              TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                              0, 0, (const uint8_t*)&packed_rc, sizeof(TransportPackedRc));
    TEST_ASSERT(sent, "sendTransportFrame(RC_CONTROL) must succeed");
    TEST_ASSERT(s_rf_ground_to_air.size() == 39, "Wire RF frame must be exactly 39 bytes (13B hdr + 24B payload + 2B crc)");

    // 3. Air unit receives RF frame in Slot 1
    // Air updates TDM engine
    air_tdm.update();

    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "Air callback must receive exactly 1 RC frame");
    TEST_ASSERT(s_air_rx_stats.last_channel == (uint8_t)TransportChannel::RC_CONTROL, "Channel must be RC_CONTROL");

    // Ingest into Air RC adapter
    bool ingested = air_adapter.ingest_rc_frame(s_air_rx_stats.last_packed_rc, g_mock_millis);
    TEST_ASSERT(ingested, "Air adapter ingest_rc_frame must return true");
    TEST_ASSERT(air_adapter.has_new_frame(), "Air adapter has_new_frame must be true after ingest");
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Link must be ACTIVE (not in failsafe)");

    // 4. Verify channel fidelity on Air adapter
    const uint16_t* air_channels = air_adapter.get_channels();
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(air_channels[i] == handset_channels[i], "Channel value mismatch across RF transport");
    }

    // 5. Air generates 26-byte CRSF frame for Flight Controller RC_IN
    uint8_t fc_frame[CRSF_FRAME_RC_TOTAL_SIZE];
    size_t fc_len = air_adapter.get_fc_frame(fc_frame);
    air_adapter.clear_new_frame();

    TEST_ASSERT(fc_len == CRSF_FRAME_RC_TOTAL_SIZE, "FC CRSF frame length must be 26 bytes");
    TEST_ASSERT(fc_frame[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "FC frame address must be CRSF_ADDRESS_FLIGHT_CONTROLLER (0xC8)");
    TEST_ASSERT(fc_frame[1] == 24, "FC frame payload length field must be 24");
    TEST_ASSERT(fc_frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED, "FC frame type must be 0x16");
    TEST_ASSERT(!air_adapter.has_new_frame(), "has_new_frame must be false after clear_new_frame()");

    // 6. Decode FC CRSF frame to confirm Flight Controller receives exact channel set
    uint16_t fc_decoded_channels[16];
    bool fc_unpacked = crsf_unpack_channels22(&fc_frame[3], fc_decoded_channels);
    TEST_ASSERT(fc_unpacked, "crsf_unpack_channels22 on FC frame must succeed");

    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(fc_decoded_channels[i] == handset_channels[i], "FC decoded channel value mismatch");
    }

    std::cout << "  [PASS] test_end_to_end_rc_dispatch_fidelity\n";
    return true;
}

// =============================================================================
// TEST 2: HANDSET TIMEOUT & LINK_CONTROL BEACON FALLBACK
// Ground switches from RC_CONTROL (39B) to LINK_CONTROL (15B beacon) when handset is disconnected
// Air maintains TDM PLL sync but Air RC watchdog triggers failsafe ("No Pulses")
// =============================================================================
bool test_handset_timeout_and_beacon_fallback() {
    reset_rf_bus();
    memset(&s_air_rx_stats, 0, sizeof(s_air_rx_stats));
    set_mock_time_ms(1000);

    E22Driver ground_radio(dummy_serial_ground, 0, 0, 0);
    E22Driver air_radio(dummy_serial_air, 0, 0, 0);

    TdmEngine ground_tdm(ground_radio, NodeRole::GROUND);
    TdmEngine air_tdm(air_radio, NodeRole::AIR);
    air_tdm.onTransportFrameReceived(test_air_transport_callback);

    ground_tdm.begin();
    air_tdm.begin();

    RcGroundAdapter ground_adapter;
    RcAirAdapter air_adapter;

    // 1. Initial state: No handset ever connected
    TEST_ASSERT(!ground_adapter.has_handset_signal(g_mock_millis), "Initial handset signal must be false");
    TransportPackedRc prc{};
    TEST_ASSERT(!ground_adapter.get_packed_rc(&prc, g_mock_millis), "get_packed_rc must return false without handset");

    // Ground sends LINK_CONTROL beacon instead
    ground_tdm.sendTransportFrame(TransportChannel::LINK_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, nullptr, 0);
    TEST_ASSERT(s_rf_ground_to_air.size() == 15, "LINK_CONTROL beacon must be 15 bytes (13B hdr + 0B payload + 2B crc)");

    air_tdm.update();
    TEST_ASSERT(s_air_rx_stats.link_frames_received == 1, "Air must receive LINK_CONTROL beacon");
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 0, "Air must receive 0 RC frames");

    // 2. Connect handset: feed 1 frame at t = 1050 ms
    advance_mock_time_ms(50);
    uint16_t ch[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    uint8_t crsf_in[26];
    crsf_encode_rc_frame(CRSF_ADDRESS_TRANSMITTER, ch, crsf_in);
    for (size_t i = 0; i < 26; ++i) ground_adapter.feed_byte(crsf_in[i], g_mock_millis);

    TEST_ASSERT(ground_adapter.has_handset_signal(g_mock_millis), "Handset signal active");
    TEST_ASSERT(ground_adapter.get_packed_rc(&prc, g_mock_millis), "get_packed_rc must return true");

    s_rf_ground_to_air.clear();
    ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, (const uint8_t*)&prc, sizeof(TransportPackedRc));
    TEST_ASSERT(s_rf_ground_to_air.size() == 39, "RC frame must be 39 bytes");

    air_tdm.update();
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "Air must receive RC frame");
    air_adapter.ingest_rc_frame(s_air_rx_stats.last_packed_rc, g_mock_millis);
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Air link active");

    // 3. Handset disconnects: 600 ms pass without new handset frames (t = 1650 ms)
    advance_mock_time_ms(600);
    TEST_ASSERT(!ground_adapter.has_handset_signal(g_mock_millis), "Handset signal must time out after 500ms");
    TEST_ASSERT(!ground_adapter.get_packed_rc(&prc, g_mock_millis), "get_packed_rc must return false on timeout");

    // Ground falls back to LINK_CONTROL beacon
    s_rf_ground_to_air.clear();
    ground_tdm.sendTransportFrame(TransportChannel::LINK_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, nullptr, 0);
    TEST_ASSERT(s_rf_ground_to_air.size() == 15, "Ground must send 15B LINK_CONTROL beacon");

    air_tdm.update();
    TEST_ASSERT(s_air_rx_stats.link_frames_received == 2, "Air receives beacon, PLL sync preserved");
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "No new RC frames received");

    // 4. Air RC watchdog ticks at t = 1650 ms (dt = 600 ms > 500 ms failsafe timeout)
    air_adapter.update(g_mock_millis);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Air RC adapter must enter FAILSAFE state");

    // 5. "No Pulses" policy: get_fc_frame MUST return 0 bytes
    uint8_t fc_out[26];
    size_t fc_bytes = air_adapter.get_fc_frame(fc_out);
    TEST_ASSERT(fc_bytes == 0, "Air FC output MUST be 0 during failsafe (No Pulses policy)");

    std::cout << "  [PASS] test_handset_timeout_and_beacon_fallback\n";
    return true;
}

// =============================================================================
// TEST 3: AIR FAILSAFE WATCHDOG & 3-FRAME RESTORATION
// Verifies:
// 1. Loss of RF uplink triggers failsafe at 500ms.
// 2. While in failsafe, CRSF frames to FC are completely suppressed (0 bytes).
// 3. Frame 1 & Frame 2 do NOT clear failsafe.
// 4. Frame 3 clears failsafe and restores 26B CRSF output to FC.
// =============================================================================
bool test_failsafe_watchdog_and_three_frame_restore() {
    reset_rf_bus();
    memset(&s_air_rx_stats, 0, sizeof(s_air_rx_stats));
    set_mock_time_ms(2000);

    E22Driver ground_radio(dummy_serial_ground, 0, 0, 0);
    E22Driver air_radio(dummy_serial_air, 0, 0, 0);

    TdmEngine ground_tdm(ground_radio, NodeRole::GROUND);
    TdmEngine air_tdm(air_radio, NodeRole::AIR);
    air_tdm.onTransportFrameReceived(test_air_transport_callback);

    ground_tdm.begin();
    air_tdm.begin();

    RcAirAdapter air_adapter;

    // 1. Establish active link with frame at t = 2000ms
    TransportPackedRc prc{};
    for (size_t i = 0; i < 16; ++i) prc.channels[i] = 0; // dummy raw
    prc.rc_sequence = 1;
    transport_pack_rc_channels((const uint16_t[]){1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}, prc.channels);

    air_adapter.ingest_rc_frame(prc, g_mock_millis);
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Link must be ACTIVE");

    uint8_t fc_buf[26];
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 26, "get_fc_frame must return 26B when ACTIVE");

    // 2. Advance time past failsafe timeout (501 ms)
    advance_mock_time_ms(501); // t = 2501 ms
    air_adapter.update(g_mock_millis);

    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must be active after 501ms without frames");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "No pulses during failsafe");
    TEST_ASSERT(air_adapter.get_stats().failsafe_events == 1, "failsafe_events must increment to 1");

    // 3. Restoration Frame 1 at t = 2590ms (+89ms): failsafe still active
    advance_mock_time_ms(89);
    prc.rc_sequence = 2;
    air_adapter.ingest_rc_frame(prc, g_mock_millis);

    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must remain active on restore frame 1");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "No pulses on restore frame 1");

    // 4. Restoration Frame 2 at t = 2680ms (+90ms): failsafe still active
    advance_mock_time_ms(90);
    prc.rc_sequence = 3;
    air_adapter.ingest_rc_frame(prc, g_mock_millis);

    TEST_ASSERT(air_adapter.is_failsafe_active(), "Failsafe must remain active on restore frame 2");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "No pulses on restore frame 2");

    // 5. Restoration Frame 3 at t = 2770ms (+90ms): 3 consecutive frames achieved!
    advance_mock_time_ms(90);
    prc.rc_sequence = 4;
    air_adapter.ingest_rc_frame(prc, g_mock_millis);

    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Failsafe must clear on restore frame 3");
    TEST_ASSERT(air_adapter.get_stats().restore_events == 1, "restore_events must increment to 1");
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 26, "CRSF pulses must immediately resume on restore frame 3");

    std::cout << "  [PASS] test_failsafe_watchdog_and_three_frame_restore\n";
    return true;
}

// =============================================================================
// TEST 4: CORRUPTED RF FRAME RESILIENCE
// Bit errors / bad CRC in RF transmission must be rejected by Air TdmEngine;
// Air adapter must not ingest corrupted frames, failsafe triggers correctly.
// =============================================================================
bool test_corrupted_rf_frame_resilience() {
    reset_rf_bus();
    memset(&s_air_rx_stats, 0, sizeof(s_air_rx_stats));
    set_mock_time_ms(3000);

    E22Driver ground_radio(dummy_serial_ground, 0, 0, 0);
    E22Driver air_radio(dummy_serial_air, 0, 0, 0);

    TdmEngine ground_tdm(ground_radio, NodeRole::GROUND);
    TdmEngine air_tdm(air_radio, NodeRole::AIR);
    air_tdm.onTransportFrameReceived(test_air_transport_callback);

    ground_tdm.begin();
    air_tdm.begin();

    RcAirAdapter air_adapter;

    // Send valid frame at t = 3000ms
    TransportPackedRc prc{};
    transport_pack_rc_channels((const uint16_t[]){1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}, prc.channels);
    ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, (const uint8_t*)&prc, sizeof(TransportPackedRc));

    air_tdm.update();
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "First frame received cleanly");
    air_adapter.ingest_rc_frame(s_air_rx_stats.last_packed_rc, g_mock_millis);
    TEST_ASSERT(!air_adapter.is_failsafe_active(), "Link active");

    // Send corrupted frame at t = 3090ms (tamper with payload byte in RF buffer)
    advance_mock_time_ms(90);
    ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, (const uint8_t*)&prc, sizeof(TransportPackedRc));

    TEST_ASSERT(!s_rf_ground_to_air.empty(), "RF bytes present");
    // Corrupt one byte of payload in the middle
    s_rf_ground_to_air[20] ^= 0xFF;

    air_tdm.update();
    // Callback must NOT have fired because CRC check failed
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "Air must NOT receive corrupted RF frame");
    TEST_ASSERT(air_tdm.getStats().crc_errors == 1, "Air TDM crc_errors stat must increment");

    // After 500ms of corrupt RF frames, Air enters failsafe
    advance_mock_time_ms(500);
    air_adapter.update(g_mock_millis);
    TEST_ASSERT(air_adapter.is_failsafe_active(), "Air must enter failsafe when RF is corrupt");

    uint8_t fc_buf[26];
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 0, "No pulses during failsafe caused by corrupt RF");

    std::cout << "  [PASS] test_corrupted_rf_frame_resilience\n";
    return true;
}

// =============================================================================
// TEST 5: SBUS HANDSET INGEST TO AIR CRSF OUTPUT
// Verifies OpenI6X SBUS output mode on Ground -> Air 26B CRSF conversion
// =============================================================================
bool test_sbus_handset_to_air_crsf() {
    reset_rf_bus();
    memset(&s_air_rx_stats, 0, sizeof(s_air_rx_stats));
    set_mock_time_ms(4000);

    E22Driver ground_radio(dummy_serial_ground, 0, 0, 0);
    E22Driver air_radio(dummy_serial_air, 0, 0, 0);

    TdmEngine ground_tdm(ground_radio, NodeRole::GROUND);
    TdmEngine air_tdm(air_radio, NodeRole::AIR);
    air_tdm.onTransportFrameReceived(test_air_transport_callback);

    ground_tdm.begin();
    air_tdm.begin();

    RcGroundAdapter ground_adapter;
    ground_adapter.enable_sbus(true);
    RcAirAdapter air_adapter;

    // Simulate standard 25-byte SBUS frame from handset
    uint16_t handset_ch[16];
    for (size_t i = 0; i < 16; ++i) handset_ch[i] = 1100 + (uint16_t)(i * 45);

    uint8_t sbus_frame[25];
    sbus_frame[0] = 0x0F; // SBUS start
    transport_pack_rc_channels(handset_ch, &sbus_frame[1]);
    sbus_frame[23] = 0x00; // flags
    sbus_frame[24] = 0x00; // end

    for (size_t i = 0; i < 25; ++i) {
        ground_adapter.feed_byte(sbus_frame[i], g_mock_millis);
    }
    TEST_ASSERT(ground_adapter.is_using_sbus(), "Ground adapter must detect SBUS frame");
    TEST_ASSERT(ground_adapter.has_handset_signal(g_mock_millis), "Handset active via SBUS");

    TransportPackedRc prc{};
    bool ok = ground_adapter.get_packed_rc(&prc, g_mock_millis);
    TEST_ASSERT(ok, "get_packed_rc must succeed for SBUS");

    ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                  TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                  0, 0, (const uint8_t*)&prc, sizeof(TransportPackedRc));

    air_tdm.update();
    TEST_ASSERT(s_air_rx_stats.rc_frames_received == 1, "Air must receive RF frame from SBUS Ground");
    air_adapter.ingest_rc_frame(s_air_rx_stats.last_packed_rc, g_mock_millis);

    // Verify channel fidelity on Air
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(air_adapter.get_channels()[i] == handset_ch[i], "Air channel mismatch from SBUS handset");
    }

    // Verify 26B CRSF frame generation for FC
    uint8_t fc_buf[26];
    TEST_ASSERT(air_adapter.get_fc_frame(fc_buf) == 26, "Air generates 26B CRSF frame from SBUS input");
    TEST_ASSERT(fc_buf[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER, "FC sync byte 0xC8");

    uint16_t decoded[16];
    TEST_ASSERT(crsf_unpack_channels22(&fc_buf[3], decoded), "FC CRSF unpack succeeds");
    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT(decoded[i] == handset_ch[i], "FC decoded channel mismatch from SBUS handset");
    }

    std::cout << "  [PASS] test_sbus_handset_to_air_crsf\n";
    return true;
}

// =============================================================================
// MAIN TEST RUNNER
// =============================================================================
int main() {
    std::cout << "===============================================================\n";
    std::cout << "Running Dual-LRS Step 3.1.2 Production RC Dispatch Host Tests\n";
    std::cout << "===============================================================\n";

    if (!test_end_to_end_rc_dispatch_fidelity()) return 1;
    if (!test_handset_timeout_and_beacon_fallback()) return 1;
    if (!test_failsafe_watchdog_and_three_frame_restore()) return 1;
    if (!test_corrupted_rf_frame_resilience()) return 1;
    if (!test_sbus_handset_to_air_crsf()) return 1;

    std::cout << "===============================================================\n";
    std::cout << "ALL STEP 3.1.2 PRODUCTION RC DISPATCH TESTS PASSED!\n";
    std::cout << "===============================================================\n";
    return 0;
}
