// Comprehensive Host-Side Unit Tests for Production TdmEngine (Step 3.1.1)
// Verifies 90ms schedule boundaries, role transmit gates, Air sync state machine,
// frame wire formatting (13B header, 49B max payload, 64B max frame), and frame rejection.

#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>

#include "Arduino.h"
#include "e22_driver.h"
#include "tdm_engine.h"
#include "transport_protocol.h"

uint32_t g_mock_micros = 0;
uint32_t g_mock_millis = 0;

void set_mock_time_us(uint32_t us) {
    g_mock_micros = us;
    g_mock_millis = us / 1000;
}

void advance_mock_time_us(uint32_t delta_us) {
    g_mock_micros += delta_us;
    g_mock_millis = g_mock_micros / 1000;
}

// ---------------------------------------------------------------------------
// Mock E22Driver Implementation for Host Test
// ---------------------------------------------------------------------------
HardwareSerial dummy_serial;
static std::vector<uint8_t> s_radio_rx_queue;
static std::vector<uint8_t> s_radio_tx_history;
static bool s_radio_busy = false;

E22Driver::E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux)
    : _serial(serialPort), _pinM0(pinM0), _pinM1(pinM1), _pinAux(pinAux), _currentMode(E22Mode::NORMAL) {}

void E22Driver::begin(uint32_t) {}
bool E22Driver::beginPassive(uint32_t) { return true; }
void E22Driver::setMode(E22Mode) {}
bool E22Driver::configureRadio(uint8_t, uint8_t) { return true; }
E22RegReadStatus E22Driver::readRegistersReadOnly(uint8_t*, size_t, size_t&) { return E22RegReadStatus::OK; }
bool E22Driver::isBusy() const { return s_radio_busy; }
bool E22Driver::waitForReady(uint32_t) { return !s_radio_busy; }

size_t E22Driver::write(const uint8_t* data, size_t length) {
    s_radio_tx_history.insert(s_radio_tx_history.end(), data, data + length);
    return length;
}

int E22Driver::available() {
    return (int)s_radio_rx_queue.size();
}

int E22Driver::read() {
    if (s_radio_rx_queue.empty()) return -1;
    uint8_t b = s_radio_rx_queue.front();
    s_radio_rx_queue.erase(s_radio_rx_queue.begin());
    return b;
}

size_t E22Driver::readBytes(uint8_t* buffer, size_t length) {
    size_t count = 0;
    while (count < length && !s_radio_rx_queue.empty()) {
        buffer[count++] = s_radio_rx_queue.front();
        s_radio_rx_queue.erase(s_radio_rx_queue.begin());
    }
    return count;
}

void E22Driver::flush() {}

void reset_mock_radio() {
    s_radio_rx_queue.clear();
    s_radio_tx_history.clear();
    s_radio_busy = false;
}

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

// =============================================================================
// 1. TEST 90MS CYCLE AND EXACT SLOT BOUNDARIES
// =============================================================================
bool test_90ms_schedule_boundaries() {
    reset_mock_radio();
    set_mock_time_us(0);

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine ground_tdm(radio, NodeRole::GROUND);
    ground_tdm.begin();

    // Schedule:
    // Slot 1 (Ground TX): 0 .. 32,000 us (32 ms)
    // Guard Gap 1:       32,000 .. 37,000 us (5 ms)
    // Slot 2 (Air TX):   37,000 .. 82,000 us (45 ms)
    // Guard Gap 2:       82,000 .. 90,000 us (8 ms)

    // At t = 0
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GROUND_TRANSMIT, "t=0 must be GROUND_TRANSMIT");
    TEST_ASSERT(ground_tdm.canTransmit(), "Ground canTransmit at t=0");

    // At t = 16,000 us (mid ground slot)
    set_mock_time_us(16000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GROUND_TRANSMIT, "t=16ms must be GROUND_TRANSMIT");
    TEST_ASSERT(ground_tdm.canTransmit(), "Ground canTransmit at t=16ms");

    // At t = 31,999 us (boundary before guard 1)
    set_mock_time_us(31999);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GROUND_TRANSMIT, "t=31.999ms must be GROUND_TRANSMIT");
    TEST_ASSERT(ground_tdm.canTransmit(), "Ground canTransmit at t=31.999ms");

    // At t = 32,000 us (start of Guard Gap 1)
    set_mock_time_us(32000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GUARD_GAP_1, "t=32ms must be GUARD_GAP_1");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in GUARD_GAP_1");

    // At t = 36,999 us (end of Guard Gap 1)
    set_mock_time_us(36999);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GUARD_GAP_1, "t=36.999ms must be GUARD_GAP_1");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in GUARD_GAP_1");

    // At t = 37,000 us (start of Air TX slot)
    set_mock_time_us(37000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::AIR_TRANSMIT, "t=37ms must be AIR_TRANSMIT");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in AIR_TRANSMIT slot");

    // At t = 81,999 us (end of Air TX slot)
    set_mock_time_us(81999);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::AIR_TRANSMIT, "t=81.999ms must be AIR_TRANSMIT");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in AIR_TRANSMIT slot");

    // At t = 82,000 us (start of Guard Gap 2)
    set_mock_time_us(82000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GUARD_GAP_2, "t=82ms must be GUARD_GAP_2");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in GUARD_GAP_2");

    // At t = 89,999 us (end of Guard Gap 2)
    set_mock_time_us(89999);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GUARD_GAP_2, "t=89.999ms must be GUARD_GAP_2");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in GUARD_GAP_2");

    // At t = 90,000 us (wrap to cycle 2 start)
    set_mock_time_us(90000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::GROUND_TRANSMIT, "t=90ms must wrap to GROUND_TRANSMIT");
    TEST_ASSERT(ground_tdm.canTransmit(), "Ground canTransmit at t=90ms");

    // Multi-cycle wrap check at t = 270,000 + 40,000 us = 310,000 us (cycle 4, air slot)
    set_mock_time_us(310000);
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.getCurrentSlot() == TdmSlot::AIR_TRANSMIT, "t=310ms must be AIR_TRANSMIT");
    TEST_ASSERT(!ground_tdm.canTransmit(), "Ground must not transmit in cycle 4 AIR_TRANSMIT");

    std::cout << "  [PASS] test_90ms_schedule_boundaries\n";
    return true;
}

// =============================================================================
// 2. TEST AIR UNSYNCHRONIZED SAFETY
// =============================================================================
bool test_air_unsynchronized_safety() {
    reset_mock_radio();
    set_mock_time_us(0);

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine air_tdm(radio, NodeRole::AIR);
    air_tdm.begin();

    TEST_ASSERT(!air_tdm.getStats().synchronized, "Air node must boot unsynchronized");

    // Even inside Air slot (t = 50ms), unsynchronized Air must NOT transmit
    set_mock_time_us(50000);
    air_tdm.update();
    TEST_ASSERT(air_tdm.getCurrentSlot() == TdmSlot::AIR_TRANSMIT, "Slot must be AIR_TRANSMIT");
    TEST_ASSERT(!air_tdm.canTransmit(), "Unsynchronized Air MUST NOT be allowed to transmit");

    // Calling sendPacket must fail and write nothing
    uint8_t dummy_payload[10] = {1, 2, 3};
    bool sent = air_tdm.sendPacket(LrsPacketType::MAVLINK_DATA, dummy_payload, 3);
    TEST_ASSERT(!sent, "sendPacket must return false when unsynchronized");
    TEST_ASSERT(s_radio_tx_history.empty(), "No bytes should be written to radio");

    // Calling sendTransportFrame directly must fail
    sent = air_tdm.sendTransportFrame(TransportChannel::MAVLINK_DOWNLINK, 0, 1, 0, dummy_payload, 3);
    TEST_ASSERT(!sent, "sendTransportFrame must return false when unsynchronized");
    TEST_ASSERT(s_radio_tx_history.empty(), "No bytes should be written to radio");

    std::cout << "  [PASS] test_air_unsynchronized_safety\n";
    return true;
}

// =============================================================================
// 3. TEST AIR SYNCHRONIZATION AND DOWNLINK TRANSMISSION
// =============================================================================
bool test_air_sync_and_transmission() {
    reset_mock_radio();
    set_mock_time_us(10200); // 10.2 ms

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine air_tdm(radio, NodeRole::AIR);
    air_tdm.begin();

    // Construct Ground sync beacon frame (LINK_CONTROL, 0B payload)
    TransportHeader gnd_hdr{};
    gnd_hdr.magic0 = TRANSPORT_MAGIC0;
    gnd_hdr.magic1 = TRANSPORT_MAGIC1;
    gnd_hdr.version = TRANSPORT_VERSION;
    gnd_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    gnd_hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    gnd_hdr.sequence = 1;
    gnd_hdr.transfer_id = 0;
    gnd_hdr.fragment_offset = 0;
    gnd_hdr.payload_length = 0;

    uint8_t wire[64];
    transport_encode_header(&gnd_hdr, wire);
    uint16_t crc = transport_crc16(wire, 13);
    transport_write_u16_be(&wire[13], crc);

    // Inject 15 bytes into radio RX queue
    for (size_t i = 0; i < 15; ++i) {
        s_radio_rx_queue.push_back(wire[i]);
    }

    // Process incoming radio data
    air_tdm.update();

    TEST_ASSERT(air_tdm.getStats().synchronized, "Air node must become synchronized upon valid Ground frame");
    TEST_ASSERT(air_tdm.getStats().packets_received == 1, "packets_received must increment to 1");

    // Advance into Air downlink slot (t = 50,000 us)
    set_mock_time_us(50000);
    air_tdm.update();

    TEST_ASSERT(air_tdm.getCurrentSlot() == TdmSlot::AIR_TRANSMIT, "Must be AIR_TRANSMIT slot");
    TEST_ASSERT(air_tdm.canTransmit(), "Synchronized Air CAN transmit in AIR_TRANSMIT slot");

    // Transmit link control / downlink status frame
    uint8_t status_payload[12] = {0xAA, 0xBB, 0xCC, 0xDD, 1, 2, 3, 4, 5, 6, 7, 8};
    bool sent = air_tdm.sendTransportFrame(TransportChannel::LINK_CONTROL,
                                           TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                           42, 0, status_payload, 12);
    TEST_ASSERT(sent, "sendTransportFrame must succeed");
    TEST_ASSERT(!s_radio_tx_history.empty(), "Radio must have transmitted bytes");

    // Verify wire frame structure: 13 + 12 + 2 = 27 bytes
    TEST_ASSERT(s_radio_tx_history.size() == 27, "Wire frame size must be 27 bytes");
    TEST_ASSERT(s_radio_tx_history[0] == TRANSPORT_MAGIC0, "Magic0 mismatch");
    TEST_ASSERT(s_radio_tx_history[1] == TRANSPORT_MAGIC1, "Magic1 mismatch");
    TEST_ASSERT(s_radio_tx_history[2] == TRANSPORT_VERSION, "Version mismatch");
    TEST_ASSERT(s_radio_tx_history[3] == (uint8_t)TransportChannel::LINK_CONTROL, "Channel mismatch");

    // Verify CRC of transmitted frame
    uint16_t tx_crc = transport_crc16(s_radio_tx_history.data(), 25);
    uint16_t wire_crc = transport_read_u16_be(&s_radio_tx_history[25]);
    TEST_ASSERT(tx_crc == wire_crc, "Transmitted frame CRC mismatch");

    // Advance into Guard Gap 2 (t = 85,000 us)
    set_mock_time_us(85000);
    air_tdm.update();
    TEST_ASSERT(air_tdm.getCurrentSlot() == TdmSlot::GUARD_GAP_2, "Must be GUARD_GAP_2");
    TEST_ASSERT(!air_tdm.canTransmit(), "Air must NOT transmit in GUARD_GAP_2");

    std::cout << "  [PASS] test_air_sync_and_transmission\n";
    return true;
}

// =============================================================================
// 4. TEST SYNC TIMEOUT (1.5 SECONDS WITHOUT GROUND BEACON)
// =============================================================================
bool test_air_sync_timeout() {
    reset_mock_radio();
    set_mock_time_us(10200);

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine air_tdm(radio, NodeRole::AIR);
    air_tdm.begin();

    // Inject Ground frame to synchronize
    TransportHeader gnd_hdr{};
    gnd_hdr.magic0 = TRANSPORT_MAGIC0;
    gnd_hdr.magic1 = TRANSPORT_MAGIC1;
    gnd_hdr.version = TRANSPORT_VERSION;
    gnd_hdr.channel = (uint8_t)TransportChannel::LINK_CONTROL;
    gnd_hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    gnd_hdr.sequence = 1;
    uint8_t wire[15];
    transport_encode_header(&gnd_hdr, wire);
    uint16_t crc = transport_crc16(wire, 13);
    transport_write_u16_be(&wire[13], crc);
    for (size_t i = 0; i < 15; ++i) s_radio_rx_queue.push_back(wire[i]);

    air_tdm.update();
    TEST_ASSERT(air_tdm.getStats().synchronized, "Air must be synchronized");

    // Advance 1400 ms: still synchronized
    set_mock_time_us(1400000);
    air_tdm.update();
    TEST_ASSERT(air_tdm.getStats().synchronized, "Air should remain synchronized at 1400ms");

    // Advance to 1600 ms (> 1500 ms timeout): must drop sync
    set_mock_time_us(1600000);
    air_tdm.update();
    TEST_ASSERT(!air_tdm.getStats().synchronized, "Air must drop synchronization after 1500ms timeout");

    std::cout << "  [PASS] test_air_sync_timeout\n";
    return true;
}

// =============================================================================
// 5. TEST FRAME WIRE CONSTRAINTS AND CLAMPING
// =============================================================================
bool test_frame_constraints_and_clamping() {
    reset_mock_radio();
    set_mock_time_us(0);

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine ground_tdm(radio, NodeRole::GROUND);
    ground_tdm.begin();

    // 1. Send normal frame (24-byte RC payload)
    uint8_t rc_payload[24];
    for (size_t i = 0; i < 24; ++i) rc_payload[i] = (uint8_t)(i + 1);

    bool ok = ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                            TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                            1, 0, rc_payload, 24);
    TEST_ASSERT(ok, "sendTransportFrame failed");
    // Wire size: 13 header + 24 payload + 2 CRC = 39 bytes (exact RC frame size)
    TEST_ASSERT(s_radio_tx_history.size() == 39, "RC wire frame size must be exactly 39 bytes");
    TEST_ASSERT(s_radio_tx_history[0] == TRANSPORT_MAGIC0, "Magic0 mismatch");
    TEST_ASSERT(s_radio_tx_history[1] == TRANSPORT_MAGIC1, "Magic1 mismatch");
    TEST_ASSERT(s_radio_tx_history[3] == (uint8_t)TransportChannel::RC_CONTROL, "Channel mismatch");

    // 2. Send oversized payload (> 49 bytes): must clamp to 49 bytes
    s_radio_tx_history.clear();
    uint8_t oversized[100];
    memset(oversized, 0xEE, sizeof(oversized));

    ok = ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                       TRANSPORT_FLAG_FIRST_FRAG,
                                       2, 0, oversized, 100);
    TEST_ASSERT(ok, "sendTransportFrame oversized failed");
    // Wire size: 13 header + 49 clamped payload + 2 CRC = 64 bytes (exact TRANSPORT_MAX_FRAME_SIZE)
    TEST_ASSERT(s_radio_tx_history.size() == TRANSPORT_MAX_FRAME_SIZE, "Frame must be clamped to 64 bytes");
    uint16_t wire_plen = transport_read_u16_be(&s_radio_tx_history[11]);
    TEST_ASSERT(wire_plen == TRANSPORT_MAX_SINGLE_BURST_PAYLOAD, "Payload len in header must be 49");

    uint16_t calc_crc = transport_crc16(s_radio_tx_history.data(), 62);
    uint16_t frame_crc = transport_read_u16_be(&s_radio_tx_history[62]);
    TEST_ASSERT(calc_crc == frame_crc, "Clamped frame CRC mismatch");

    std::cout << "  [PASS] test_frame_constraints_and_clamping\n";
    return true;
}

// =============================================================================
// 6. TEST REJECTION OF INCOMPATIBLE / LEGACY FRAMES
// =============================================================================
static uint32_t s_legacy_cb_count = 0;
static uint32_t s_native_cb_count = 0;

void mock_legacy_cb(LrsPacketType, const uint8_t*, uint8_t) {
    s_legacy_cb_count++;
}

void mock_native_cb(const TransportHeader&, const uint8_t*, uint16_t) {
    s_native_cb_count++;
}

bool test_parser_rejection_of_incompatible_frames() {
    reset_mock_radio();
    set_mock_time_us(0);
    s_legacy_cb_count = 0;
    s_native_cb_count = 0;

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine tdm(radio, NodeRole::AIR);
    tdm.begin();
    tdm.onPacketReceived(mock_legacy_cb);
    tdm.onTransportFrameReceived(mock_native_cb);

    // 1. Incompatible legacy 5-byte header frame: 'D' 'L' packet_type=2 seq=5 len=10
    // LrsFrameHeader was 5 bytes, no version byte, different CRC scheme
    uint8_t legacy_frame[17] = {'D', 'L', 0x02, 0x05, 10,
                                1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                0x12, 0x34};
    for (size_t i = 0; i < sizeof(legacy_frame); ++i) {
        s_radio_rx_queue.push_back(legacy_frame[i]);
    }

    tdm.update();

    TEST_ASSERT(s_native_cb_count == 0, "Native callback must NOT fire for legacy frame");
    TEST_ASSERT(s_legacy_cb_count == 0, "Legacy callback must NOT fire for legacy frame");
    TEST_ASSERT(tdm.getStats().packets_received == 0, "packets_received must remain 0");

    // 2. Corrupted CRC on valid header
    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)TransportChannel::RC_CONTROL;
    hdr.flags = TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG;
    hdr.payload_length = 4;
    uint8_t wire[19];
    transport_encode_header(&hdr, wire);
    wire[13] = 0x11; wire[14] = 0x22; wire[15] = 0x33; wire[16] = 0x44;
    wire[17] = 0xDE; wire[18] = 0xAD; // Intentionally bad CRC

    for (size_t i = 0; i < 19; ++i) {
        s_radio_rx_queue.push_back(wire[i]);
    }

    tdm.update();

    TEST_ASSERT(s_native_cb_count == 0, "Bad CRC frame must be rejected");
    TEST_ASSERT(s_legacy_cb_count == 0, "Bad CRC frame must be rejected");
    TEST_ASSERT(tdm.getStats().packets_received == 0, "packets_received must remain 0");

    // 3. Valid Phase 2 frame with matching CRC
    uint16_t good_crc = transport_crc16(wire, 17);
    transport_write_u16_be(&wire[17], good_crc);

    for (size_t i = 0; i < 19; ++i) {
        s_radio_rx_queue.push_back(wire[i]);
    }

    tdm.update();

    TEST_ASSERT(s_native_cb_count == 1, "Native callback must fire for valid Phase 2 frame");
    TEST_ASSERT(s_legacy_cb_count == 1, "Legacy callback must fire for valid Phase 2 frame");
    TEST_ASSERT(tdm.getStats().packets_received == 1, "packets_received must increment to 1");

    std::cout << "  [PASS] test_parser_rejection_of_incompatible_frames\n";
    return true;
}

// =============================================================================
// 7. TEST STAGE 3.1 RC-ONLY MAVLINK RF ISOLATION
// =============================================================================
bool test_stage31_rc_only_mavlink_isolation() {
    reset_mock_radio();
    set_mock_time_us(0);

    E22Driver radio(dummy_serial, 0, 0, 0);
    TdmEngine ground_tdm(radio, NodeRole::GROUND);
    ground_tdm.begin();

    // In Ground slot (canTransmit is true)
    ground_tdm.update();
    TEST_ASSERT(ground_tdm.canTransmit(), "Ground canTransmit at t=0");

    // 1. Attempt to send MAVLINK_DATA via legacy sendPacket()
    uint8_t dummy_mavlink[16] = {0xFD, 9, 0, 0, 1, 1, 1, 0, 0, 0, 1, 2, 3, 4, 5, 6};
    bool sent = ground_tdm.sendPacket(LrsPacketType::MAVLINK_DATA, dummy_mavlink, sizeof(dummy_mavlink));
    TEST_ASSERT(!sent, "sendPacket(MAVLINK_DATA) MUST be rejected in Stage 3.1 RC-only mode");
    TEST_ASSERT(s_radio_tx_history.empty(), "Zero bytes must be written to radio for MAVLINK_DATA");

    // 2. Attempt to send native MAVLINK_UPLINK via sendTransportFrame()
    sent = ground_tdm.sendTransportFrame(TransportChannel::MAVLINK_UPLINK, 0, 1, 0, dummy_mavlink, sizeof(dummy_mavlink));
    TEST_ASSERT(!sent, "sendTransportFrame(MAVLINK_UPLINK) MUST be rejected in Stage 3.1 RC-only mode");
    TEST_ASSERT(s_radio_tx_history.empty(), "Zero bytes must be written to radio for MAVLINK_UPLINK");

    // 3. Attempt to send native MAVLINK_DOWNLINK via sendTransportFrame()
    sent = ground_tdm.sendTransportFrame(TransportChannel::MAVLINK_DOWNLINK, 0, 1, 0, dummy_mavlink, sizeof(dummy_mavlink));
    TEST_ASSERT(!sent, "sendTransportFrame(MAVLINK_DOWNLINK) MUST be rejected in Stage 3.1 RC-only mode");
    TEST_ASSERT(s_radio_tx_history.empty(), "Zero bytes must be written to radio for MAVLINK_DOWNLINK");

    // 4. Verify RC_CONTROL and LINK_CONTROL ARE permitted
    sent = ground_tdm.sendTransportFrame(TransportChannel::RC_CONTROL, 0, 1, 0, dummy_mavlink, 10);
    TEST_ASSERT(sent, "RC_CONTROL must be allowed in Stage 3.1 RC-only mode");
    TEST_ASSERT(!s_radio_tx_history.empty(), "Bytes must be written to radio for RC_CONTROL");

    s_radio_tx_history.clear();
    sent = ground_tdm.sendTransportFrame(TransportChannel::LINK_CONTROL, 0, 1, 0, nullptr, 0);
    TEST_ASSERT(sent, "LINK_CONTROL must be allowed in Stage 3.1 RC-only mode");
    TEST_ASSERT(!s_radio_tx_history.empty(), "Bytes must be written to radio for LINK_CONTROL");

    std::cout << "  [PASS] test_stage31_rc_only_mavlink_isolation\n";
    return true;
}

// =============================================================================
// MAIN TEST RUNNER
// =============================================================================
int main() {
    std::cout << "====================================================\n";
    std::cout << "Running Dual-LRS Step 3.1.1 Production TDM Host Tests\n";
    std::cout << "====================================================\n";

    if (!test_90ms_schedule_boundaries()) return 1;
    if (!test_air_unsynchronized_safety()) return 1;
    if (!test_air_sync_and_transmission()) return 1;
    if (!test_air_sync_timeout()) return 1;
    if (!test_frame_constraints_and_clamping()) return 1;
    if (!test_parser_rejection_of_incompatible_frames()) return 1;
    if (!test_stage31_rc_only_mavlink_isolation()) return 1;

    std::cout << "====================================================\n";
    std::cout << "ALL STEP 3.1.1 PRODUCTION TDM TESTS PASSED!\n";
    std::cout << "====================================================\n";
    return 0;
}
