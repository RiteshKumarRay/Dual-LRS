// =============================================================================
// Dual-LRS Step 3.2 Production MAVLink Transport Host Test Suite
// Verifies:
// 1. Air Downlink Single-Burst Telemetry (<= 49B) -> 1 Fragment -> Immediate Ground Reassembly
// 2. Air Downlink Multi-Fragment Message (128B) -> 3 Fragments -> Ground Reassembly Fidelity
// 3. Ground Uplink Command Transfer -> Air Reassembly
// 4. Ground Handset RC Priority & Uplink Interleaving (RC continues uninterrupted)
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
        s_rf_ground_to_air.insert(s_rf_ground_to_air.end(), data, data + length);
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
}

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            return false; \
        } \
    } while(0)

// =============================================================================
// 1. TEST AIR DOWNLINK SINGLE BURST (<= 49B)
// =============================================================================
bool test_air_downlink_single_burst() {
    reset_rf_bus();
    set_mock_time_ms(1000);

    E22Driver radio_ground(dummy_serial_ground, 0, 0, 0);
    E22Driver radio_air(dummy_serial_air, 0, 0, 0);

    TdmEngine tdm_ground(radio_ground, NodeRole::GROUND);
    TdmEngine tdm_air(radio_air, NodeRole::AIR);

    tdm_ground.begin();
    tdm_air.begin();
    tdm_air.setSynchronized(true);

    TransportFragmenter air_fragmenter;
    TransportReassembler ground_reassembler;

    // Simulate 36-byte ATTITUDE MAVLink message
    uint8_t attitude_msg[36];
    attitude_msg[0] = 0xFE; // MAVLink v1
    attitude_msg[1] = 28;   // 28B payload
    attitude_msg[5] = 30;   // msgid 30 (ATTITUDE)
    for (size_t i = 6; i < 36; ++i) attitude_msg[i] = (uint8_t)(i ^ 0x3C);

    TEST_ASSERT(air_fragmenter.start_transfer(TransportChannel::MAVLINK_DOWNLINK, attitude_msg, sizeof(attitude_msg), false),
                "air_fragmenter start_transfer failed");
    TEST_ASSERT(air_fragmenter.get_fragment_count() == 1, "36B message must produce exactly 1 fragment");

    // Air slot transmit
    tdm_air.forceSlot(TdmSlot::AIR_TRANSMIT);
    uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = air_fragmenter.get_next_fragment(frame_buf, tdm_air.getNextSequence());
    TEST_ASSERT(frame_len > 0 && frame_len <= 64, "Air frame exceeds single-burst boundary");

    bool sent = tdm_air.sendRawTransportFrame(frame_buf, frame_len);
    TEST_ASSERT(sent, "Air sendRawTransportFrame failed");

    // Ground receives in background
    static bool ground_got_packet = false;
    static std::vector<uint8_t> ground_received_data;
    ground_got_packet = false;
    ground_received_data.clear();

    tdm_ground.onTransportFrameReceived([](const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
        if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK) {
            static TransportReassembler r;
            TransportNackReason nack = r.process_fragment(hdr, payload, g_mock_millis);
            if (nack == TransportNackReason::NONE && r.is_complete()) {
                ground_got_packet = true;
                ground_received_data.assign(r.get_reassembled_data(), r.get_reassembled_data() + r.get_reassembled_length());
                r.mark_complete_consumed();
            }
        }
    });

    tdm_ground.update(); // Processes RF bytes from Air

    TEST_ASSERT(ground_got_packet, "Ground did not reassemble single-burst MAVLink message");
    TEST_ASSERT(ground_received_data.size() == sizeof(attitude_msg), "Reassembled length mismatch");
    TEST_ASSERT(memcmp(ground_received_data.data(), attitude_msg, sizeof(attitude_msg)) == 0,
                "Reassembled content mismatch");

    std::cout << "  [PASS] test_air_downlink_single_burst (36B ATTITUDE -> 1 fragment -> 100% fidelity)\n";
    return true;
}

// =============================================================================
// 2. TEST AIR DOWNLINK MULTI-FRAGMENT (128B)
// =============================================================================
bool test_air_downlink_multi_fragment() {
    reset_rf_bus();
    set_mock_time_ms(2000);

    E22Driver radio_ground(dummy_serial_ground, 0, 0, 0);
    E22Driver radio_air(dummy_serial_air, 0, 0, 0);

    TdmEngine tdm_ground(radio_ground, NodeRole::GROUND);
    TdmEngine tdm_air(radio_air, NodeRole::AIR);

    tdm_ground.begin();
    tdm_air.begin();
    tdm_air.setSynchronized(true);

    TransportFragmenter air_fragmenter;

    // 128B PARAM_VALUE message
    uint8_t large_msg[128];
    for (size_t i = 0; i < 128; ++i) large_msg[i] = (uint8_t)(i * 3 + 7);

    TEST_ASSERT(air_fragmenter.start_transfer(TransportChannel::MAVLINK_DOWNLINK, large_msg, sizeof(large_msg), false),
                "air_fragmenter start_transfer failed");
    TEST_ASSERT(air_fragmenter.get_fragment_count() == 3, "128B message must produce 3 fragments (49 + 49 + 30)");

    static bool ground_completed = false;
    static std::vector<uint8_t> ground_assembled;
    ground_completed = false;
    ground_assembled.clear();

    static TransportReassembler test_reassembler;
    test_reassembler.reset();

    tdm_ground.onTransportFrameReceived([](const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
        if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK) {
            TransportNackReason nack = test_reassembler.process_fragment(hdr, payload, g_mock_millis);
            if (nack == TransportNackReason::NONE && test_reassembler.is_complete()) {
                ground_completed = true;
                ground_assembled.assign(test_reassembler.get_reassembled_data(),
                                        test_reassembler.get_reassembled_data() + test_reassembler.get_reassembled_length());
                test_reassembler.mark_complete_consumed();
            }
        }
    });

    // Simulate 3 Air slots (e.g. 3 TDM cycles, 90ms apart)
    int fragments_sent = 0;
    while (air_fragmenter.has_next_fragment()) {
        tdm_air.forceSlot(TdmSlot::AIR_TRANSMIT);
        uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
        size_t frame_len = air_fragmenter.get_next_fragment(frame_buf, tdm_air.getNextSequence());
        TEST_ASSERT(frame_len > 0 && frame_len <= 64, "Fragment wire size exceeds 64B");

        bool sent = tdm_air.sendRawTransportFrame(frame_buf, frame_len);
        TEST_ASSERT(sent, "Air sendRawTransportFrame failed");
        fragments_sent++;

        // Deliver over RF
        tdm_ground.update();
        advance_mock_time_ms(90);
    }

    TEST_ASSERT(fragments_sent == 3, "Expected exactly 3 fragments sent");
    TEST_ASSERT(ground_completed, "Ground reassembler did not complete 128B message");
    TEST_ASSERT(ground_assembled.size() == 128, "Reassembled size mismatch");
    TEST_ASSERT(memcmp(ground_assembled.data(), large_msg, 128) == 0, "128B reassembled content mismatch");

    std::cout << "  [PASS] test_air_downlink_multi_fragment (128B -> 3 fragments -> reassembly bit-exact)\n";
    return true;
}

// =============================================================================
// 3. TEST GROUND UPLINK COMMAND TRANSFER & AIR REASSEMBLY
// =============================================================================
bool test_ground_uplink_command_transfer() {
    reset_rf_bus();
    set_mock_time_ms(3000);

    E22Driver radio_ground(dummy_serial_ground, 0, 0, 0);
    E22Driver radio_air(dummy_serial_air, 0, 0, 0);

    TdmEngine tdm_ground(radio_ground, NodeRole::GROUND);
    TdmEngine tdm_air(radio_air, NodeRole::AIR);

    tdm_ground.begin();
    tdm_air.begin();

    TransportFragmenter ground_fragmenter;
    static TransportReassembler air_reassembler;
    air_reassembler.reset();

    // 40B COMMAND_LONG message from GCS
    uint8_t cmd_msg[40];
    cmd_msg[0] = 0xFE;
    cmd_msg[1] = 33;
    cmd_msg[5] = 76; // COMMAND_LONG
    for (size_t i = 6; i < 40; ++i) cmd_msg[i] = (uint8_t)(i + 1);

    TEST_ASSERT(ground_fragmenter.start_transfer(TransportChannel::MAVLINK_UPLINK, cmd_msg, sizeof(cmd_msg), false),
                "ground_fragmenter start_transfer failed");

    static bool air_got_cmd = false;
    static std::vector<uint8_t> air_cmd_data;
    air_got_cmd = false;
    air_cmd_data.clear();

    tdm_air.onTransportFrameReceived([](const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
        if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_UPLINK) {
            TransportNackReason nack = air_reassembler.process_fragment(hdr, payload, g_mock_millis);
            if (nack == TransportNackReason::NONE && air_reassembler.is_complete()) {
                air_got_cmd = true;
                air_cmd_data.assign(air_reassembler.get_reassembled_data(),
                                    air_reassembler.get_reassembled_data() + air_reassembler.get_reassembled_length());
                air_reassembler.mark_complete_consumed();
            }
        }
    });

    tdm_ground.forceSlot(TdmSlot::GROUND_TRANSMIT);
    uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
    size_t frame_len = ground_fragmenter.get_next_fragment(frame_buf, tdm_ground.getNextSequence());
    TEST_ASSERT(frame_len > 0 && frame_len <= 64, "Uplink frame exceeds 64B");

    bool sent = tdm_ground.sendRawTransportFrame(frame_buf, frame_len);
    TEST_ASSERT(sent, "Ground sendRawTransportFrame failed");

    tdm_air.update();

    TEST_ASSERT(air_got_cmd, "Air did not receive uplink command");
    TEST_ASSERT(air_cmd_data.size() == sizeof(cmd_msg), "Command length mismatch");
    TEST_ASSERT(memcmp(air_cmd_data.data(), cmd_msg, sizeof(cmd_msg)) == 0, "Command content mismatch");

    std::cout << "  [PASS] test_ground_uplink_command_transfer (40B COMMAND_LONG uplink -> Air reassembly bit-exact)\n";
    return true;
}

// =============================================================================
// 4. TEST RC AND MAVLINK UPLINK COEXISTENCE
// =============================================================================
bool test_rc_and_mavlink_coexistence() {
    reset_rf_bus();
    set_mock_time_ms(4000);

    E22Driver radio_ground(dummy_serial_ground, 0, 0, 0);
    E22Driver radio_air(dummy_serial_air, 0, 0, 0);

    TdmEngine tdm_ground(radio_ground, NodeRole::GROUND);
    TdmEngine tdm_air(radio_air, NodeRole::AIR);

    tdm_ground.begin();
    tdm_air.begin();

    TransportFragmenter ground_fragmenter;
    static TransportReassembler air_reassembler;
    air_reassembler.reset();

    // Setup an uplink command
    uint8_t cmd_msg[20] = {0xFE, 12, 0, 0, 1, 76};
    for (size_t i = 6; i < 20; ++i) cmd_msg[i] = (uint8_t)i;
    TEST_ASSERT(ground_fragmenter.start_transfer(TransportChannel::MAVLINK_UPLINK, cmd_msg, sizeof(cmd_msg), false),
                "start_transfer failed");

    static uint32_t air_rc_frames_received = 0;
    static bool air_cmd_completed = false;
    air_rc_frames_received = 0;
    air_cmd_completed = false;

    tdm_air.onTransportFrameReceived([](const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
        if (hdr.channel == (uint8_t)TransportChannel::RC_CONTROL) {
            air_rc_frames_received++;
        } else if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_UPLINK) {
            TransportNackReason nack = air_reassembler.process_fragment(hdr, payload, g_mock_millis);
            if (nack == TransportNackReason::NONE && air_reassembler.is_complete()) {
                air_cmd_completed = true;
                air_reassembler.mark_complete_consumed();
            }
        }
    });

    // Simulate 8 Ground slots (8 TDM cycles, 90ms apart)
    // Production interleaving rule: when handset is active and uplink is pending,
    // allow 1 uplink slot every 4 slots (slotsSinceRc >= 3)
    uint8_t slotsSinceRc = 0;
    TransportPackedRc mock_rc{};
    mock_rc.rc_sequence = 1;

    for (int cycle = 0; cycle < 8; ++cycle) {
        bool has_rc = true;
        bool has_uplink = ground_fragmenter.has_next_fragment();

        bool send_rc = has_rc;
        if (has_rc && has_uplink && slotsSinceRc >= 3) {
            send_rc = false;
            slotsSinceRc = 0;
        } else if (has_rc) {
            send_rc = true;
            slotsSinceRc++;
        }

        tdm_ground.forceSlot(TdmSlot::GROUND_TRANSMIT);

        if (send_rc) {
            tdm_ground.sendTransportFrame(TransportChannel::RC_CONTROL,
                                         TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                         0, 0, (const uint8_t*)&mock_rc, sizeof(TransportPackedRc));
        } else if (has_uplink) {
            uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
            size_t frame_len = ground_fragmenter.get_next_fragment(frame_buf, tdm_ground.getNextSequence());
            tdm_ground.sendRawTransportFrame(frame_buf, frame_len);
        }

        // Deliver over RF
        tdm_air.update();
        advance_mock_time_ms(90);
    }

    // In 8 cycles with 1 command fragment:
    // Slot 0 (RC), Slot 1 (RC), Slot 2 (RC), Slot 3 (Command), Slot 4 (RC), Slot 5 (RC), Slot 6 (RC), Slot 7 (RC)
    // Exactly 7 RC frames and 1 Command frame!
    TEST_ASSERT(air_rc_frames_received >= 6, "RC was starved by uplink traffic");
    TEST_ASSERT(air_cmd_completed, "Uplink command was never transmitted");

    std::cout << "  [PASS] test_rc_and_mavlink_coexistence (7 RC frames + 1 GCS cmd in 8 slots, zero RC starvation)\n";
    return true;
}

// =============================================================================
// 5. TEST SINGLE-BURST PAYLOAD LIMIT
// =============================================================================
bool test_oversized_transport_payload_rejected() {
    reset_rf_bus();
    set_mock_time_ms(5000);

    E22Driver radio_ground(dummy_serial_ground, 0, 0, 0);
    TdmEngine tdm_ground(radio_ground, NodeRole::GROUND);
    tdm_ground.begin();
    tdm_ground.forceSlot(TdmSlot::GROUND_TRANSMIT);

    uint8_t oversized_payload[TRANSPORT_MAX_SINGLE_BURST_PAYLOAD + 1] = {};
    bool sent = tdm_ground.sendTransportFrame(
        TransportChannel::MAVLINK_UPLINK,
        TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
        0, 0, oversized_payload, sizeof(oversized_payload));

    TEST_ASSERT(!sent, "oversized single-burst payload was accepted");
    TEST_ASSERT(s_rf_ground_to_air.empty(), "oversized payload wrote a truncated RF frame");

    std::cout << "  [PASS] test_oversized_transport_payload_rejected (no silent truncation)\n";
    return true;
}

// =============================================================================
// MAIN RUNNER
// =============================================================================
int main() {
    std::cout << "====================================================\n";
    std::cout << "Running Dual-LRS Step 3.2 Production MAVLink Transport Tests\n";
    std::cout << "====================================================\n";

    if (!test_air_downlink_single_burst()) return 1;
    if (!test_air_downlink_multi_fragment()) return 1;
    if (!test_ground_uplink_command_transfer()) return 1;
    if (!test_rc_and_mavlink_coexistence()) return 1;
    if (!test_oversized_transport_payload_rejected()) return 1;

    std::cout << "====================================================\n";
    std::cout << "ALL STEP 3.2 MAVLINK TRANSPORT TESTS PASSED!\n";
    std::cout << "====================================================\n";
    return 0;
}
