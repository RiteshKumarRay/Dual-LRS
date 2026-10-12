#include <iostream>
#include <vector>
#include <cstdint>

#include "mavlink_handler.h"

uint32_t g_mock_micros = 0;
uint32_t g_mock_millis = 1000;

class CaptureStream : public Stream {
public:
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(const uint8_t* data, size_t length) override {
        bytes.insert(bytes.end(), data, data + length);
        return length;
    }
    std::vector<uint8_t> bytes;
};

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "FAIL: " << msg << "\n"; \
            return false; \
        } \
    } while (0)

static const uint8_t TEST_MAVLINK_CRCS[256] = {
    50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0,
    0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246,
    185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153,
    41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119,
    191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0,
    0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0,
    0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34,
    174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25,
    226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131,
    127, 0, 103, 154, 178, 200, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0,
    78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138,
    234, 240, 47, 189, 52, 174, 229, 85, 159, 186, 72, 0, 0, 0, 0, 92,
    36, 71, 98, 120, 0, 0, 0, 0, 134, 205, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 69, 101, 50, 202, 17, 162, 0, 0, 0, 0,
    0, 208, 207, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0,
    0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0,
};

static void mavlink_crc_accum(uint8_t data, uint16_t* crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static std::vector<uint8_t> mavlink_v1_valid(uint8_t msgid, uint8_t payload_len) {
    std::vector<uint8_t> frame((size_t)payload_len + 8, 0);
    frame[0] = 0xFE;
    frame[1] = payload_len;
    frame[2] = 1;
    frame[3] = 1;
    frame[4] = 1;
    frame[5] = msgid;
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < (size_t)(6 + payload_len); ++i) {
        mavlink_crc_accum(frame[i], &crc);
    }
    mavlink_crc_accum(TEST_MAVLINK_CRCS[msgid], &crc);
    frame[6 + payload_len] = (uint8_t)(crc & 0xFF);
    frame[7 + payload_len] = (uint8_t)((crc >> 8) & 0xFF);
    return frame;
}

static std::vector<uint8_t> mavlink_v1(uint8_t msgid, uint8_t payload_len) {
    return mavlink_v1_valid(msgid, payload_len);
}

static void update_crc(std::vector<uint8_t>& frame) {
    const size_t header = frame[0] == 0xFE ? 6 : 10;
    const uint8_t msgid = frame[0] == 0xFE ? frame[5] : frame[7];
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < header + frame[1]; ++i) mavlink_crc_accum(frame[i], &crc);
    mavlink_crc_accum(TEST_MAVLINK_CRCS[msgid], &crc);
    frame[header + frame[1]] = (uint8_t)crc;
    frame[header + frame[1] + 1] = (uint8_t)(crc >> 8);
}

static std::vector<uint8_t> param_value(uint16_t count, uint16_t index, bool v2 = false) {
    std::vector<uint8_t> frame = mavlink_v1(22, 25);
    if (v2) {
        frame.assign(37, 0);
        frame[0] = 0xFD;
        frame[1] = 25;
        frame[5] = frame[6] = 1;
        frame[7] = 22;
    }
    const size_t offset = v2 ? 10 : 6;
    frame[offset + 4] = (uint8_t)count;
    frame[offset + 5] = (uint8_t)(count >> 8);
    frame[offset + 6] = (uint8_t)index;
    frame[offset + 7] = (uint8_t)(index >> 8);
    update_crc(frame);
    return frame;
}

static std::vector<uint8_t> mavlink_v2(uint8_t msgid, uint8_t payload_len, bool signedFrame = false) {
    std::vector<uint8_t> frame((size_t)payload_len + 12 + (signedFrame ? 13 : 0), 0);
    frame[0] = 0xFD;
    frame[1] = payload_len;
    frame[2] = signedFrame ? 1 : 0;
    frame[5] = frame[6] = 1;
    frame[7] = msgid;
    update_crc(frame);
    return frame;
}

static void set_param_value_marker(std::vector<uint8_t>& frame, uint8_t marker) {
    const size_t offset = frame[0] == 0xFD ? 10 : 6;
    frame[offset] = marker;
    update_crc(frame);
}

static bool drain_packet(MavlinkHandler& handler, uint8_t expected_msgid) {
    uint8_t packet[296] = {};
    size_t len = handler.getOutboundPacket(packet, sizeof(packet));
    if (len == 0) return false;
    return packet[0] == 0xFE && packet[5] == expected_msgid;
}

bool test_mission_suppression_and_ack_resume() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> mission_count = mavlink_v1(44, 4);
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 20);
    std::vector<uint8_t> mission_ack = mavlink_v1(47, 3);

    handler.ingestBytes(mission_count.data(), mission_count.size());
    TEST_ASSERT(handler.isMissionActive(), "MISSION_COUNT must activate mission gate");
    TEST_ASSERT(handler.isTelemetrySuppressed(), "Mission gate must suppress telemetry");
    TEST_ASSERT(drain_packet(handler, 44), "MISSION_COUNT must remain deliverable");

    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    uint8_t packet[296] = {};
    TEST_ASSERT(handler.getOutboundPacket(packet, sizeof(packet)) == 0,
                "VFR_HUD must be suppressed during mission transfer");

    handler.ingestBytes(mission_ack.data(), mission_ack.size());
    TEST_ASSERT(!handler.isMissionActive(), "MISSION_ACK must end mission gate");
    TEST_ASSERT(!handler.isTelemetrySuppressed(), "Telemetry gate must clear after MISSION_ACK");
    TEST_ASSERT(drain_packet(handler, 47), "MISSION_ACK must remain deliverable");

    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(drain_packet(handler, 74), "Telemetry must resume after mission ACK");
    return true;
}

bool test_remote_mission_completion_and_queued_telemetry_drop() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 20);
    std::vector<uint8_t> mission_request = mavlink_v1(43, 2);
    std::vector<uint8_t> mission_ack = mavlink_v1(47, 3);

    // Queue telemetry first, then start a mission. The queued telemetry must
    // not leak after the mission packet is selected.
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    handler.ingestBytes(mission_request.data(), mission_request.size());
    TEST_ASSERT(handler.isMissionActive(), "Mission request must activate gate");
    TEST_ASSERT(drain_packet(handler, 43), "Mission request must be sent first");
    uint8_t packet[296] = {};
    TEST_ASSERT(handler.getOutboundPacket(packet, sizeof(packet)) == 0,
                "Queued telemetry must be discarded while mission gate is active");

    // The Air-side writeToLocal path carries the final GCS ACK to the FC and
    // must also clear the Air telemetry gate.
    handler.writeToLocal(mission_ack.data(), mission_ack.size());
    TEST_ASSERT(!handler.isMissionActive(), "Remote MISSION_ACK must end mission gate");
    TEST_ASSERT(local.bytes.size() == mission_ack.size(), "Remote ACK must reach FC UART");

    g_mock_millis += 1000;
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(drain_packet(handler, 74), "Telemetry must resume after remote ACK");
    return true;
}

bool test_mission_queue_does_not_overwrite_pending_handshake() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> mission_count = mavlink_v1(44, 4);
    std::vector<uint8_t> mission_ack = mavlink_v1(47, 3);

    // A second mission packet can arrive before the first gets a Ground slot.
    // Both packets must survive the cache handoff in arrival order.
    handler.ingestBytes(mission_count.data(), mission_count.size());
    handler.ingestBytes(mission_ack.data(), mission_ack.size());

    uint8_t packet[296] = {};
    size_t len = handler.getOutboundPacket(packet, sizeof(packet));
    TEST_ASSERT(len == mission_count.size() && packet[5] == 44,
                "pending MISSION_COUNT must not be replaced by a later MISSION_ACK");
    len = handler.getOutboundPacket(packet, sizeof(packet));
    TEST_ASSERT(len == mission_ack.size() && packet[5] == 47,
                "queued MISSION_ACK must be delivered after the earlier mission packet");
    return true;
}

bool test_new_mission_session_discards_abandoned_packets() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();
    auto abandoned = mavlink_v1(44, 4);
    auto request_list = mavlink_v1(43, 2);
    handler.ingestBytes(abandoned.data(), abandoned.size());
    g_mock_millis += 2501;
    handler.readFromLocal();
    TEST_ASSERT(!handler.isMissionActive(), "stalled mission exchange must expire");
    TEST_ASSERT(!handler.hasOutboundData(), "stalled mission exchange must discard its abandoned packet");

    handler.ingestBytes(request_list.data(), request_list.size());
    uint8_t packet[296] = {};
    size_t len = handler.getOutboundPacket(packet, sizeof(packet));
    TEST_ASSERT(len == request_list.size() && packet[5] == 43,
                "new mission session must not transmit an abandoned packet first");
    return true;
}

bool test_new_parameter_request_discards_stale_queued_values() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> request = mavlink_v1(21, 2);
    std::vector<uint8_t> stale = param_value(1, 0);
    std::vector<uint8_t> fresh = param_value(1, 0);
    set_param_value_marker(stale, 0x11);
    set_param_value_marker(fresh, 0xA5);

    handler.writeToLocal(request.data(), request.size());
    handler.ingestBytes(stale.data(), stale.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "initial parameter request must activate the gate");

    // Reconnect/request-list starts a new session while the old value is still
    // queued. The stale value must not complete or precede the new response.
    handler.writeToLocal(request.data(), request.size());
    handler.ingestBytes(fresh.data(), fresh.size());

    uint8_t packet[296] = {};
    TEST_ASSERT(handler.isParamDownloadActive(), "new parameter session must remain active until delivery");
    size_t len = handler.getOutboundPacket(packet, sizeof(packet));
    TEST_ASSERT(len == fresh.size() && packet[5] == 22 && packet[6] == 0xA5,
                "new request must deliver the fresh PARAM_VALUE, not stale queued data");
    TEST_ASSERT(!handler.isParamDownloadActive(), "fresh single-value parameter session must complete on delivery");
    return true;
}

bool test_begin_resets_incomplete_parsers_and_transfer_state() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();
    auto request = mavlink_v1(21, 2);
    auto mission = mavlink_v1(44, 4);
    auto heartbeat = mavlink_v1(0, 9);
    handler.writeToLocalAir(request.data(), request.size());
    handler.ingestBytes(mission.data(), mission.size());
    handler.ingestBytes(heartbeat.data(), 5);
    handler.writeToLocalGround(heartbeat.data(), 5);
    handler.begin();
    TEST_ASSERT(!handler.isTelemetrySuppressed(), "begin must clear abandoned mission/parameter gates");
    TEST_ASSERT(!handler.hasOutboundData(), "begin must clear abandoned queued packets");
    handler.ingestBytes(heartbeat.data(), heartbeat.size());
    TEST_ASSERT(drain_packet(handler, 0), "first local packet after begin must parse immediately");
    local.bytes.clear();
    handler.writeToLocalGround(heartbeat.data(), heartbeat.size());
    TEST_ASSERT(local.bytes == heartbeat, "first RF packet after begin must parse immediately");
    return true;
}

bool test_split_and_concatenated_uplink_session_observation() {
    for (bool v2 : {false, true}) {
        CaptureStream local;
        MavlinkHandler handler(local);
        handler.begin();
        auto request = v2 ? mavlink_v2(21, 2, true) : mavlink_v1(21, 2);
        auto mission = v2 ? mavlink_v2(44, 4) : mavlink_v1(44, 4);
        // An odd v1 sequence byte must not be interpreted as v2 signing flags.
        mission[2] = v2 ? 0 : 1;
        update_crc(mission);
        handler.writeToLocalAir(request.data(), 5);
        TEST_ASSERT(!handler.isParamDownloadActive(), "partial uplink must not start a session");
        std::vector<uint8_t> rest(request.begin() + 5, request.end());
        rest.insert(rest.end(), mission.begin(), mission.end());
        handler.writeToLocalAir(rest.data(), rest.size());
        TEST_ASSERT(handler.isParamDownloadActive(), "split valid PARAM_REQUEST_LIST must activate its session");
        TEST_ASSERT(handler.isMissionActive(), "second frame in one uplink buffer must activate its session");
        auto ack = v2 ? mavlink_v2(47, 3) : mavlink_v1(47, 3);
        handler.writeToLocalAir(ack.data(), ack.size());
        TEST_ASSERT(!handler.isMissionActive(), "ACK following v1 odd sequence must be observed");
        std::vector<uint8_t> expected = request;
        expected.insert(expected.end(), mission.begin(), mission.end());
        expected.insert(expected.end(), ack.begin(), ack.end());
        TEST_ASSERT(local.bytes == expected, "uplink observation must preserve every FC byte");
    }
    return true;
}

bool test_mission_fragments_survive_new_packet_ingress() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();
    auto item = mavlink_v2(73, 38, true);
    auto request = mavlink_v2(51, 5);
    handler.ingestBytes(item.data(), item.size());
    uint8_t bytes[296] = {};
    size_t len = handler.getOutboundPayload(bytes, 9);
    std::vector<uint8_t> received(bytes, bytes + len);
    handler.ingestBytes(request.data(), request.size());
    while (received.size() < item.size()) {
        len = handler.getOutboundPayload(bytes, 9);
        TEST_ASSERT(len > 0, "fragmented mission item must keep progressing");
        received.insert(received.end(), bytes, bytes + len);
    }
    TEST_ASSERT(received == item, "later mission ingress must not replace an in-progress mission tail");
    len = handler.getOutboundPacket(bytes, sizeof(bytes));
    TEST_ASSERT(len == request.size() && memcmp(bytes, request.data(), len) == 0,
                "next mission request must stay queued intact");
    return true;
}

bool test_wifi_ingress_recovers_after_partial_packet_timeout() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();
    auto request = mavlink_v1(21, 2);
    handler.ingestBytes(request.data(), 5);
    g_mock_millis += 51;
    handler.ingestBytes(request.data(), request.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "new Wi-Fi packet after a gap must not join a stale partial frame");
    TEST_ASSERT(drain_packet(handler, 21), "request after timeout must remain intact");
    return true;
}

bool test_parameter_suppression_and_completion() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> param_request = mavlink_v1(21, 2);
    update_crc(param_request);
    std::vector<uint8_t> param_value_0 = mavlink_v1(22, 25);
    std::vector<uint8_t> param_value_1 = mavlink_v1(22, 25);
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 20);

    // A request arrives from the GCS through the Air uplink path.
    handler.writeToLocal(param_request.data(), param_request.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "PARAM_REQUEST_LIST must activate gate");

    // PARAM_VALUE payload: param_count at offset 4, param_index at offset 6.
    param_value_0[10] = 2;
    param_value_0[12] = 0;
    update_crc(param_value_0);
    handler.ingestBytes(param_value_0.data(), param_value_0.size());
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(handler.isTelemetrySuppressed(), "Parameter gate must remain active before final value");

    uint8_t packet[296] = {};
    TEST_ASSERT(handler.getOutboundPacket(packet, sizeof(packet)) != 0 && packet[5] == 22,
                "PARAM_VALUE must remain deliverable");
    TEST_ASSERT(handler.getOutboundPacket(packet, sizeof(packet)) == 0,
                "VFR_HUD must be suppressed during parameter transfer");

    param_value_1[10] = 2;
    param_value_1[12] = 1;
    update_crc(param_value_1);
    handler.ingestBytes(param_value_1.data(), param_value_1.size());
    TEST_ASSERT(handler.isParamDownloadActive(),
                "Parameter gate must stay active until the final PARAM_VALUE leaves the queue");
    TEST_ASSERT(drain_packet(handler, 22), "Final PARAM_VALUE must be delivered");
    TEST_ASSERT(!handler.isParamDownloadActive(), "Final PARAM_VALUE transmission must end list gate");
    TEST_ASSERT(!handler.isTelemetrySuppressed(), "Telemetry gate must clear after final PARAM_VALUE transmission");

    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(drain_packet(handler, 74), "Telemetry must resume after parameter completion");
    return true;
}

// Test 1: Two small telemetry frames aggregate into a single transport payload (<= 49B)
bool test_normal_telemetry_aggregation_two_frames() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    // ATTITUDE: payload 10B -> 18B total frame
    std::vector<uint8_t> attitude = mavlink_v1(30, 10);
    // VFR_HUD: payload 16B -> 24B total frame
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 16);

    // Ensure combined length <= 49 bytes (18 + 24 = 42 bytes)
    TEST_ASSERT(attitude.size() + vfr_hud.size() <= 49, "Combined frames must be <= 49 bytes");

    g_mock_millis += 1000;
    handler.ingestBytes(attitude.data(), attitude.size());
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());

    uint8_t payload[296] = {};
    size_t payload_len = handler.getNormalTelemetryPayload(payload, sizeof(payload));

    TEST_ASSERT(payload_len == attitude.size() + vfr_hud.size(),
                "Combined payload length must equal sum of both complete frames");
    TEST_ASSERT(payload_len <= 49, "Combined payload must not exceed 49 bytes");

    // Frame 1 check (ATTITUDE: msgid 30)
    TEST_ASSERT(payload[0] == 0xFE && payload[1] == 10 && payload[5] == 30,
                "First frame must be complete ATTITUDE");
    TEST_ASSERT(memcmp(payload, attitude.data(), attitude.size()) == 0,
                "First frame bytes must match intact ATTITUDE");

    // Frame 2 check (VFR_HUD: msgid 74)
    size_t offset2 = attitude.size();
    TEST_ASSERT(payload[offset2] == 0xFE && payload[offset2 + 1] == 16 && payload[offset2 + 5] == 74,
                "Second frame must be complete VFR_HUD");
    TEST_ASSERT(memcmp(payload + offset2, vfr_hud.data(), vfr_hud.size()) == 0,
                "Second frame bytes must match intact VFR_HUD");

    // Only one transport payload is required; subsequent call returns 0
    TEST_ASSERT(handler.getNormalTelemetryPayload(payload, sizeof(payload)) == 0,
                "Subsequent call must return 0 once frames have been packed");

    return true;
}

// Test 2: Aggregation never exceeds TRANSPORT_MAX_SINGLE_BURST_PAYLOAD (49 bytes)
bool test_aggregation_never_exceeds_49_bytes() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    // 3 complete frames of 24 bytes each (payload 16)
    std::vector<uint8_t> frame1 = mavlink_v1(74, 16);
    std::vector<uint8_t> frame2 = mavlink_v1(74, 16);
    std::vector<uint8_t> frame3 = mavlink_v1(74, 16);

    // Push directly to queue: total 72 bytes. 24 + 24 = 48 (fits <= 49), + 24 = 72 (> 49).
    handler.pushBytes(frame1.data(), frame1.size());
    handler.pushBytes(frame2.data(), frame2.size());
    handler.pushBytes(frame3.data(), frame3.size());

    uint8_t payload[296] = {};
    size_t len1 = handler.getNormalTelemetryPayload(payload, sizeof(payload));

    TEST_ASSERT(len1 <= 49, "Payload 1 must never exceed 49 bytes");
    TEST_ASSERT(len1 == 48, "Payload 1 must pack exactly 2 frames (48 bytes)");
    TEST_ASSERT(memcmp(payload, frame1.data(), 24) == 0, "Frame 1 intact");
    TEST_ASSERT(memcmp(payload + 24, frame2.data(), 24) == 0, "Frame 2 intact");

    // Frame 3 must remain intact in queue for the next opportunity
    size_t len2 = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len2 == 24, "Next frame must remain intact for next Air slot");
    TEST_ASSERT(memcmp(payload, frame3.data(), 24) == 0, "Frame 3 intact");

    // Queue now empty; zero bytes dropped or leaked
    TEST_ASSERT(handler.getNormalTelemetryPayload(payload, sizeof(payload)) == 0,
                "Queue must be completely drained");

    return true;
}

// Test 3: MAVLink frame boundaries survive Ground parsing
static std::vector<std::vector<uint8_t>> g_groundEmissions;
static void testGroundOutputCallback(const uint8_t* buf, size_t len) {
    g_groundEmissions.push_back(std::vector<uint8_t>(buf, buf + len));
}

bool test_ground_output_parser_concatenated_frames() {
    CaptureStream local;
    MavlinkHandler groundHandler(local);
    groundHandler.begin();
    g_groundEmissions.clear();
    groundHandler.setOutputCallback(testGroundOutputCallback);

    std::vector<uint8_t> frame1 = mavlink_v1(30, 10); // 18 bytes
    std::vector<uint8_t> frame2 = mavlink_v1(74, 16); // 24 bytes

    // Concatenate into a single transport payload
    std::vector<uint8_t> transportPayload = frame1;
    transportPayload.insert(transportPayload.end(), frame2.begin(), frame2.end());
    TEST_ASSERT(transportPayload.size() == 42, "Concatenated payload must be 42 bytes");

    // Pass through Ground-side frame boundary parser
    groundHandler.writeToLocalGround(transportPayload.data(), transportPayload.size());

    // Confirm Ground stream emitted both frames separately via callback
    TEST_ASSERT(g_groundEmissions.size() == 2, "Ground parser must emit 2 separate frames");
    TEST_ASSERT(g_groundEmissions[0] == frame1, "First emitted frame must match Frame 1 intact");
    TEST_ASSERT(g_groundEmissions[1] == frame2, "Second emitted frame must match Frame 2 intact");

    // Confirm local serial received total bytes
    TEST_ASSERT(local.bytes.size() == 42, "Ground local serial must receive all 42 bytes");
    TEST_ASSERT(memcmp(local.bytes.data(), frame1.data(), 18) == 0, "Local byte stream frame 1 valid");
    TEST_ASSERT(memcmp(local.bytes.data() + 18, frame2.data(), 24) == 0, "Local byte stream frame 2 valid");

    return true;
}

// Test 4: Mission priority and telemetry suppression with aggregation
bool test_mission_priority_and_telemetry_suppression_with_aggregation() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> mission_count = mavlink_v1(44, 4); // 12 bytes
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 16);       // 24 bytes
    std::vector<uint8_t> mission_ack = mavlink_v1(47, 3);     // 11 bytes

    // Ingest mission packet
    handler.ingestBytes(mission_count.data(), mission_count.size());
    TEST_ASSERT(handler.isMissionActive(), "Mission gate active");
    TEST_ASSERT(handler.isTelemetrySuppressed(), "Telemetry suppressed");

    // Ingest telemetry while mission active
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());

    // Pull normal telemetry payload: must ONLY return mission packet alone!
    uint8_t payload[296] = {};
    size_t len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == mission_count.size(), "Mission packet must be delivered alone without aggregation");
    TEST_ASSERT(payload[0] == 0xFE && payload[5] == 44, "Payload must be MISSION_COUNT");

    // Next call must return 0 (background telemetry strictly suppressed)
    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == 0, "Background telemetry must NOT be sent or aggregated during mission");

    // Complete mission via ACK
    handler.ingestBytes(mission_ack.data(), mission_ack.size());
    TEST_ASSERT(!handler.isMissionActive(), "Mission gate cleared by ACK");
    TEST_ASSERT(!handler.isTelemetrySuppressed(), "Suppression cleared by ACK");

    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == mission_ack.size() && payload[5] == 47, "MISSION_ACK delivered");

    // Now normal telemetry can resume
    g_mock_millis += 1000;
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == vfr_hud.size() && payload[5] == 74, "Telemetry resumes after mission completion");

    return true;
}

// Test 5: Parameter priority and suppression with aggregation
bool test_parameter_priority_and_suppression_with_aggregation() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> param_request = mavlink_v1(21, 2);
    update_crc(param_request);
    std::vector<uint8_t> param_val0 = mavlink_v1(22, 25);
    param_val0[10] = 2; // param_count = 2
    param_val0[12] = 0; // param_index = 0
    std::vector<uint8_t> param_val1 = mavlink_v1(22, 25);
    param_val1[10] = 2; // param_count = 2
    param_val1[12] = 1; // param_index = 1
    update_crc(param_val0);
    update_crc(param_val1);
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 16);

    // GCS requests parameter list
    handler.writeToLocal(param_request.data(), param_request.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "Param download active");

    // Ingest PARAM_VALUE 0 and telemetry
    handler.ingestBytes(param_val0.data(), param_val0.size());
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(handler.isTelemetrySuppressed(), "Telemetry suppressed");

    // getNormalTelemetryPayload must deliver PARAM_VALUE alone (never aggregated with telemetry)
    uint8_t payload[296] = {};
    size_t len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == param_val0.size() && payload[5] == 22, "PARAM_VALUE delivered alone");

    // Telemetry must NOT be delivered
    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == 0, "Telemetry suppressed during parameter download");

    // Deliver final parameter
    handler.ingestBytes(param_val1.data(), param_val1.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "Param gate remains active while final value is queued");

    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == param_val1.size() && payload[5] == 22, "Final PARAM_VALUE delivered");
    TEST_ASSERT(!handler.isParamDownloadActive(), "Param download completed after final value transmission");
    TEST_ASSERT(!handler.isTelemetrySuppressed(), "Suppression cleared after final value transmission");

    // Telemetry resumes
    g_mock_millis += 1000;
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    len = handler.getNormalTelemetryPayload(payload, sizeof(payload));
    TEST_ASSERT(len == vfr_hud.size() && payload[5] == 74, "Telemetry resumes after parameter completion");

    return true;
}

bool test_parameter_completion_requires_distinct_indices() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> request = mavlink_v1(21, 2);
    std::vector<uint8_t> first = param_value(3, 0);
    std::vector<uint8_t> duplicate = param_value(3, 2);
    std::vector<uint8_t> last = param_value(3, 2);
    std::vector<uint8_t> vfr_hud = mavlink_v1(74, 16);

    handler.writeToLocal(request.data(), request.size());
    handler.ingestBytes(first.data(), first.size());
    handler.ingestBytes(duplicate.data(), duplicate.size());
    handler.ingestBytes(last.data(), last.size());
    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(handler.isParamDownloadActive(),
                "Out-of-order values must not complete a list with a missing index");

    uint8_t packet[296] = {};
    while (handler.getOutboundPacket(packet, sizeof(packet)) != 0) {
        // Drain the values already received; telemetry must remain gated.
    }
    TEST_ASSERT(handler.isParamDownloadActive(), "Missing parameter must keep telemetry gated");
    TEST_ASSERT(handler.getOutboundPacket(packet, sizeof(packet)) == 0,
                "Routine telemetry must stay suppressed while a parameter index is missing");

    std::vector<uint8_t> missing = param_value(3, 1);
    handler.ingestBytes(missing.data(), missing.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "Final received value remains queued until transmitted");
    TEST_ASSERT(drain_packet(handler, 22), "Missing parameter must remain deliverable");
    TEST_ASSERT(!handler.isParamDownloadActive(), "Distinct complete parameter set must release gate");

    handler.ingestBytes(vfr_hud.data(), vfr_hud.size());
    TEST_ASSERT(drain_packet(handler, 74), "Routine telemetry must resume after all parameters transmit");
    return true;
}

bool test_stalled_parameter_list_releases_gate_after_timeout() {
    CaptureStream local;
    MavlinkHandler handler(local);
    handler.begin();

    std::vector<uint8_t> request = mavlink_v1(21, 2);
    update_crc(request);
    handler.writeToLocal(request.data(), request.size());
    TEST_ASSERT(handler.isParamDownloadActive(), "parameter list must activate gate");

    // A stalled FC must not leave routine telemetry disabled forever. The
    // timeout is intentionally much longer than the normal reliable retry
    // window and only applies after the parameter queue has drained.
    g_mock_millis += 10001;
    handler.readFromLocal();
    TEST_ASSERT(!handler.isParamDownloadActive(),
                "stalled parameter list must release suppression after timeout");
    TEST_ASSERT(!handler.isTelemetrySuppressed(),
                "routine telemetry must be able to recover after an aborted list");
    return true;
}

int main() {
    std::cout << "=== Running MAVLink telemetry gating tests ===\n";
    if (!test_mission_suppression_and_ack_resume() ||
        !test_remote_mission_completion_and_queued_telemetry_drop() ||
        !test_mission_queue_does_not_overwrite_pending_handshake() ||
        !test_new_mission_session_discards_abandoned_packets() ||
        !test_new_parameter_request_discards_stale_queued_values() ||
        !test_begin_resets_incomplete_parsers_and_transfer_state() ||
        !test_split_and_concatenated_uplink_session_observation() ||
        !test_mission_fragments_survive_new_packet_ingress() ||
        !test_wifi_ingress_recovers_after_partial_packet_timeout() ||
        !test_parameter_suppression_and_completion() ||
        !test_normal_telemetry_aggregation_two_frames() ||
        !test_aggregation_never_exceeds_49_bytes() ||
        !test_ground_output_parser_concatenated_frames() ||
        !test_mission_priority_and_telemetry_suppression_with_aggregation() ||
        !test_parameter_priority_and_suppression_with_aggregation() ||
        !test_parameter_completion_requires_distinct_indices() ||
        !test_stalled_parameter_list_releases_gate_after_timeout()) {
        return 1;
    }
    std::cout << "=== ALL MAVLink telemetry gating and aggregation tests passed ===\n";
    return 0;
}
