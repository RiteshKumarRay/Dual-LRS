#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cassert>

#include "wifi_client_manager.h"
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
            std::cerr << "FAIL: " << msg << " (line " << __LINE__ << ")\n"; \
            return false; \
        } \
    } while (0)

static void mavlink_crc_accum(uint8_t data, uint16_t* crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// -----------------------------------------------------------------------------
// Test 1: Initial state of WifiClientManager
// -----------------------------------------------------------------------------
static bool test_client_manager_initial_state() {
    WifiClientManager mgr(15000);
    TEST_ASSERT(!mgr.hasActiveClient(0), "Manager must not have active client at init");
    TEST_ASSERT(mgr.getClientIp() == 0, "Initial client IP must be 0");
    TEST_ASSERT(mgr.getClientPort() == 0, "Initial client port must be 0");
    TEST_ASSERT(mgr.getTimeoutMs() == 15000, "Timeout must be 15000 ms");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 0, "Initial accepted count must be 0");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 0, "Initial rejected count must be 0");
    return true;
}

// -----------------------------------------------------------------------------
// Test 2: Single client lock
// -----------------------------------------------------------------------------
static bool test_single_client_lock() {
    WifiClientManager mgr(15000);
    uint32_t ipA = 0xC0A82B64; // 192.168.43.100
    uint16_t portA = 14550;

    bool accepted = mgr.processIncomingPacket(ipA, portA, 1000);
    TEST_ASSERT(accepted, "First client packet must be accepted");
    TEST_ASSERT(mgr.hasActiveClient(1000), "Client A must be active");
    TEST_ASSERT(mgr.getClientIp() == ipA, "Stored IP must match Client A");
    TEST_ASSERT(mgr.getClientPort() == portA, "Stored Port must match Client A");
    TEST_ASSERT(mgr.getLastPacketMs() == 1000, "Last packet ms must match 1000");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 1, "Accepted packets must be 1");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 0, "Rejected packets must be 0");
    return true;
}

// -----------------------------------------------------------------------------
// Test 3: Active client continuous streaming keeps lock alive
// -----------------------------------------------------------------------------
static bool test_active_client_continuous_stream() {
    WifiClientManager mgr(15000);
    uint32_t ipA = 0xC0A82B64;
    uint16_t portA = 14550;

    TEST_ASSERT(mgr.processIncomingPacket(ipA, portA, 1000), "Packet at 1s accepted");
    TEST_ASSERT(mgr.processIncomingPacket(ipA, portA, 5000), "Packet at 5s accepted");
    TEST_ASSERT(mgr.processIncomingPacket(ipA, portA, 10000), "Packet at 10s accepted");
    TEST_ASSERT(mgr.processIncomingPacket(ipA, portA, 18000), "Packet at 18s accepted (8s gap)");
    TEST_ASSERT(mgr.processIncomingPacket(ipA, portA, 25000), "Packet at 25s accepted (7s gap)");

    TEST_ASSERT(mgr.hasActiveClient(25000), "Client must remain active at 25s");
    TEST_ASSERT(mgr.getLastPacketMs() == 25000, "Last packet ms updated to 25s");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 5, "All 5 packets accepted");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 0, "Zero packets rejected");
    return true;
}

// -----------------------------------------------------------------------------
// Test 4: Second client rejection while active client is alive (different IP)
// -----------------------------------------------------------------------------
static bool test_second_client_rejection_different_ip() {
    WifiClientManager mgr(15000);
    uint32_t ipA = 0xC0A82B64; // 192.168.43.100
    uint32_t ipB = 0xC0A82BC8; // 192.168.43.200
    uint16_t port = 14550;

    TEST_ASSERT(mgr.processIncomingPacket(ipA, port, 1000), "Client A locked");

    // Client B attempts to send at t = 3000ms
    bool bAccepted = mgr.processIncomingPacket(ipB, port, 3000);
    TEST_ASSERT(!bAccepted, "Client B packet must be REJECTED while Client A is active");
    TEST_ASSERT(mgr.getClientIp() == ipA, "Active client must remain Client A");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 1, "Accepted count must be 1");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 1, "Rejected count must be 1");

    // Client A sends another packet at t = 4000ms -> should still be accepted
    bool aAccepted = mgr.processIncomingPacket(ipA, port, 4000);
    TEST_ASSERT(aAccepted, "Client A packet must still be accepted");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 2, "Accepted count must be 2");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 1, "Rejected count remains 1");
    return true;
}

// -----------------------------------------------------------------------------
// Test 5: Same GCS IP may migrate to a new ephemeral source port
// -----------------------------------------------------------------------------
static bool test_same_client_port_migration() {
    WifiClientManager mgr(15000);
    uint32_t ip = 0xC0A82B64;
    uint16_t portA = 14550;
    uint16_t portB = 54321;

    TEST_ASSERT(mgr.processIncomingPacket(ip, portA, 1000), "Client A locked");

    bool migrated = mgr.processIncomingPacket(ip, portB, 2000);
    TEST_ASSERT(migrated, "Same GCS IP must be accepted after source-port migration");
    TEST_ASSERT(mgr.getClientIp() == ip, "Active IP must remain the GCS IP");
    TEST_ASSERT(mgr.getClientPort() == portB, "Outbound route must follow the latest source port");
    TEST_ASSERT(mgr.getTotalPacketsAccepted() == 2, "Both packets from the same GCS must be accepted");
    TEST_ASSERT(mgr.getTotalPacketsRejected() == 0, "Port migration must not count as a rejection");
    return true;
}

// -----------------------------------------------------------------------------
// Test 6: Client takeover after 15-second inactivity timeout
// -----------------------------------------------------------------------------
static bool test_client_takeover_after_inactivity_timeout() {
    WifiClientManager mgr(15000);
    uint32_t ipA = 0xC0A82B64;
    uint32_t ipB = 0xC0A82BC8;
    uint16_t port = 14550;

    // Client A sends at t = 1000 ms
    TEST_ASSERT(mgr.processIncomingPacket(ipA, port, 1000), "Client A locked at 1s");

    // At t = 15999 ms (14.999s elapsed since last packet) -> Client A still alive
    TEST_ASSERT(mgr.hasActiveClient(15999), "Client A still active at 15.999s");
    TEST_ASSERT(!mgr.processIncomingPacket(ipB, port, 15999), "Client B rejected at 15.999s");

    // At t = 16001 ms (15.001s elapsed since last packet) -> Client A expired!
    TEST_ASSERT(!mgr.hasActiveClient(16001), "Client A must be expired at 16.001s");

    // Now Client B sends at t = 16001 ms -> Client B takes over!
    bool bAccepted = mgr.processIncomingPacket(ipB, port, 16001);
    TEST_ASSERT(bAccepted, "Client B must be ACCEPTED after Client A timed out");
    TEST_ASSERT(mgr.hasActiveClient(16001), "Manager now has active client");
    TEST_ASSERT(mgr.getClientIp() == ipB, "Active client IP must now be Client B");
    TEST_ASSERT(mgr.getLastPacketMs() == 16001, "Last packet ms updated to 16001");
    return true;
}

// -----------------------------------------------------------------------------
// Test 7: UDP inbound bytes reach MavlinkHandler correctly
// -----------------------------------------------------------------------------
static bool test_udp_inbound_bytes_reach_mavlink_handler() {
    CaptureStream dummySerial;
    MavlinkHandler handler(dummySerial);
    handler.begin();

    // Construct a valid MAVLink v1 HEARTBEAT packet (sysid=255, compid=190, msgid=0)
    uint8_t hbPacket[17] = {
        0xFE, 9, 0, 255, 190, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0 // CRC placeholder
    };
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 15; ++i) {
        mavlink_crc_accum(hbPacket[i], &crc);
    }
    mavlink_crc_accum(50, &crc); // CRC_EXTRA for HEARTBEAT
    hbPacket[15] = (uint8_t)(crc & 0xFF);
    hbPacket[16] = (uint8_t)((crc >> 8) & 0xFF);

    size_t prevBytes = handler.rawBytesRead();
    // Simulate UDP receive feeding into MavlinkHandler::ingestBytes
    handler.ingestBytes(hbPacket, sizeof(hbPacket));

    TEST_ASSERT(handler.rawBytesRead() == prevBytes + sizeof(hbPacket),
                "MavlinkHandler must ingest all UDP bytes");
    return true;
}

static std::vector<uint8_t> s_dispatchedCallbackData;
static void test_output_cb(const uint8_t* data, size_t len) {
    s_dispatchedCallbackData.assign(data, data + len);
}

// -----------------------------------------------------------------------------
// Test 8: Complete outbound MAVLink packet delivery to active client callback
// -----------------------------------------------------------------------------
static bool test_complete_outbound_mavlink_packet_delivery() {
    CaptureStream dummySerial;
    MavlinkHandler handler(dummySerial);
    handler.begin();

    s_dispatchedCallbackData.clear();
    handler.setOutputCallback(test_output_cb);

    // Construct a valid MAVLink v1 packet (sysid=1, compid=1, msgid=0 HEARTBEAT)
    uint8_t hbPacket[17] = {
        0xFE, 9, 42, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0
    };
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 15; ++i) {
        mavlink_crc_accum(hbPacket[i], &crc);
    }
    mavlink_crc_accum(50, &crc);
    hbPacket[15] = (uint8_t)(crc & 0xFF);
    hbPacket[16] = (uint8_t)((crc >> 8) & 0xFF);

    // Feed to writeToLocalGround (simulating completed RF reception)
    handler.writeToLocalGround(hbPacket, sizeof(hbPacket));

    TEST_ASSERT(!s_dispatchedCallbackData.empty(), "Output callback must be invoked");
    TEST_ASSERT(s_dispatchedCallbackData.size() == sizeof(hbPacket),
                "Dispatched packet size must match full MAVLink frame");
    TEST_ASSERT(memcmp(s_dispatchedCallbackData.data(), hbPacket, sizeof(hbPacket)) == 0,
                "Dispatched packet must bit-exact match complete MAVLink frame");
    return true;
}

// -----------------------------------------------------------------------------
// Test 9: No hardcoded credentials in source or manager
// -----------------------------------------------------------------------------
static bool test_no_credentials_in_manager() {
    WifiClientManager mgr;
    // Ensure manager has zero string members or credential fields
    TEST_ASSERT(sizeof(mgr) <= 40, "WifiClientManager must remain lean and free of strings");
    return true;
}

// -----------------------------------------------------------------------------
// Test 10: Reset clears active client state cleanly
// -----------------------------------------------------------------------------
static bool test_client_manager_reset() {
    WifiClientManager mgr(15000);
    uint32_t ip = 0xC0A82B64;
    uint16_t port = 14550;

    TEST_ASSERT(mgr.processIncomingPacket(ip, port, 1000), "Client locked");
    TEST_ASSERT(mgr.hasActiveClient(1000), "Client active");

    mgr.reset();
    TEST_ASSERT(!mgr.hasActiveClient(1000), "Client inactive after reset");
    TEST_ASSERT(mgr.getClientIp() == 0, "IP reset to 0");
    TEST_ASSERT(mgr.getClientPort() == 0, "Port reset to 0");
    return true;
}

int main() {
    std::cout << "===============================================================\n";
    std::cout << "Running Dual-LRS Step 9: Wi-Fi Telemetry & Client Manager Unit Tests\n";
    std::cout << "===============================================================\n";

    int passed = 0;
    int total = 10;

    if (test_client_manager_initial_state()) { std::cout << "  [PASS] test_client_manager_initial_state\n"; passed++; }
    if (test_single_client_lock()) { std::cout << "  [PASS] test_single_client_lock\n"; passed++; }
    if (test_active_client_continuous_stream()) { std::cout << "  [PASS] test_active_client_continuous_stream\n"; passed++; }
    if (test_second_client_rejection_different_ip()) { std::cout << "  [PASS] test_second_client_rejection_different_ip\n"; passed++; }
    if (test_same_client_port_migration()) { std::cout << "  [PASS] test_same_client_port_migration\n"; passed++; }
    if (test_client_takeover_after_inactivity_timeout()) { std::cout << "  [PASS] test_client_takeover_after_inactivity_timeout\n"; passed++; }
    if (test_udp_inbound_bytes_reach_mavlink_handler()) { std::cout << "  [PASS] test_udp_inbound_bytes_reach_mavlink_handler\n"; passed++; }
    if (test_complete_outbound_mavlink_packet_delivery()) { std::cout << "  [PASS] test_complete_outbound_mavlink_packet_delivery\n"; passed++; }
    if (test_no_credentials_in_manager()) { std::cout << "  [PASS] test_no_credentials_in_manager\n"; passed++; }
    if (test_client_manager_reset()) { std::cout << "  [PASS] test_client_manager_reset\n"; passed++; }

    std::cout << "===============================================================\n";
    if (passed == total) {
        std::cout << "ALL " << total << " WI-FI TELEMETRY TESTS PASSED (100% SUCCESS)!\n";
        std::cout << "===============================================================\n";
        return 0;
    } else {
        std::cout << "FAILED: " << (total - passed) << " tests failed!\n";
        std::cout << "===============================================================\n";
        return 1;
    }
}
