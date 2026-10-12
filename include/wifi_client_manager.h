#pragma once
#include <stdint.h>
#include <stddef.h>

// =============================================================================
// WifiClientManager
// Strict One-Client Ownership State Machine for MAVLink UDP Bridge.
//
// Rules:
// 1. The first valid MAVLink client IP becomes the active client.
// 2. Packets from the same IP are accepted even when the UDP source port
//    changes. GCS applications commonly use an ephemeral source port and may
//    select a new one after reconnecting or reopening a link.
// 3. All packets from differing IPs are REJECTED while the active client is alive.
// 4. Active client ownership expires after a bounded inactivity timeout (default 15s).
// 5. Once expired, another GCS client may take ownership.
// 6. Never logs or touches Wi-Fi credentials.
// =============================================================================

class WifiClientManager {
public:
    static constexpr uint32_t DEFAULT_INACTIVITY_TIMEOUT_MS = 15000; // 15 seconds

    explicit WifiClientManager(uint32_t timeoutMs = DEFAULT_INACTIVITY_TIMEOUT_MS)
        : _timeoutMs(timeoutMs),
          _hasClient(false),
          _clientIp(0),
          _clientPort(0),
          _lastPacketMs(0),
          _totalPacketsAccepted(0),
          _totalPacketsRejected(0) {}

    // Evaluate an incoming packet from senderIp and senderPort at timestamp nowMs.
    // Returns true if packet is accepted (and client lock is refreshed/acquired).
    // The source port is retained as the current outbound destination. It is
    // deliberately not part of ownership so an ephemeral-port change does not
    // strand mission/parameter traffic.
    bool processIncomingPacket(uint32_t senderIp, uint16_t senderPort, uint32_t nowMs) {
        if (senderIp == 0 || senderPort == 0) {
            _totalPacketsRejected++;
            return false;
        }

        if (_hasClient) {
            if (nowMs - _lastPacketMs <= _timeoutMs) {
                // Active client is still within timeout window
                if (_clientIp == senderIp) {
                    // Route replies to the most recently used source port. A
                    // GCS UDP socket is allowed to migrate ports while keeping
                    // the same host/IP (for example after a reconnect).
                    _clientPort = senderPort;
                    _lastPacketMs = nowMs;
                    _totalPacketsAccepted++;
                    return true;
                }
                // Different host attempted to send while active client is locked -> REJECT
                _totalPacketsRejected++;
                return false;
            } else {
                // Previous client timed out due to inactivity
                _hasClient = false;
            }
        }

        // Lock in the new active client
        _clientIp = senderIp;
        _clientPort = senderPort;
        _hasClient = true;
        _lastPacketMs = nowMs;
        _totalPacketsAccepted++;
        return true;
    }

    // Check if an active client is currently locked and alive.
    bool hasActiveClient(uint32_t nowMs) const {
        if (!_hasClient) return false;
        return (nowMs - _lastPacketMs <= _timeoutMs);
    }

    // Periodic check to cleanly expire client state.
    void checkTimeout(uint32_t nowMs) {
        if (_hasClient && (nowMs - _lastPacketMs > _timeoutMs)) {
            _hasClient = false;
            _clientIp = 0;
            _clientPort = 0;
        }
    }

    uint32_t getClientIp() const { return _clientIp; }
    uint16_t getClientPort() const { return _clientPort; }
    uint32_t getLastPacketMs() const { return _lastPacketMs; }
    uint32_t getTimeoutMs() const { return _timeoutMs; }
    uint32_t getTotalPacketsAccepted() const { return _totalPacketsAccepted; }
    uint32_t getTotalPacketsRejected() const { return _totalPacketsRejected; }

    void reset() {
        _hasClient = false;
        _clientIp = 0;
        _clientPort = 0;
        _lastPacketMs = 0;
    }

private:
    uint32_t _timeoutMs;
    bool _hasClient;
    uint32_t _clientIp;
    uint16_t _clientPort;
    uint32_t _lastPacketMs;
    uint32_t _totalPacketsAccepted;
    uint32_t _totalPacketsRejected;
};
