#pragma once
#if defined(ESP32)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "mavlink_handler.h"

// =============================================================================
// WifiTelemetry
// Wireless MAVLink UDP bridge for Ground station (ESP32)
//
// Features:
// - Auto-Fallback: Tries connecting to Phone Hotspot (STA) first so the GCS laptop
//   retains simultaneous cellular/Wi-Fi internet access (for live satellite maps).
//   If hotspot is unreachable after timeout, automatically boots into standalone AP mode.
// - Zero TDM Jitter: Wi-Fi stack runs on Core 0 (PRO_CPU). Non-blocking UDP parsing
//   guarantees Core 1 radio timing is never blocked or delayed.
// - Dual-stream: Outbound telemetry is sent via both USB Serial and UDP port 14550.
// - Client Tracking: Unicasts to active GCS client; broadcasts if no client connected.
// =============================================================================

class WifiTelemetry {
public:
    WifiTelemetry();

    // Initialize Wi-Fi (tries STA mode with phone hotspot, falls back to AP mode)
    void begin();

    // Non-blocking poll for incoming UDP packets from GCS (QGroundControl / Mission Planner)
    void update(MavlinkHandler& telemHandler);

    // Send complete MAVLink packet out over UDP
    void sendMavlinkPacket(const uint8_t* data, size_t len);

    bool isConnected() const;
    bool isApMode() const { return _isApMode; }
    IPAddress getLocalIP() const;

private:
    WiFiUDP _udp;
    bool _isApMode = false;
    bool _hasClient = false;
    IPAddress _clientIP;
    uint16_t _clientPort = WIFI_UDP_PORT;
    uint32_t _lastClientPacketMs = 0;
    IPAddress _broadcastIP;
    uint8_t _rxBuffer[512];
    uint32_t _lastStaReconnectCheckMs = 0;
};

#endif // ESP32
