#pragma once
#if defined(ESP32)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "config.h"
#include "mavlink_handler.h"
#include "wifi_client_manager.h"

// =============================================================================
// WifiTelemetry
// Wireless MAVLink UDP bridge for Ground station (ESP32)
//
// Features:
// - Station Mode with Phone Hotspot: Connects to the locally configured hotspot.
// - Standalone AP Fallback: Automatically falls back to softAP if hotspot is unreachable or unconfigured.
// - Strict One-Client Ownership: Locks to the first GCS client IP, rejects
//   conflicting devices, and permits that client to migrate UDP source ports.
// - Bounded Non-Blocking UDP: Limits packet processing to prevent loop starvation.
// - No connection waits in setup/loop; RX work is bounded between datagrams.
// - Dual-stream: Outbound telemetry is sent via both USB Serial and UDP port 14550.
// - mDNS: Broadcasts hostname "dual-lrs-ground.local".
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
    bool hasActiveClient() const;
    IPAddress getClientIP() const;
    uint16_t getClientPort() const;
    const WifiClientManager& getClientManager() const { return _clientMgr; }

private:
    void startAccessPoint();
    void startMdns();

    WiFiUDP _udp;
    WifiClientManager _clientMgr;
    bool _isApMode = false;
    bool _hasStaCredentials = false;
    bool _staEverConnected = false;
    bool _mdnsStarted = false;
    bool _lastClientLogged = false;
    IPAddress _lastLoggedIp;
    uint16_t _lastLoggedPort = 0;
    IPAddress _broadcastIP;
    uint8_t _rxBuffer[WIFI_UDP_MAX_PAYLOAD_BYTES];
    uint32_t _staConnectStartedMs = 0;
    uint32_t _lastStaReconnectCheckMs = 0;
};

#endif // ESP32
