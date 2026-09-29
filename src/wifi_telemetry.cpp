#if defined(ESP32)

#include "wifi_telemetry.h"
#include "esp_wifi.h"

WifiTelemetry::WifiTelemetry()
    : _isApMode(false),
      _hasClient(false),
      _clientPort(WIFI_UDP_PORT),
      _lastClientPacketMs(0),
      _lastStaReconnectCheckMs(0) {}

void WifiTelemetry::begin() {
    Serial.println("\n[WIFI] Initializing Dual-LRS Wireless Telemetry...");
    Serial.flush();

    // Try connecting to phone hotspot first (STA mode)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    Serial.printf("[WIFI] Searching for Hotspot: \"%s\" (timeout: %d ms)...\n", 
                  WIFI_STA_SSID, WIFI_CONNECT_TIMEOUT_MS);
    Serial.flush();

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs < WIFI_CONNECT_TIMEOUT_MS)) {
        delay(100);
        Serial.print(".");
        Serial.flush();
    }
    Serial.println();
    Serial.flush();

    if (WiFi.status() == WL_CONNECTED) {
        _isApMode = false;
        WiFi.setSleep(false);
        Serial.printf("[WIFI] CONNECTED to Hotspot! IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WIFI] Subnet Mask: %s | Gateway: %s\n", 
                      WiFi.subnetMask().toString().c_str(), WiFi.gatewayIP().toString().c_str());
        // Calculate subnet broadcast IP (e.g. 192.168.43.255)
        _broadcastIP = IPAddress(WiFi.localIP() | ~WiFi.subnetMask());
        Serial.printf("[WIFI] Telemetry Broadcast IP: %s:%u\n", 
                      _broadcastIP.toString().c_str(), WIFI_UDP_PORT);
    } else {
        // Hotspot unreachable: fallback to standalone Access Point
        Serial.println("[WIFI] Hotspot not found. Switching to AP Mode...");
        Serial.flush();
        WiFi.disconnect(true);
        delay(100);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
        WiFi.setSleep(false);
        _isApMode = true;
        _broadcastIP = IPAddress(192, 168, 4, 255);
        Serial.printf("[WIFI] Standalone AP Started: SSID=\"%s\" | IP=%s\n",
                      WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    }

    _udp.begin(WIFI_UDP_PORT);
    Serial.printf("[WIFI] MAVLink UDP listening on port %u (Ready for QGC / Mission Planner)\n\n", WIFI_UDP_PORT);
    Serial.flush();
}

void WifiTelemetry::update(MavlinkHandler& telemHandler) {
    int packetSize = _udp.parsePacket();
    if (packetSize > 0) {
        IPAddress remote = _udp.remoteIP();
        uint16_t port = _udp.remotePort();

        if (!_hasClient || _clientIP != remote || _clientPort != port) {
            _clientIP = remote;
            _clientPort = port;
            _hasClient = true;
            Serial.printf("[WIFI] GCS Client connected from %s:%u\n", 
                          _clientIP.toString().c_str(), _clientPort);
        }
        _lastClientPacketMs = millis();

        while (packetSize > 0) {
            int toRead = packetSize > (int)sizeof(_rxBuffer) ? (int)sizeof(_rxBuffer) : packetSize;
            int bytesRead = _udp.read(_rxBuffer, toRead);
            if (bytesRead <= 0) break;
            telemHandler.ingestBytes(_rxBuffer, (size_t)bytesRead);
            packetSize -= bytesRead;
        }
    }

    // Reset to broadcast mode if client has been silent for > 15s
    if (_hasClient && (millis() - _lastClientPacketMs > 15000)) {
        _hasClient = false;
        Serial.println("[WIFI] GCS Client inactive. Returned to broadcast mode.");
    }

    // Non-blocking auto-reconnect if hotspot dropped
    if (!_isApMode && WiFi.status() != WL_CONNECTED) {
        uint32_t now = millis();
        if (now - _lastStaReconnectCheckMs > 5000) {
            _lastStaReconnectCheckMs = now;
            WiFi.reconnect();
        }
    }
}

void WifiTelemetry::sendMavlinkPacket(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) return;

    if (_hasClient) {
        _udp.beginPacket(_clientIP, _clientPort);
    } else {
        _udp.beginPacket(_broadcastIP, WIFI_UDP_PORT);
    }
    _udp.write(data, len);
    _udp.endPacket();
}

bool WifiTelemetry::isConnected() const {
    if (_isApMode) {
        return (WiFi.softAPgetStationNum() > 0);
    }
    return (WiFi.status() == WL_CONNECTED);
}

IPAddress WifiTelemetry::getLocalIP() const {
    return _isApMode ? WiFi.softAPIP() : WiFi.localIP();
}

#endif // ESP32
