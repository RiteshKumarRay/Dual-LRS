#if defined(ESP32)

#include "wifi_telemetry.h"
#include <cstring>

WifiTelemetry::WifiTelemetry()
    : _clientMgr(WIFI_CLIENT_INACTIVITY_TIMEOUT_MS),
      _isApMode(false),
      _hasStaCredentials(false),
      _staEverConnected(false),
      _mdnsStarted(false),
      _lastClientLogged(false),
      _lastLoggedPort(0),
      _staConnectStartedMs(0),
      _lastStaReconnectCheckMs(0) {}

void WifiTelemetry::begin() {
    Serial.println("\n[WIFI] Initializing Dual-LRS Wireless Telemetry...");

    // Keep the Wi-Fi stack from adding power-save wake latency to MAVLink
    // request/response traffic. Configure this before association so it also
    // applies to the first connection attempt.
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);

    _hasStaCredentials = (strlen(WIFI_STA_SSID) > 0 &&
                          strlen(WIFI_STA_PASS) > 0 &&
                          strcmp(WIFI_STA_SSID, "DISABLED_PLACEHOLDER_SSID") != 0 &&
                          strcmp(WIFI_STA_PASS, "DISABLED_PLACEHOLDER_PASSWORD") != 0 &&
                          strcmp(WIFI_STA_PASS, "YOUR_HOTSPOT_PASSWORD_HERE") != 0);

    if (_hasStaCredentials) {
        Serial.printf("[WIFI] Connecting to Phone Hotspot: \"%s\"...\n", WIFI_STA_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.setTxPower(WIFI_TX_POWER);
        WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
        _staConnectStartedMs = millis();
        // Do not wait here. begin() runs from setup immediately before the
        // RF loop starts; a connection timeout must never stall that loop.
    } else {
        Serial.println("[WIFI] NOTICE: Station credentials are not configured; using AP mode.");
        startAccessPoint();
    }

    if (_hasStaCredentials && WiFi.status() == WL_CONNECTED) {
        _staEverConnected = true;
        _broadcastIP = IPAddress(WiFi.localIP() | ~WiFi.subnetMask());
        Serial.printf("[WIFI] CONNECTED to Hotspot! Station IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WIFI] Subnet Mask: %s | Gateway: %s\n",
                      WiFi.subnetMask().toString().c_str(), WiFi.gatewayIP().toString().c_str());
    }

    _udp.begin(WIFI_UDP_PORT);
    startMdns();
    Serial.printf("[WIFI] MAVLink UDP listening on local port %u (one GCS IP, ephemeral source ports supported)\n\n",
                  WIFI_UDP_PORT);
}

void WifiTelemetry::startAccessPoint() {
    if (_isApMode) return;

    // This path is used once during startup fallback, never from the timing-
    // critical RF transmit section. Avoid the old blocking delay/disconnect
    // sequence; the ESP32 Wi-Fi task handles the mode transition itself.
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_TX_POWER);
    _isApMode = true;
    _broadcastIP = IPAddress(192, 168, 4, 255);
    Serial.printf("[WIFI] Standalone AP Ready! SSID: \"%s\" | IP: %s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    startMdns();
}

void WifiTelemetry::startMdns() {
    if (_mdnsStarted || (!_isApMode && WiFi.status() != WL_CONNECTED)) return;
    if (MDNS.begin("dual-lrs-ground")) {
        MDNS.addService("mavlink", "udp", WIFI_UDP_PORT);
        _mdnsStarted = true;
        Serial.println("[WIFI] mDNS responder active: dual-lrs-ground.local");
    }
}

void WifiTelemetry::update(MavlinkHandler& telemHandler) {
    uint32_t now = millis();

    // Association and reconnect handling is deliberately bounded to one
    // status check per loop. No delay or wait-for-connection loop is allowed
    // here because this function runs beside the RF/TDM scheduler.
    if (!_isApMode) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!_staEverConnected) {
                _staEverConnected = true;
                Serial.printf("[WIFI] CONNECTED to Hotspot! Station IP: %s\n",
                              WiFi.localIP().toString().c_str());
            }
            _broadcastIP = IPAddress(WiFi.localIP() | ~WiFi.subnetMask());
            startMdns();
        } else if (!_staEverConnected &&
                   (now - _staConnectStartedMs >= WIFI_CONNECT_TIMEOUT_MS)) {
            Serial.println("[WIFI] Hotspot not found within timeout. Starting Standalone AP...");
            startAccessPoint();
        } else if (_staEverConnected &&
                   (now - _lastStaReconnectCheckMs >= 5000)) {
            _lastStaReconnectCheckMs = now;
            WiFi.reconnect();
        }
    }

    // Process incoming UDP packets with a bound on packets per loop iteration.
    // The UDP socket remains bound to WIFI_UDP_PORT; replies use the active
    // client's latest remotePort (which may be ephemeral).
    const uint32_t startedUs = micros();
    int packetsProcessed = 0;
    // Check limits BEFORE parsePacket(). Arduino ESP32 returns zero from
    // parsePacket while a previous datagram is unread. Parsing a ninth packet
    // in the loop condition used to strand it and permanently stop all RX.
    while (packetsProcessed < WIFI_RX_MAX_PACKETS_PER_UPDATE &&
           (uint32_t)(micros() - startedUs) < WIFI_RX_BUDGET_US) {
        int packetSize = _udp.parsePacket();
        if (packetSize <= 0) break;
        packetsProcessed++;
        IPAddress remote = _udp.remoteIP();
        uint16_t port = _udp.remotePort();

        // Do not let arbitrary UDP discovery/noise claim the one-client slot.
        // A GCS datagram must begin with a MAVLink v1/v2 framing byte. Read
        // the bounded datagram before acquiring ownership so rejected traffic
        // cannot strand the bridge for the inactivity window.
        bool accepted = false;
        int bytesRead = 0;
        if (packetSize <= (int)sizeof(_rxBuffer)) {
            bytesRead = _udp.read(_rxBuffer, packetSize);
            accepted = (bytesRead == packetSize && bytesRead > 0 &&
                        (_rxBuffer[0] == 0xFE || _rxBuffer[0] == 0xFD) &&
                        _clientMgr.processIncomingPacket((uint32_t)remote, port, now));
        }
        if (accepted) {
            if (!_lastClientLogged || _lastLoggedIp != remote || _lastLoggedPort != port) {
                _lastLoggedIp = remote;
                _lastLoggedPort = port;
                _lastClientLogged = true;
                Serial.printf("[WIFI] Active GCS route: %s:%u (ownership keyed by IP)\n",
                              remote.toString().c_str(), port);
            }
            if (bytesRead == packetSize) {
                telemHandler.ingestBytes(_rxBuffer, (size_t)bytesRead);
            }
        }
        // ESP32 flush() discards this RX datagram (it does not wait for TX).
        // Also release short/failed reads, without an unbounded drain loop.
        _udp.flush();
    }

    _clientMgr.checkTimeout(now);
    if (!_clientMgr.hasActiveClient(now) && _lastClientLogged) {
        _lastClientLogged = false;
        Serial.println("[WIFI] Active GCS client expired after inactivity. Resumed discovery broadcast.");
    }
}

void WifiTelemetry::sendMavlinkPacket(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0 || len > WIFI_UDP_MAX_PAYLOAD_BYTES || !isConnected()) return;

    uint32_t now = millis();
    IPAddress destination;
    uint16_t destinationPort;
    if (_clientMgr.hasActiveClient(now)) {
        destination = IPAddress(_clientMgr.getClientIp());
        destinationPort = WIFI_GCS_UDP_PORT ? WIFI_GCS_UDP_PORT : _clientMgr.getClientPort();
    } else {
        destination = _broadcastIP;
        destinationPort = WIFI_UDP_PORT;
    }

    // The destination is intentionally the GCS source port for unicast. A
    // fixed 14550 destination would miss QGC/Mission Planner sockets that use
    // an ephemeral source port, while 14550 remains the local listen port.
    if (_udp.beginPacket(destination, destinationPort) != 1) return;
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

bool WifiTelemetry::hasActiveClient() const {
    return _clientMgr.hasActiveClient(millis());
}

IPAddress WifiTelemetry::getClientIP() const {
    return IPAddress(_clientMgr.getClientIp());
}

uint16_t WifiTelemetry::getClientPort() const {
    return _clientMgr.getClientPort();
}

#endif // ESP32
