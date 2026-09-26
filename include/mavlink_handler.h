#pragma once
#include <Arduino.h>
#include "config.h"

// Ring buffer for high-throughput MAVLink buffering
class RingBuffer {
public:
    RingBuffer(size_t size);
    ~RingBuffer();

    bool push(uint8_t byte);
    size_t pushBytes(const uint8_t* data, size_t length);
    int pop();
    size_t popBytes(uint8_t* buffer, size_t maxLen);

    size_t available() const;
    size_t freeSpace() const;
    int peek(size_t offset = 0) const;
    void clear();

private:
    uint8_t* _buffer;
    size_t _size;
    volatile size_t _head;
    volatile size_t _tail;
};

class MavlinkHandler {
public:
    MavlinkHandler(Stream& localSerial);

    void begin();

    // Read bytes from local port (FC or GCS) into the outbound queue.
    // On Air unit: parses FC MAVLink stream, queues for RF TX with drop logic.
    void readFromLocal();

    // Pull a chunk of outbound telemetry to transmit over radio slot (transparent FIFO).
    size_t getOutboundPayload(uint8_t* dest, size_t maxLen);

    // Feed inbound payload received from radio into local port (FC or GCS).
    // On Ground unit: runs a MAVLink parser so RADIO_STATUS is only injected
    // between complete frames — never mid-packet (prevents stream corruption).
    void writeToLocal(const uint8_t* src, size_t length);

    // Directly push bytes into outbound queue
    bool pushByte(uint8_t b);
    size_t pushBytes(const uint8_t* data, size_t length);

    // Send a standard MAVLink RADIO_STATUS (ID 109) packet to local stream.
    // On Ground unit this is queued for injection between complete MAVLink frames.
    // On Air unit this is written directly to FC UART (flow control).
    void injectRadioStatus(uint8_t rssi, uint8_t remRssi, uint8_t txBufPct, uint16_t rxErrors, uint16_t txPackets = 0);

    size_t pendingBytes() const { return _txQueue.available(); }
    size_t freeSpace() const { return _txQueue.freeSpace(); }

    uint32_t validPackets() const { return _validPackets; }
    uint32_t lastValidPacketMs() const { return _lastValidPacketMs; }
    uint32_t validHeartbeats() const { return _validHeartbeats; }
    uint32_t lastHeartbeatMs() const { return _lastHeartbeatMs; }
    uint32_t validParamValues() const { return _validParamValues; }
    uint32_t rawBytesRead() const { return _rawBytesRead; }

private:
    uint32_t _rawBytesRead = 0;
    uint32_t _validPackets = 0;
    uint32_t _lastValidPacketMs = 0;
    uint32_t _validHeartbeats = 0;
    uint32_t _lastHeartbeatMs = 0;
    uint32_t _validParamValues = 0;
    uint32_t _lastParamRxMs = 0;
    uint32_t _lastLocalByteMs = 0;
    uint32_t _lastSysStatusMs = 0;   // SYS_STATUS  (msgid 1)   — battery
    uint32_t _lastGpsMs = 0;         // GPS_RAW_INT (msgid 24)  — GPS fix & satellites
    uint32_t _lastGlobalPosMs = 0;   // GLOBAL_POSITION_INT (msgid 33) — lat/lon/alt
    uint32_t _lastAttitudeMs = 0;    // ATTITUDE    (msgid 30)  — roll/pitch/yaw
    uint32_t _lastVfrHudMs = 0;      // VFR_HUD     (msgid 74)  — airspeed, alt
    uint32_t _lastBatteryStatusMs = 0; // BATTERY_STATUS (msgid 147) — smart battery
    // ---- Air-side TX parser (FC → RF queue) ----
    enum class RxState {
        IDLE,
        V1_LEN, V1_PAYLOAD,
        V2_LEN, V2_INC_FLAGS, V2_CMP_FLAGS, V2_PAYLOAD
    };
    RxState _rxState = RxState::IDLE;
    uint8_t _rxBuffer[296];
    uint16_t _rxIndex = 0;
    uint16_t _rxExpectedLen = 0;
    bool _signatureExpected = false;

    // ---- Ground-side RX parser (RF → USB, frame-boundary-safe) ----
    // Parses incoming RF byte stream to track MAVLink frame boundaries.
    // RADIO_STATUS is injected only when _gndFrameComplete == true.
    enum class GndRxState {
        IDLE,
        V1_LEN, V1_PAYLOAD,
        V2_LEN, V2_INC_FLAGS, V2_CMP_FLAGS, V2_PAYLOAD
    };
    GndRxState _gndRxState = GndRxState::IDLE;
    uint16_t   _gndExpectedLen = 0;
    uint16_t   _gndRxCount = 0;
    bool       _gndSignatureExpected = false;
    bool       _gndFrameComplete = true;  // true = safe to inject between frames

    // Pending RADIO_STATUS to inject at next frame boundary
    uint8_t  _pendingStatusPkt[17];
    bool     _pendingStatusReady = false;
    uint32_t _lastRfPacketMs = 0;

    Stream& _localSerial;
    void _outputToLocal(const uint8_t* buf, size_t len);

    // Internal: write raw bytes to _localSerial (used by both Air and Ground)
    void _writeRaw(const uint8_t* buf, size_t len);

    // Priority Heartbeat cache (bypasses FIFO queue to prevent GCS comms loss)
    uint8_t  _hbCache[32];
    uint8_t  _hbLen = 0;
    bool     _hbPending = false;

    // High-priority urgent response cache (bypasses 48KB param queue on Air: STATUSTEXT, MISSION_COUNT, COMMAND_ACK)
    uint8_t  _urgentCache[128];
    uint8_t  _urgentLen = 0;
    bool     _urgentPending = false;
    bool     _urgentSending = false;

    // Ground frame assembly buffer: guarantees only complete MAVLink frames are written to GCS
    uint8_t  _gndFrameBuf[296];
    uint16_t _gndFrameBufLen = 0;

    RingBuffer _txQueue;
    size_t _fragmentRemaining = 0;
};
