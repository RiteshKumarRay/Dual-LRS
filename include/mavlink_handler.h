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

    // Read bytes from local port (FC or GCS) into the outbound queue
    void readFromLocal();

    // Pull a chunk of outbound telemetry to transmit over radio slot
    size_t getOutboundPayload(uint8_t* dest, size_t maxLen);

    // Feed inbound payload received from radio into local port (FC or GCS)
    void writeToLocal(const uint8_t* src, size_t length);

    // Send a standard MAVLink RADIO_STATUS (ID 109) packet to local stream
    void injectRadioStatus(uint8_t rssi, uint8_t remRssi, uint8_t txBufPct, uint16_t rxErrors);

    size_t pendingBytes() const { return _txQueue.available(); }

private:
    Stream& _localSerial;
    RingBuffer _txQueue;
};
