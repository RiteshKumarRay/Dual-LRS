#include "mavlink_handler.h"

// --- RingBuffer Implementation ---
RingBuffer::RingBuffer(size_t size) : _size(size), _head(0), _tail(0) {
    _buffer = new uint8_t[size];
}

RingBuffer::~RingBuffer() {
    delete[] _buffer;
}

bool RingBuffer::push(uint8_t byte) {
    size_t nextHead = (_head + 1) % _size;
    if (nextHead == _tail) {
        return false; // Full
    }
    _buffer[_head] = byte;
    _head = nextHead;
    return true;
}

size_t RingBuffer::pushBytes(const uint8_t* data, size_t length) {
    size_t count = 0;
    for (size_t i = 0; i < length; ++i) {
        if (!push(data[i])) break;
        count++;
    }
    return count;
}

int RingBuffer::pop() {
    if (_head == _tail) return -1; // Empty
    uint8_t val = _buffer[_tail];
    _tail = (_tail + 1) % _size;
    return val;
}

size_t RingBuffer::popBytes(uint8_t* buffer, size_t maxLen) {
    size_t count = 0;
    while (count < maxLen && _head != _tail) {
        buffer[count++] = _buffer[_tail];
        _tail = (_tail + 1) % _size;
    }
    return count;
}

size_t RingBuffer::available() const {
    if (_head >= _tail) {
        return _head - _tail;
    }
    return _size - (_tail - _head);
}

size_t RingBuffer::freeSpace() const {
    return _size - available() - 1;
}

void RingBuffer::clear() {
    _head = _tail = 0;
}

// --- MavlinkHandler Implementation ---
MavlinkHandler::MavlinkHandler(Stream& localSerial)
    : _localSerial(localSerial), _txQueue(TELEM_BUFFER_SIZE) {}

void MavlinkHandler::begin() {
    _txQueue.clear();
}

void MavlinkHandler::readFromLocal() {
    while (_localSerial.available() > 0) {
        uint8_t b = (uint8_t)_localSerial.read();
        _txQueue.push(b);
    }
}

size_t MavlinkHandler::getOutboundPayload(uint8_t* dest, size_t maxLen) {
    return _txQueue.popBytes(dest, maxLen);
}

void MavlinkHandler::writeToLocal(const uint8_t* src, size_t length) {
    if (src != nullptr && length > 0) {
        _localSerial.write(src, length);
    }
}

// MAVLink X.25 CRC accumulator
static inline void mavlink_crc_accumulate(uint8_t data, uint16_t* crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void MavlinkHandler::injectRadioStatus(uint8_t rssi, uint8_t remRssi, uint8_t txBufPct, uint16_t rxErrors) {
    static uint8_t mavSeq = 0;

    // MAVLink v1 RADIO_STATUS (msgid: 109, len: 9, crc_extra: 185)
    uint8_t packet[6 + 9 + 2]; // 6 header + 9 payload + 2 checksum = 17 bytes
    packet[0] = 0xFE;          // MAVLink v1 magic
    packet[1] = 9;             // Payload length
    packet[2] = mavSeq++;      // Packet sequence
    packet[3] = 51;            // System ID (Radio)
    packet[4] = 68;            // Component ID (MAV_COMP_ID_TELEMETRY_RADIO)
    packet[5] = 109;           // Message ID (RADIO_STATUS)

    // Payload (Little Endian)
    packet[6] = (uint8_t)(rxErrors & 0xFF);
    packet[7] = (uint8_t)((rxErrors >> 8) & 0xFF);
    packet[8] = 0; // fixed count (low)
    packet[9] = 0; // fixed count (high)
    packet[10] = rssi;
    packet[11] = remRssi;
    packet[12] = txBufPct;
    packet[13] = 0; // local noise
    packet[14] = 0; // remote noise

    // Calculate CRC
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 6 + 9; ++i) {
        mavlink_crc_accumulate(packet[i], &crc);
    }
    mavlink_crc_accumulate(185, &crc); // CRC_EXTRA for msg 109

    packet[15] = (uint8_t)(crc & 0xFF);
    packet[16] = (uint8_t)((crc >> 8) & 0xFF);

    _localSerial.write(packet, sizeof(packet));
}
