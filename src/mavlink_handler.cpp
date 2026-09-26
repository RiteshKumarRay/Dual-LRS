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
    if (length == 0 || data == nullptr) return 0;
    if (freeSpace() < length) {
        return 0; // Atomic all-or-nothing: NEVER push a truncated MAVLink frame
    }
    for (size_t i = 0; i < length; ++i) {
        _buffer[_head] = data[i];
        _head = (_head + 1) % _size;
    }
    return length;
}

int RingBuffer::pop() {
    if (_head == _tail) return -1; // Empty
    int val = _buffer[_tail];
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

int RingBuffer::peek(size_t offset) const {
    if (offset >= available()) return -1;
    return _buffer[(_tail + offset) % _size];
}

void RingBuffer::clear() {
    _head = _tail = 0;
}

#include <string.h>

// MAVLink X.25 CRC accumulator (standard MAVLink CRC algorithm)
static inline void mavlink_crc_accumulate(uint8_t data, uint16_t* crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// Helper: identifies critical MAVLink packets that must NEVER be dropped during telemetry congestion.
// Parameters, heartbeats, status texts, commands, and mission items are strictly preserved.
static inline bool isCriticalMavlinkMessage(uint32_t msgid) {
    switch (msgid) {
        case 0:    // HEARTBEAT
        case 11:   // SET_MODE
        case 20:   // PARAM_REQUEST_READ
        case 21:   // PARAM_REQUEST_LIST
        case 22:   // PARAM_VALUE
        case 23:   // PARAM_SET
        case 39:   // MISSION_ITEM
        case 40:   // MISSION_REQUEST
        case 41:   // MISSION_SET_CURRENT
        case 43:   // MISSION_REQUEST_LIST
        case 44:   // MISSION_COUNT
        case 47:   // MISSION_ACK
        case 51:   // MISSION_REQUEST_INT
        case 70:   // RC_CHANNELS_OVERRIDE
        case 73:   // MISSION_ITEM_INT
        case 76:   // COMMAND_LONG
        case 77:   // COMMAND_ACK
        case 110:  // FILE_TRANSFER_PROTOCOL (MAVFTP)
        case 253:  // STATUSTEXT
            return true;
        default:
            return false;
    }
}

// --- MavlinkHandler Implementation ---
MavlinkHandler::MavlinkHandler(Stream& localSerial)
    : _localSerial(localSerial),
      _txQueue(TELEM_BUFFER_SIZE) {}

void MavlinkHandler::begin() {
    _txQueue.clear();
    _lastRfPacketMs = millis();  // Prevent safety-fallback from firing immediately on boot
    _gndFrameComplete = true;
    _pendingStatusReady = false;
    _hbPending = false;
    _hbLen = 0;
    _stPending = false;
    _stSending = false;
    _stLen = 0;
    _gndFrameBufLen = 0;
}

bool MavlinkHandler::pushByte(uint8_t b) {
    return _txQueue.push(b);
}

size_t MavlinkHandler::pushBytes(const uint8_t* data, size_t length) {
    return _txQueue.pushBytes(data, length);
}

// ============================================================================
// readFromLocal()
// Dual-role local port parser:
//
// AIR UNIT:
//   - Parses FC MAVLink stream from SerialTELEM (UART2).
//   - Priority Cache: HEARTBEAT (msgid 0) and STATUSTEXT (msgid 253) bypass the
//     FIFO queue so flight mode changes and warnings jump immediately to the front.
//   - Parameter Bursts: Full parameter table (~38 KB) is absorbed into the 48 KB RAM
//     FIFO without throttling ArduPilot txbuf below 33.
//   - Telemetry Pacing: Rate-limits non-critical telemetry (IMU, HUD) during parameter floods.
//
// GROUND UNIT:
//   - Ingests GCS commands from USB CDC / UART.
//   - Load Shedding: Drops QGC TIMESYNC (msgid 111) outright to eliminate ~280 B/s bloat.
//   - Priority Bypass: Routes GCS HEARTBEAT (msgid 0) to _hbCache, draining 4B per slot.
//   - Critical Protection: Ensures parameter requests, mode switches, and commands
//     are preserved in the 1 KB FIFO without head-of-line blocking.
// ============================================================================
void MavlinkHandler::readFromLocal() {
    uint32_t now = millis();
    // Inter-byte timeout: if > 50ms elapsed between bytes and packet is incomplete, reset to IDLE
    if (now - _lastLocalByteMs > 50 && _rxState != RxState::IDLE) {
        _rxState = RxState::IDLE;
        _rxIndex = 0;
    }

    // Allow draining up to 512 bytes per call to match the hardware SERIAL_RX_BUFFER_SIZE.
    // In STM32duino, available() and read() read from non-blocking RAM ring buffer (takes nanoseconds),
    // so draining prevents UART hardware buffer overrun during parameter and telemetry bursts.
    static const int MAX_LOCAL_BYTES_PER_CALL = 512;
    int localReadCount = 0;

    while (_localSerial.available() > 0 && localReadCount++ < MAX_LOCAL_BYTES_PER_CALL) {
        _lastLocalByteMs = millis();
        _rawBytesRead++;
        uint8_t c = (uint8_t)_localSerial.read();

        if (_rxState == RxState::IDLE) {
            if (c == 0xFE || c == 0xFD) {
                _rxBuffer[0] = c;
                _rxIndex = 1;
                _rxState = (c == 0xFE) ? RxState::V1_LEN : RxState::V2_LEN;
            }
        } else if (_rxState == RxState::V1_LEN) {
            _rxBuffer[1] = c;
            _rxExpectedLen = c + 8;     // 6 header + payload + 2 CRC
            _rxIndex = 2;
            _rxState = RxState::V1_PAYLOAD;
        } else if (_rxState == RxState::V1_PAYLOAD) {
            _rxBuffer[_rxIndex++] = c;
            if (_rxIndex >= _rxExpectedLen) {
                uint8_t payloadLen = _rxBuffer[1];
                uint8_t sysid = _rxBuffer[3];
                uint8_t msgid = _rxBuffer[5];

                // Sanity validation: reject corrupted bytes masquerading as MAVLink
                bool valid = (sysid > 0);
                if (msgid == 22 && (payloadLen == 0 || payloadLen > 25)) valid = false; // PARAM_VALUE
                else if (msgid == 0 && (payloadLen == 0 || payloadLen > 9)) valid = false; // HEARTBEAT
                else if (msgid == 20 && (payloadLen == 0 || payloadLen > 20)) valid = false; // PARAM_REQUEST_READ
                else if (msgid == 21 && (payloadLen == 0 || payloadLen > 2)) valid = false; // PARAM_REQUEST_LIST
                else if (msgid == 23 && (payloadLen == 0 || payloadLen > 23)) valid = false; // PARAM_SET
                else if (msgid == 109 && (payloadLen == 0 || payloadLen > 9)) valid = false; // RADIO_STATUS

                if (!valid) {
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                }

                _validPackets++;
                _lastValidPacketMs = millis();
                if (msgid == 109) {
                    // NEVER forward RADIO_STATUS over RF! Both Air and Ground generate it locally.
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
#if defined(DUAL_LRS_ROLE_AIR)
                } else if (msgid == 0) {
                    _validHeartbeats++;
                    _lastHeartbeatMs = millis();
                    if (_rxExpectedLen <= sizeof(_hbCache)) {
                        memcpy(_hbCache, _rxBuffer, _rxExpectedLen);
                        _hbLen = (uint8_t)_rxExpectedLen;
                        _hbPending = true;
                        _rxState = RxState::IDLE;
                        _rxIndex = 0;
                        continue;
                    }
                } else if (msgid == 253) {
                    // Air: High-priority STATUSTEXT bypass (mode changes jump to front of FIFO)
                    if (!_stPending && _rxExpectedLen <= sizeof(_stCache)) {
                        memcpy(_stCache, _rxBuffer, _rxExpectedLen);
                        _stLen = (uint8_t)_rxExpectedLen;
                        _stPending = true;
                        _stSending = false;
                        _rxState = RxState::IDLE;
                        _rxIndex = 0;
                        continue;
                    }
                } else if (msgid == 22) {
                    _validParamValues++;
                    _lastParamRxMs = millis();
                }

                bool drop = false;
                size_t freeSpace = _txQueue.freeSpace();

                bool isEssentialTelem = (msgid == 1   ||  // SYS_STATUS (battery)
                                         msgid == 24  ||  // GPS_RAW_INT (sats)
                                         msgid == 33  ||  // GLOBAL_POSITION_INT (lat/lon/alt)
                                         msgid == 30  ||  // ATTITUDE
                                         msgid == 74  ||  // VFR_HUD
                                         msgid == 147);   // BATTERY_STATUS

                if (freeSpace < _rxExpectedLen) {
                    drop = true; // Hard limit: physically no room in FIFO
                } else if (isCriticalMavlinkMessage(msgid)) {
                    drop = false; // Critical (params, heartbeat, mission, commands): always pass
                } else if (isEssentialTelem) {
                    // Allow essential HUD telemetry at 1Hz each, even during param flood
                    uint32_t* lastMs = nullptr;
                    if      (msgid == 1)   lastMs = &_lastSysStatusMs;
                    else if (msgid == 24)  lastMs = &_lastGpsMs;
                    else if (msgid == 33)  lastMs = &_lastGlobalPosMs;
                    else if (msgid == 30)  lastMs = &_lastAttitudeMs;
                    else if (msgid == 74)  lastMs = &_lastVfrHudMs;
                    else if (msgid == 147) lastMs = &_lastBatteryStatusMs;
                    if (lastMs && (millis() - *lastMs >= 1000)) {
                        *lastMs = millis();
                        drop = false; // Let one through per second
                    } else {
                        drop = true; // Rate-limited: not yet
                    }
                } else if (_rxExpectedLen > MAX_PAYLOAD_AIR_SLOT) {
                    drop = true; // Non-critical packets exceeding slot size should never fragment
                } else {
                    // Non-critical streaming telemetry (RC channels, raw IMU, etc.):
                    // Drop during param download or when queue has backlog
                    bool paramActive = (millis() - _lastParamRxMs < 2500);
                    bool queueBacklog = (_txQueue.available() > 200);
                    if (paramActive || queueBacklog) {
                        drop = true;
                    }
                }

                if (!drop) {
                    _txQueue.pushBytes(_rxBuffer, _rxExpectedLen);
                }
#else
                } else if (msgid == 0) {
                    // Ground: GCS Heartbeat priority bypass — send first, never queue behind traffic
                    _validHeartbeats++;
                    _lastHeartbeatMs = millis();
                    if (!_hbPending && _rxExpectedLen <= sizeof(_hbCache)) {
                        memcpy(_hbCache, _rxBuffer, _rxExpectedLen);
                        _hbLen = (uint8_t)_rxExpectedLen;
                        _hbPending = true;
                    }
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                } else if (msgid == 111) {
                    // Ground: Drop TIMESYNC from QGC outright (saves ~280 B/s of congestion)
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                }

                // Ground Unit: GCS uplink commands, parameter requests, mission items
                if (_txQueue.freeSpace() >= _rxExpectedLen) {
                    _txQueue.pushBytes(_rxBuffer, _rxExpectedLen);
                }
#endif
                _rxState = RxState::IDLE;
            }
        } else if (_rxState == RxState::V2_LEN) {
            _rxBuffer[1] = c;
            _rxIndex = 2;
            _rxState = RxState::V2_INC_FLAGS;
        } else if (_rxState == RxState::V2_INC_FLAGS) {
            _rxBuffer[2] = c;
            _signatureExpected = (c & 0x01);
            _rxIndex = 3;
            _rxState = RxState::V2_CMP_FLAGS;
        } else if (_rxState == RxState::V2_CMP_FLAGS) {
            _rxBuffer[3] = c;
            _rxExpectedLen = _rxBuffer[1] + 12 + (_signatureExpected ? 13 : 0);
            _rxIndex = 4;
            _rxState = RxState::V2_PAYLOAD;
        } else if (_rxState == RxState::V2_PAYLOAD) {
            _rxBuffer[_rxIndex++] = c;
            if (_rxIndex >= _rxExpectedLen) {
                // MAVLink v2: msgid is 24-bit at bytes [7:9]
                uint8_t payloadLen = _rxBuffer[1];
                uint8_t incompatFlags = _rxBuffer[2];
                uint8_t sysid = _rxBuffer[5];
                uint32_t msgid = _rxBuffer[7] | ((uint32_t)_rxBuffer[8] << 8) | ((uint32_t)_rxBuffer[9] << 16);

                // Sanity validation: reject corrupted bytes masquerading as MAVLink
                // In MAVLink v2, trailing zeros in payloads are truncated, so payloadLen can be <= MAX_LEN
                bool valid = (incompatFlags <= 1 && sysid > 0);
                if (msgid == 22 && (payloadLen == 0 || payloadLen > 25)) valid = false; // PARAM_VALUE
                else if (msgid == 0 && (payloadLen == 0 || payloadLen > 9)) valid = false; // HEARTBEAT
                else if (msgid == 20 && (payloadLen == 0 || payloadLen > 20)) valid = false; // PARAM_REQUEST_READ
                else if (msgid == 21 && (payloadLen == 0 || payloadLen > 2)) valid = false; // PARAM_REQUEST_LIST
                else if (msgid == 23 && (payloadLen == 0 || payloadLen > 23)) valid = false; // PARAM_SET
                else if (msgid == 76 && (payloadLen == 0 || payloadLen > 33)) valid = false; // COMMAND_LONG
                else if (msgid == 109 && (payloadLen == 0 || payloadLen > 9)) valid = false; // RADIO_STATUS

                if (!valid) {
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                }

                _validPackets++;
                _lastValidPacketMs = millis();
                if (msgid == 109) {
                    // NEVER forward RADIO_STATUS over RF! Both Air and Ground generate it locally.
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
#if defined(DUAL_LRS_ROLE_AIR)
                } else if (msgid == 0) {
                    _validHeartbeats++;
                    _lastHeartbeatMs = millis();
                    if (_rxExpectedLen <= sizeof(_hbCache)) {
                        memcpy(_hbCache, _rxBuffer, _rxExpectedLen);
                        _hbLen = (uint8_t)_rxExpectedLen;
                        _hbPending = true;
                        _rxState = RxState::IDLE;
                        _rxIndex = 0;
                        continue;
                    }
                } else if (msgid == 253) {
                    // Air: High-priority STATUSTEXT bypass (mode changes jump to front of FIFO)
                    if (!_stPending && _rxExpectedLen <= sizeof(_stCache)) {
                        memcpy(_stCache, _rxBuffer, _rxExpectedLen);
                        _stLen = (uint8_t)_rxExpectedLen;
                        _stPending = true;
                        _stSending = false;
                        _rxState = RxState::IDLE;
                        _rxIndex = 0;
                        continue;
                    }
                } else if (msgid == 22) {
                    _validParamValues++;
                    _lastParamRxMs = millis();
                }

                bool drop = false;
                size_t freeSpace = _txQueue.freeSpace();

                bool isEssentialTelem = (msgid == 1   ||  // SYS_STATUS (battery)
                                         msgid == 24  ||  // GPS_RAW_INT (sats)
                                         msgid == 33  ||  // GLOBAL_POSITION_INT (lat/lon/alt)
                                         msgid == 30  ||  // ATTITUDE
                                         msgid == 74  ||  // VFR_HUD
                                         msgid == 147);   // BATTERY_STATUS

                if (freeSpace < _rxExpectedLen) {
                    drop = true; // Hard limit: physically no room in FIFO
                } else if (isCriticalMavlinkMessage(msgid)) {
                    drop = false; // Critical (params, heartbeat, mission, commands): always pass
                } else if (isEssentialTelem) {
                    // Allow essential HUD telemetry at 1Hz each, even during param flood
                    uint32_t* lastMs = nullptr;
                    if      (msgid == 1)   lastMs = &_lastSysStatusMs;
                    else if (msgid == 24)  lastMs = &_lastGpsMs;
                    else if (msgid == 33)  lastMs = &_lastGlobalPosMs;
                    else if (msgid == 30)  lastMs = &_lastAttitudeMs;
                    else if (msgid == 74)  lastMs = &_lastVfrHudMs;
                    else if (msgid == 147) lastMs = &_lastBatteryStatusMs;
                    if (lastMs && (millis() - *lastMs >= 1000)) {
                        *lastMs = millis();
                        drop = false; // Let one through per second
                    } else {
                        drop = true; // Rate-limited: not yet
                    }
                } else if (_rxExpectedLen > MAX_PAYLOAD_AIR_SLOT) {
                    drop = true; // Non-critical packets exceeding slot size should never fragment
                } else {
                    // Non-critical streaming telemetry (RC channels, raw IMU, etc.):
                    // Drop during param download or when queue has backlog
                    bool paramActive = (millis() - _lastParamRxMs < 2500);
                    bool queueBacklog = (_txQueue.available() > 200);
                    if (paramActive || queueBacklog) {
                        drop = true;
                    }
                }

                if (!drop) {
                    _txQueue.pushBytes(_rxBuffer, _rxExpectedLen);
                }
#else
                } else if (msgid == 0) {
                    // Ground: GCS Heartbeat priority bypass — send first, never queue behind traffic
                    _validHeartbeats++;
                    _lastHeartbeatMs = millis();
                    if (!_hbPending && _rxExpectedLen <= sizeof(_hbCache)) {
                        memcpy(_hbCache, _rxBuffer, _rxExpectedLen);
                        _hbLen = (uint8_t)_rxExpectedLen;
                        _hbPending = true;
                    }
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                } else if (msgid == 111) {
                    // Ground: Drop TIMESYNC from QGC outright (saves ~280 B/s of congestion)
                    _rxState = RxState::IDLE;
                    _rxIndex = 0;
                    continue;
                }

                // Ground Unit: GCS uplink commands, parameter requests, mission items
                if (_txQueue.freeSpace() >= _rxExpectedLen) {
                    _txQueue.pushBytes(_rxBuffer, _rxExpectedLen);
                }
#endif
                _rxState = RxState::IDLE;
            }
        }

        // Safety: if parse buffer overflows (impossible msg length), reset
        if (_rxIndex >= sizeof(_rxBuffer)) {
            _rxState = RxState::IDLE;
            _rxIndex = 0;
        }
    }
}

// ============================================================================
// getOutboundPayload()
// Packs outbound telemetry frames to transmit over a radio slot.
// ============================================================================
size_t MavlinkHandler::getOutboundPayload(uint8_t* dest, size_t maxLen) {
    if (dest == nullptr || maxLen == 0) return 0;

    size_t avail = _txQueue.available();
#if defined(DUAL_LRS_ROLE_AIR)
    if (avail == 0 && !_hbPending && !_stPending && _fragmentRemaining == 0) {
        return 0;
    }
#else
    if (avail == 0 && !_hbPending && _fragmentRemaining == 0) {
        return 0;
    }
#endif

    // If we are currently continuing a fragmented multi-slot packet (> maxLen) from _txQueue
    if (_fragmentRemaining > 0) {
        size_t toPop = (_fragmentRemaining < maxLen) ? _fragmentRemaining : maxLen;
        if (toPop > avail) toPop = avail;
        size_t popped = _txQueue.popBytes(dest, toPop);
        _fragmentRemaining -= popped;
        return popped;
    }

#if defined(DUAL_LRS_ROLE_AIR)
    // 1. If a multi-slot STATUSTEXT transmission is already in progress, finish it first!
    if (_stSending && _stPending && _stLen > 0) {
        size_t toSend = (_stLen < maxLen) ? _stLen : maxLen;
        memcpy(dest, _stCache, toSend);
        if (toSend < _stLen) {
            memmove(_stCache, _stCache + toSend, _stLen - toSend);
            _stLen -= (uint8_t)toSend;
        } else {
            _stPending = false;
            _stLen = 0;
            _stSending = false;
        }
        return toSend;
    }

    // 2. High-priority Heartbeat bypass: always send HEARTBEAT first if pending!
    if (_hbPending && _hbLen > 0) {
        size_t toSend = (_hbLen < maxLen) ? _hbLen : maxLen;
        memcpy(dest, _hbCache, toSend);
        if (toSend < _hbLen) {
            memmove(_hbCache, _hbCache + toSend, _hbLen - toSend);
            _hbLen -= (uint8_t)toSend;
        } else {
            _hbPending = false;
            _hbLen = 0;
        }
        return toSend;
    }

    // 3. High-priority STATUSTEXT bypass: start sending pending STATUSTEXT
    if (_stPending && _stLen > 0) {
        _stSending = true;
        size_t toSend = (_stLen < maxLen) ? _stLen : maxLen;
        memcpy(dest, _stCache, toSend);
        if (toSend < _stLen) {
            memmove(_stCache, _stCache + toSend, _stLen - toSend);
            _stLen -= (uint8_t)toSend;
        } else {
            _stPending = false;
            _stLen = 0;
            _stSending = false;
        }
        return toSend;
    }
#else
    // Ground: Fragment-aware Heartbeat bypass (4-byte slot sends ~17-23B heartbeat over ~5 slots)
    if (_hbPending && _hbLen > 0) {
        size_t toSend = (_hbLen < maxLen) ? _hbLen : maxLen;
        memcpy(dest, _hbCache, toSend);
        if (toSend < _hbLen) {
            memmove(_hbCache, _hbCache + toSend, _hbLen - toSend);
            _hbLen -= (uint8_t)toSend;
        } else {
            _hbPending = false;
            _hbLen = 0;
        }
        return toSend;
    }
#endif

    size_t packedBytes = 0;

    avail = _txQueue.available();

    // Scan complete MAVLink frames from the head of _txQueue
    while (packedBytes < maxLen && (avail > 0)) {
        size_t remainingSlot = maxLen - packedBytes;

        int magic = _txQueue.peek(0);
        if (magic < 0) break;

        size_t frameLen = 0;
        if (magic == 0xFE) { // MAVLink v1
            if (avail < 2) break;
            int payloadLen = _txQueue.peek(1);
            if (payloadLen < 0) break;
            frameLen = (size_t)payloadLen + 8; // 6 header + payload + 2 CRC
        } else if (magic == 0xFD) { // MAVLink v2
            if (avail < 3) break;
            int payloadLen = _txQueue.peek(1);
            int incompatFlags = _txQueue.peek(2);
            if (payloadLen < 0 || incompatFlags < 0) break;
            size_t sigLen = (incompatFlags & 0x01) ? 13 : 0;
            frameLen = (size_t)payloadLen + 12 + sigLen; // 10 header + payload + 2 CRC + sig
        } else {
            // Discard stray non-magic byte from queue head to realign
            _txQueue.pop(); // discard 1 byte
            avail = _txQueue.available();
            continue;
        }

        // Sanity: if frameLen is impossible (> 280), discard single byte and realign
        if (frameLen > 280) {
            _txQueue.pop();
            avail = _txQueue.available();
            continue;
        }

        if (avail < frameLen) {
            // Since readFromLocal() only pushes complete frames, if avail < frameLen,
            // this magic byte was false (e.g. data byte). Pop 1 byte to realign!
            _txQueue.pop();
            avail = _txQueue.available();
            continue;
        }

        // Does this complete frame fit in the remaining space of this slot?
        if (frameLen <= remainingSlot) {
            _txQueue.popBytes(dest + packedBytes, frameLen);
            packedBytes += frameLen;
            avail = _txQueue.available();
        } else {
            // Frame does not fit in remaining slot space
            if (packedBytes > 0) {
                // Transmit already packed complete frames; save this frame intact for next slot!
                break;
            }

            // Single frame is strictly larger than maxLen: must fragment across slots
            size_t popped = _txQueue.popBytes(dest, maxLen);
            if (frameLen > popped) {
                _fragmentRemaining = frameLen - popped;
            }
            return popped;
        }
    }

    return packedBytes;
}

void MavlinkHandler::_writeRaw(const uint8_t* buf, size_t len) {
    if (buf == nullptr || len == 0) return;
    _localSerial.write(buf, len);
}

void MavlinkHandler::_outputToLocal(const uint8_t* buf, size_t len) {
    if (buf == nullptr || len == 0) return;
    _localSerial.write(buf, len);
}

// ============================================================================
// writeToLocal()
// Air:    Raw bulk write of GCS uplink bytes to FC UART — no reassembly needed.
// Ground: Frame-boundary parser — reassembles complete MAVLink frames before
//         writing to QGC USB, enabling clean RADIO_STATUS injection at boundaries.
// ============================================================================
void MavlinkHandler::writeToLocal(const uint8_t* src, size_t length) {
    if (src == nullptr || length == 0) return;

#if defined(DUAL_LRS_ROLE_AIR)
    // Air unit: Direct stream write of GCS uplink bytes to FC UART.
    // ArduPilot's native MAVLink parser reassembles multi-slot packet fragments.
    // Zero buffering latency, no timeout drops for fragmented commands!
    _localSerial.write(src, length);
#else
    // Ground unit: frame-boundary parser.
    // Reassembles multi-slot packet fragments into complete MAVLink frames before
    // writing to local USB. Prevents partial-packet delivery and protects against
    // RADIO_STATUS mid-frame collision.
    uint32_t now = millis();
    // Timeout: if > 350ms passed since last RF reception, any incomplete frame from previous slot is dead
    if (now - _lastRfPacketMs > 350) {
        _gndRxState = GndRxState::IDLE;
        _gndFrameComplete = true;
        _gndFrameBufLen = 0;
    }
    _lastRfPacketMs = now;

    // If incoming RF packet begins with a MAVLink magic byte (0xFE or 0xFD)
    // while mid-frame, the tail of the previous frame was lost over RF.
    // Discard the truncated previous frame immediately to prevent frame splicing / corruption.
    if ((src[0] == 0xFE || src[0] == 0xFD) && _gndRxState != GndRxState::IDLE) {
        _gndRxState = GndRxState::IDLE;
        _gndFrameBufLen = 0;
        _gndFrameComplete = true;
    }

    // Track frame boundaries and buffer complete frames
    for (size_t i = 0; i < length; i++) {
        uint8_t c = src[i];
        if (_gndFrameBufLen < sizeof(_gndFrameBuf)) {
            _gndFrameBuf[_gndFrameBufLen++] = c;
        } else {
            // Safety reset on overflow
            _gndFrameBufLen = 0;
            _gndRxState = GndRxState::IDLE;
            _gndFrameComplete = true;
            continue;
        }

        switch (_gndRxState) {
            case GndRxState::IDLE:
                if (c == 0xFE) {
                    _gndRxState = GndRxState::V1_LEN;
                    _gndFrameComplete = false;
                } else if (c == 0xFD) {
                    _gndRxState = GndRxState::V2_LEN;
                    _gndFrameComplete = false;
                } else {
                    _gndFrameBufLen = 0; // Discard stray bytes outside frame
                }
                break;

            case GndRxState::V1_LEN:
                _gndExpectedLen = c + 8;   // 6 header + payload + 2 CRC
                _gndRxCount = 2;
                _gndRxState = GndRxState::V1_PAYLOAD;
                break;

            case GndRxState::V1_PAYLOAD:
                _gndRxCount++;
                if (_gndRxCount >= _gndExpectedLen) {
                    _gndRxState = GndRxState::IDLE;
                    _gndFrameComplete = true;
                    // Validate MAVLink v1 header before sending: sysid must be non-zero
                    if (_gndFrameBufLen == _gndExpectedLen && _gndFrameBufLen >= 8 && _gndFrameBuf[3] > 0) {
                        _localSerial.write(_gndFrameBuf, _gndFrameBufLen);
                    }
                    _gndFrameBufLen = 0;
                }
                break;

            case GndRxState::V2_LEN:
                _gndExpectedLen = c + 12;
                _gndRxCount = 2;
                _gndRxState = GndRxState::V2_INC_FLAGS;
                break;

            case GndRxState::V2_INC_FLAGS:
                _gndSignatureExpected = (c & 0x01);
                _gndRxCount++;
                _gndRxState = GndRxState::V2_CMP_FLAGS;
                break;

            case GndRxState::V2_CMP_FLAGS:
                if (_gndSignatureExpected) _gndExpectedLen += 13;
                _gndRxCount++;
                _gndRxState = GndRxState::V2_PAYLOAD;
                break;

            case GndRxState::V2_PAYLOAD:
                _gndRxCount++;
                if (_gndRxCount >= _gndExpectedLen) {
                    _gndRxState = GndRxState::IDLE;
                    _gndFrameComplete = true;
                    // Validate MAVLink v2 header before sending: sysid > 0, incompat_flags <= 1
                    if (_gndFrameBufLen == _gndExpectedLen && _gndFrameBufLen >= 12 &&
                        _gndFrameBuf[5] > 0 && _gndFrameBuf[2] <= 1) {
                        _localSerial.write(_gndFrameBuf, _gndFrameBufLen);
                    }
                    _gndFrameBufLen = 0;
                }
                break;
        }
    }

    // If we completed a MAVLink frame AND there's a pending RADIO_STATUS, inject now
    if (_gndFrameComplete && _pendingStatusReady) {
        _localSerial.write(_pendingStatusPkt, sizeof(_pendingStatusPkt));
        _pendingStatusReady = false;
    }
#endif
}

// ============================================================================
// injectRadioStatus()
// Air:    Direct injection to FC UART for flow control.
// Ground: Stores packet, deferred until next MAVLink frame boundary in writeToLocal()
//         to prevent corrupting in-flight multi-slot packets to GCS.
// ============================================================================
void MavlinkHandler::injectRadioStatus(uint8_t rssi, uint8_t remRssi, uint8_t txBufPct, uint16_t rxErrors, uint16_t txPackets) {
    static uint8_t mavSeq = 0;

    // Build MAVLink v1 RADIO_STATUS (msgid: 109, payload: 9 bytes, crc_extra: 185)
    uint8_t packet[17]; // 6 header + 9 payload + 2 CRC = 17 bytes
    packet[0] = 0xFE;          // MAVLink v1 magic
    packet[1] = 9;             // Payload length
    packet[2] = mavSeq++;      // Packet sequence
    packet[3] = 51;            // System ID (Radio: MAV_TYPE_GENERIC telemetry radio)
    packet[4] = 68;            // Component ID (MAV_COMP_ID_TELEMETRY_RADIO)
    packet[5] = 109;           // Message ID (RADIO_STATUS)

    // Payload — MAVLink RADIO_STATUS field order (little-endian):
    // rxerrors (uint16), fixed (uint16), rssi, remrssi, txbuf, noise, remnoise
    packet[6]  = (uint8_t)(rxErrors & 0xFF);
    packet[7]  = (uint8_t)((rxErrors >> 8) & 0xFF);
    packet[8]  = (uint8_t)(txPackets & 0xFF);         // fixed count low (reporting tx packets)
    packet[9]  = (uint8_t)((txPackets >> 8) & 0xFF);  // fixed count high
    packet[10] = rssi;
    packet[11] = remRssi;
    packet[12] = txBufPct;
    packet[13] = 0;            // local noise
    packet[14] = 0;            // remote noise

    // X.25 CRC over bytes [1..14] + CRC_EXTRA(185)
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i <= 14; ++i) {
        mavlink_crc_accumulate(packet[i], &crc);
    }
    mavlink_crc_accumulate(185, &crc);

    packet[15] = (uint8_t)(crc & 0xFF);
    packet[16] = (uint8_t)((crc >> 8) & 0xFF);

#if defined(DUAL_LRS_ROLE_AIR)
    // Air unit: send RADIO_STATUS directly to FC UART for immediate pacing
    _localSerial.write(packet, sizeof(packet));
#else
    // Ground: store for deferred injection at next MAVLink frame boundary.
    memcpy(_pendingStatusPkt, packet, sizeof(packet));
    _pendingStatusReady = true;

    // Safety: if no RF data has been received for > 1 second (link offline),
    // force-inject directly so GCS still sees link status.
    if (millis() - _lastRfPacketMs > 1000) {
        _localSerial.write(_pendingStatusPkt, sizeof(_pendingStatusPkt));
        _pendingStatusReady = false;
        _gndFrameComplete = true;
    }
#endif
}
