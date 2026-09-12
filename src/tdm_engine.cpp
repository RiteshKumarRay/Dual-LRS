#include "tdm_engine.h"

// Standard CRC-16-CCITT (Polynomial 0x1021, Initial 0xFFFF)
uint16_t TdmEngine::calculateCrc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

TdmEngine::TdmEngine(E22Driver& radio, NodeRole role)
    : _radio(radio),
      _role(role),
      _currentSlot(TdmSlot::AIR_TRANSMIT),
      _frameStartTimeUs(0),
      _txSeqNum(0),
      _rxState(RxState::WAIT_MAGIC0),
      _rxBytesCount(0),
      _rxCrc(0),
      _rxCallback(nullptr) {}

void TdmEngine::begin() {
    _radio.begin();
    _frameStartTimeUs = micros();
    _stats.last_sync_ms = millis();
    _stats.synchronized = (_role == NodeRole::GROUND); // Ground starts as master
}

void TdmEngine::update() {
    updateSlotState();
    processIncomingRadioData();
}

void TdmEngine::updateSlotState() {
    uint32_t nowUs = micros();
    uint32_t elapsedUs = nowUs - _frameStartTimeUs;
    uint32_t framePeriodUs = TDM_FRAME_PERIOD_MS * 1000;

    if (elapsedUs >= framePeriodUs) {
        if (_role == NodeRole::GROUND || _stats.synchronized) {
            _frameStartTimeUs += framePeriodUs;
            elapsedUs = nowUs - _frameStartTimeUs;
        }
    }

    // Check for sync timeout on Air unit (1.5 seconds)
    if (_role == NodeRole::AIR) {
        if (millis() - _stats.last_sync_ms > 1500) {
            _stats.synchronized = false;
        }
    }

    uint32_t airEndUs    = TDM_AIR_SLOT_MS * 1000;
    uint32_t guard1EndUs = airEndUs + (TDM_GUARD_GAP1_MS * 1000);
    uint32_t groundEndUs = guard1EndUs + (TDM_GROUND_SLOT_MS * 1000);

    if (elapsedUs < airEndUs) {
        _currentSlot = TdmSlot::AIR_TRANSMIT;
    } else if (elapsedUs < guard1EndUs) {
        _currentSlot = TdmSlot::GUARD_GAP_1;
    } else if (elapsedUs < groundEndUs) {
        _currentSlot = TdmSlot::GROUND_TRANSMIT;
    } else {
        _currentSlot = TdmSlot::GUARD_GAP_2;
    }
}

bool TdmEngine::canTransmit() const {
    if (_radio.isBusy()) {
        return false;
    }

    if (_role == NodeRole::AIR) {
        // Air transmits during AIR_TRANSMIT slot
        return (_currentSlot == TdmSlot::AIR_TRANSMIT);
    } else {
        // Ground transmits during GROUND_TRANSMIT slot
        return (_currentSlot == TdmSlot::GROUND_TRANSMIT);
    }
}

bool TdmEngine::sendPacket(LrsPacketType type, const uint8_t* payload, uint8_t length) {
    if (!canTransmit()) {
        return false;
    }

    if (length > MAX_PAYLOAD_PER_SLOT) {
        length = MAX_PAYLOAD_PER_SLOT;
    }

    LrsFrameHeader header;
    header.magic0 = DUAL_LRS_MAGIC_0;
    header.magic1 = DUAL_LRS_MAGIC_1;
    header.packet_type = static_cast<uint8_t>(type);
    header.seq_num = _txSeqNum++;
    header.payload_len = length;

    // Compute CRC over header and payload
    uint16_t crc = 0xFFFF;
    crc = calculateCrc16((const uint8_t*)&header, sizeof(header));
    if (length > 0 && payload != nullptr) {
        // Continue CRC
        for (size_t i = 0; i < length; ++i) {
            crc ^= (uint16_t)payload[i] << 8;
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
    }

    // Write full frame to radio
    _radio.write((const uint8_t*)&header, sizeof(header));
    if (length > 0 && payload != nullptr) {
        _radio.write(payload, length);
    }
    _radio.write((const uint8_t*)&crc, sizeof(crc));
    _radio.flush();

    _stats.packets_sent++;
    return true;
}

void TdmEngine::processIncomingRadioData() {
    while (_radio.available() > 0) {
        uint8_t b = (uint8_t)_radio.read();

        switch (_rxState) {
            case RxState::WAIT_MAGIC0:
                if (b == DUAL_LRS_MAGIC_0) {
                    _rxState = RxState::WAIT_MAGIC1;
                }
                break;

            case RxState::WAIT_MAGIC1:
                if (b == DUAL_LRS_MAGIC_1) {
                    _rxHeader.magic0 = DUAL_LRS_MAGIC_0;
                    _rxHeader.magic1 = DUAL_LRS_MAGIC_1;
                    _rxBytesCount = 0;
                    _rxState = RxState::WAIT_HEADER;
                } else {
                    _rxState = RxState::WAIT_MAGIC0;
                }
                break;

            case RxState::WAIT_HEADER:
                ((uint8_t*)&_rxHeader)[2 + _rxBytesCount] = b;
                _rxBytesCount++;
                if (_rxBytesCount == sizeof(LrsFrameHeader) - 2) {
                    if (_rxHeader.payload_len > MAX_PAYLOAD_PER_SLOT) {
                        // Invalid length, drop frame
                        _rxState = RxState::WAIT_MAGIC0;
                    } else if (_rxHeader.payload_len == 0) {
                        _rxBytesCount = 0;
                        _rxState = RxState::WAIT_CRC;
                    } else {
                        _rxBytesCount = 0;
                        _rxState = RxState::WAIT_PAYLOAD;
                    }
                }
                break;

            case RxState::WAIT_PAYLOAD:
                _rxBuffer[_rxBytesCount++] = b;
                if (_rxBytesCount == _rxHeader.payload_len) {
                    _rxBytesCount = 0;
                    _rxState = RxState::WAIT_CRC;
                }
                break;

            case RxState::WAIT_CRC:
                ((uint8_t*)&_rxCrc)[_rxBytesCount++] = b;
                if (_rxBytesCount == sizeof(uint16_t)) {
                    // Verify CRC
                    uint16_t calcCrc = calculateCrc16((const uint8_t*)&_rxHeader, sizeof(_rxHeader));
                    if (_rxHeader.payload_len > 0) {
                        for (size_t i = 0; i < _rxHeader.payload_len; ++i) {
                            calcCrc ^= (uint16_t)_rxBuffer[i] << 8;
                            for (uint8_t j = 0; j < 8; ++j) {
                                if (calcCrc & 0x8000) calcCrc = (calcCrc << 1) ^ 0x1021;
                                else calcCrc <<= 1;
                            }
                        }
                    }

                    if (calcCrc == _rxCrc) {
                        // Frame is valid!
                        _stats.packets_received++;
                        _stats.last_sync_ms = millis();
                        _stats.synchronized = true;

                        // Synchronize Air node TDM frame
                        if (_role == NodeRole::AIR) {
                            // Ground slot occurs at (TDM_AIR_SLOT_MS + TDM_GUARD_GAP1_MS)
                            uint32_t expectedGroundOffsetUs = (TDM_AIR_SLOT_MS + TDM_GUARD_GAP1_MS) * 1000;
                            uint32_t nowUs = micros();
                            if (nowUs >= expectedGroundOffsetUs) {
                                _frameStartTimeUs = nowUs - expectedGroundOffsetUs;
                            }
                        }

                        // Check sequence loss
                        uint8_t expectedSeq = _stats.last_rx_seq + 1;
                        if (_rxHeader.seq_num != expectedSeq && _stats.packets_received > 1) {
                            uint8_t dropped = _rxHeader.seq_num - expectedSeq;
                            _stats.packets_dropped += dropped;
                        }
                        _stats.last_rx_seq = _rxHeader.seq_num;

                        // Dispatch callback
                        if (_rxCallback != nullptr) {
                            _rxCallback(static_cast<LrsPacketType>(_rxHeader.packet_type),
                                        _rxBuffer,
                                        _rxHeader.payload_len);
                        }
                    } else {
                        _stats.packets_dropped++;
                    }

                    _rxState = RxState::WAIT_MAGIC0;
                }
                break;
        }
    }
}
