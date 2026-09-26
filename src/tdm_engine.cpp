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
    uint32_t framePeriodUs = TDM_FRAME_PERIOD_MS * 1000;

    // Advance frame start time in 50ms periods
    while ((nowUs - _frameStartTimeUs) >= framePeriodUs) {
        _frameStartTimeUs += framePeriodUs;
    }

    uint32_t elapsedUs = nowUs - _frameStartTimeUs;

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
    if (_role == NodeRole::AIR) {
        // Air unit only transmits if it is synchronized to Ground master.
        // When unsynchronized, it listens continuously to prevent jamming Ground beacons.
        if (!_stats.synchronized) {
            return false;
        }
        return (_currentSlot == TdmSlot::AIR_TRANSMIT);
    } else {
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
    header.seq_num = _txSeqNum;
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

    // Role-aware AUX busy handling:
    // GROUND: Air packet arrives at Ground E22 at ~34.8ms (30.0ms Air tx + 4.1ms UART out).
    //         Ground slot starts at 35ms. Wait up to 5ms (until 40ms) for Ground AUX to clear.
    //         11-byte frame takes 8.5ms, completing by 48.5ms with 1.5ms margin before 50ms wrap.
    // AIR:    If AUX is LOW when Air's slot starts, timing is wrong — skip rather than corrupt.
    if (_radio.isBusy()) {
        if (_role == NodeRole::GROUND) {
            if (!_radio.waitForReady(5)) {
                return false; // Still busy after 5ms — skip this ground slot to guarantee no collision with 50ms wrap
            }
        } else {
            if (!_radio.waitForReady(3)) {
                return false; // Still busy after 3ms — skip this air slot
            }
        }
    }

    // Write full frame to radio (non-blocking UART DMA/FIFO handles transmission)
    _radio.write((const uint8_t*)&header, sizeof(header));
    if (length > 0 && payload != nullptr) {
        _radio.write(payload, length);
    }
    _radio.write((const uint8_t*)&crc, sizeof(crc));
    // NOTE: No flush() — blocking flush() wastes 0.5-3.4ms per slot stalling
    // the main loop. UART TX FIFO is hardware-driven; no explicit flush needed.

    _txSeqNum++;
    _stats.packets_sent++;
    return true;
}

void TdmEngine::processIncomingRadioData() {
    uint32_t nowMs = millis();
    // Inter-byte timeout: if > 15ms elapsed since last radio byte and frame is incomplete, reset to WAIT_MAGIC0
    if (nowMs - _lastRxByteMs > 15 && _rxState != RxState::WAIT_MAGIC0) {
        _rxState = RxState::WAIT_MAGIC0;
    }

    while (_radio.available() > 0) {
        _lastRxByteMs = millis();
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
                } else if (b == DUAL_LRS_MAGIC_0) {
                    // Consecutive magic0 bytes (e.g. 0x44 0x44 0x4C) - stay in WAIT_MAGIC1
                    _rxState = RxState::WAIT_MAGIC1;
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
                        bool wasSync = _stats.synchronized;
                        _stats.packets_received++;
                        _stats.last_sync_ms = millis();
                        _stats.synchronized = true;

                        // Synchronize Air node TDM frame from Ground packets (both HEARTBEAT_SYNC and MAVLINK_DATA).
                        // Ground packets are strictly <= MAX_PAYLOAD_GROUND_SLOT (4 bytes).
                        // Transit delay formula linearly compensates for payload airtime and UART time:
                        // 0B beacon: 10200us. Each byte adds ~215us (UART TX/RX @ 115200 + LoRa airtime @ 62.5k).
                        if (_role == NodeRole::AIR &&
                            (static_cast<LrsPacketType>(_rxHeader.packet_type) == LrsPacketType::HEARTBEAT_SYNC ||
                             static_cast<LrsPacketType>(_rxHeader.packet_type) == LrsPacketType::MAVLINK_DATA) &&
                            _rxHeader.payload_len <= MAX_PAYLOAD_GROUND_SLOT) {
                            const uint32_t transitDelayUs = 10200 + ((uint32_t)_rxHeader.payload_len * 215);
                            uint32_t expectedArrivalUs = ((TDM_AIR_SLOT_MS + TDM_GUARD_GAP1_MS) * 1000) + transitDelayUs;
                            uint32_t nowUs = micros();
                            if (nowUs >= expectedArrivalUs) {
                                uint32_t targetStartUs = nowUs - expectedArrivalUs;
                                if (!wasSync) {
                                    _frameStartTimeUs = targetStartUs;
                                } else {
                                    // Graduated PLL slew:
                                    // - Large error (>5ms): fast correction ±200us to converge in ~1 second
                                    // - Medium error (>2ms): ±50us for ~2 second convergence
                                    // - Small error (>500us): ±20us fine approach
                                    // - Near lock (<500us): ±5us micro-adjust (crystal drift compensation)
                                    // - Deadband <250us: no adjustment (rejects loop/UART jitter)
                                    int32_t err = (int32_t)(targetStartUs - _frameStartTimeUs);
                                    while (err > 25000) err -= 50000;
                                    while (err < -25000) err += 50000;
                                    _stats.last_arrival_us = (uint32_t)(nowUs - _frameStartTimeUs);
                                    _stats.last_pll_err = err;
                                    int32_t absErr = (err < 0) ? -err : err;
                                    if (absErr > 3000) {
                                        _frameStartTimeUs += err; // Fast snap using wrapped error: instantly locks phase in 1 step
                                    } else {
                                        int32_t step = 0;
                                        if      (absErr > 1000) step = 100;
                                        else if (absErr > 500)  step = 25;
                                        else if (absErr > 250)  step = 5;
                                        if (err > 0) _frameStartTimeUs += step;
                                        else if (err < 0) _frameStartTimeUs -= step;
                                    }
                                }
                            }
                        }

                        // Check sequence loss
                        uint8_t seqDiff = (uint8_t)(_rxHeader.seq_num - _stats.last_rx_seq);
                        if (seqDiff > 1 && seqDiff < 100 && _stats.packets_received > 1) {
                            _stats.packets_dropped += (seqDiff - 1);
                            _stats.seq_drops += (seqDiff - 1);
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
                        _stats.crc_errors++;
                    }

                    _rxState = RxState::WAIT_MAGIC0;
                }
                break;
        }
    }
}
