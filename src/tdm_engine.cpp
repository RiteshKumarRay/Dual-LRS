#include "tdm_engine.h"

// Standard CRC-16-CCITT (Polynomial 0x1021, Initial 0xFFFF)
uint16_t TdmEngine::calculateCrc16(const uint8_t* data, size_t length) {
    return transport_crc16(data, length);
}

TdmEngine::TdmEngine(E22Driver& radio, NodeRole role)
    : _radio(radio),
      _role(role),
      _currentSlot(TdmSlot::GROUND_TRANSMIT),
      _frameStartTimeUs(0),
      _txSeqNum(0),
      _parser(role == NodeRole::AIR ? TransportNodeRole::AIR : TransportNodeRole::GROUND),
      _rxCallback(nullptr),
      _transportRxCallback(nullptr),
      _stats{} {}

void TdmEngine::begin() {
    _radio.begin();
    _frameStartTimeUs = micros();
    _stats.last_sync_ms = millis();
    _stats.synchronized = (_role == NodeRole::GROUND); // Ground starts as timing master
    _parser.reset();
}

void TdmEngine::update() {
    updateSlotState();
    processIncomingRadioData();
}

void TdmEngine::updateSlotState() {
    uint32_t nowUs = micros();
    uint32_t framePeriodUs = TDM_FRAME_PERIOD_MS * 1000; // 90,000 us

    // Advance frame start time in 90ms periods
    while ((nowUs - _frameStartTimeUs) >= framePeriodUs) {
        _frameStartTimeUs += framePeriodUs;
    }

    uint32_t elapsedUs = nowUs - _frameStartTimeUs;

    // Check for sync timeout on Air unit (1.5 seconds without Ground beacon)
    if (_role == NodeRole::AIR) {
        if (millis() - _stats.last_sync_ms > 1500) {
            _stats.synchronized = false;
        }
    }

    // Approved Bench Schedule (90.0 ms Total Cycle):
    // Slot 1: Ground Uplink (0 .. 32ms)
    // Guard Gap 1:           (32 .. 37ms)
    // Slot 2: Air Downlink  (37 .. 82ms)
    // Guard Gap 2:           (82 .. 90ms)
    uint32_t groundEndUs = TDM_GROUND_SLOT_MS * 1000;                // 32,000 us
    uint32_t guard1EndUs = groundEndUs + (TDM_GUARD_GAP1_MS * 1000); // 37,000 us
    uint32_t airEndUs    = guard1EndUs + (TDM_AIR_SLOT_MS * 1000);    // 82,000 us

    if (elapsedUs < groundEndUs) {
        _currentSlot = TdmSlot::GROUND_TRANSMIT;
    } else if (elapsedUs < guard1EndUs) {
        _currentSlot = TdmSlot::GUARD_GAP_1;
    } else if (elapsedUs < airEndUs) {
        _currentSlot = TdmSlot::AIR_TRANSMIT;
    } else {
        _currentSlot = TdmSlot::GUARD_GAP_2;
    }
}

bool TdmEngine::canTransmit() const {
    if (_role == NodeRole::AIR) {
        // Air unit only transmits if it is synchronized to Ground master and inside Air slot.
        // When unsynchronized, it listens continuously to prevent jamming Ground beacons.
        if (!_stats.synchronized) {
            return false;
        }
        return (_currentSlot == TdmSlot::AIR_TRANSMIT);
    } else {
        // Ground unit only transmits inside Ground uplink slot
        return (_currentSlot == TdmSlot::GROUND_TRANSMIT);
    }
}

bool TdmEngine::sendPacket(LrsPacketType type, const uint8_t* payload, uint8_t length) {
#if defined(DUAL_LRS_STAGE31_RC_ONLY) && (DUAL_LRS_STAGE31_RC_ONLY == 1)
    if (type == LrsPacketType::MAVLINK_DATA) {
        // Enforce hard isolation: MAVLink RF transmission forbidden in Stage 3.1 RC-only mode
        return false;
    }
#endif
    TransportChannel chan;
    uint8_t flags = 0;

    if (type == LrsPacketType::HEARTBEAT_SYNC) {
        chan = TransportChannel::LINK_CONTROL;
    } else if (type == LrsPacketType::RADIO_STATUS) {
        chan = TransportChannel::LINK_CONTROL;
    } else if (type == LrsPacketType::RC_OVERRIDE) {
        chan = TransportChannel::RC_CONTROL;
    } else if (type == LrsPacketType::MAVLINK_DATA) {
        chan = (_role == NodeRole::AIR) ? TransportChannel::MAVLINK_DOWNLINK
                                        : TransportChannel::MAVLINK_UPLINK;
    } else {
        chan = TransportChannel::LINK_CONTROL;
    }

    return sendTransportFrame(chan, flags, 0, 0, payload, length);
}

bool TdmEngine::sendTransportFrame(TransportChannel channel, uint8_t flags, uint16_t transfer_id,
                                   uint16_t frag_offset, const uint8_t* payload, uint16_t length) {
#if defined(DUAL_LRS_STAGE31_RC_ONLY) && (DUAL_LRS_STAGE31_RC_ONLY == 1)
    if (channel == TransportChannel::MAVLINK_DOWNLINK || channel == TransportChannel::MAVLINK_UPLINK) {
        // Enforce hard isolation: MAVLink RF channels forbidden in Stage 3.1 RC-only mode
        return false;
    }
#endif
    if (length > TRANSPORT_MAX_SINGLE_BURST_PAYLOAD) {
        // A single-burst frame cannot carry more than the radio payload budget.
        // Reject rather than silently changing the caller's transport payload.
        return false;
    }
    if (!canTransmit()) {
        return false;
    }

    // Role-aware AUX busy check:
    // Ground: wait up to 5ms for AUX to clear
    // Air: wait up to 2ms
    if (_radio.isBusy()) {
        if (_role == NodeRole::GROUND) {
            if (!_radio.waitForReady(5)) {
                return false; // Skip this slot to avoid spill
            }
        } else {
            if (!_radio.waitForReady(2)) {
                return false; // Skip this slot
            }
        }
    }

    TransportHeader hdr{};
    hdr.magic0 = TRANSPORT_MAGIC0;
    hdr.magic1 = TRANSPORT_MAGIC1;
    hdr.version = TRANSPORT_VERSION;
    hdr.channel = (uint8_t)channel;
    hdr.flags = flags;
    hdr.sequence = _txSeqNum++;
    hdr.transfer_id = transfer_id;
    hdr.fragment_offset = frag_offset;
    hdr.payload_length = length;

    uint8_t wire_buf[TRANSPORT_MAX_FRAME_SIZE];
    transport_encode_header(&hdr, wire_buf);

    if (length > 0 && payload != nullptr) {
        memcpy(&wire_buf[TRANSPORT_HEADER_SIZE], payload, length);
    }

    uint16_t crc = transport_crc16(wire_buf, TRANSPORT_HEADER_SIZE + length);
    transport_write_u16_be(&wire_buf[TRANSPORT_HEADER_SIZE + length], crc);

    size_t total_frame_len = TRANSPORT_HEADER_SIZE + length + TRANSPORT_CRC_SIZE;
    _radio.write(wire_buf, total_frame_len);

    _stats.packets_sent++;
    return true;
}

bool TdmEngine::sendRawTransportFrame(const uint8_t* frame, size_t length) {
#if defined(DUAL_LRS_STAGE31_RC_ONLY) && (DUAL_LRS_STAGE31_RC_ONLY == 1)
    if (length >= TRANSPORT_HEADER_SIZE && frame != nullptr) {
        uint8_t chan = frame[3];
        if (chan == (uint8_t)TransportChannel::MAVLINK_DOWNLINK || chan == (uint8_t)TransportChannel::MAVLINK_UPLINK) {
            // Enforce hard isolation: MAVLink RF channels forbidden in Stage 3.1 RC-only mode
            return false;
        }
    }
#endif
    if (!canTransmit() || frame == nullptr || length == 0 || length > TRANSPORT_MAX_FRAME_SIZE) {
        return false;
    }

    if (_radio.isBusy()) {
        if (_role == NodeRole::GROUND) {
            if (!_radio.waitForReady(5)) {
                return false;
            }
        } else {
            if (!_radio.waitForReady(2)) {
                return false;
            }
        }
    }

    _radio.write(frame, length);
    _txSeqNum++;
    _stats.packets_sent++;
    return true;
}

void TdmEngine::processIncomingRadioData() {
    uint32_t nowMs = millis();

    while (_radio.available() > 0) {
        uint8_t b = (uint8_t)_radio.read();
        if (_parser.feed_byte(b, nowMs)) {
            // Valid Phase 2 frame decoded by TransportParser!
            const TransportHeader& hdr = _parser.get_header();
            const uint8_t* payload = _parser.get_payload();
            uint16_t plen = _parser.get_payload_length();

            bool wasSync = _stats.synchronized;
            _stats.packets_received++;
            _stats.last_sync_ms = nowMs;
            _stats.synchronized = true;

            // Synchronize Air node TDM frame from Ground frame arrival
            if (_role == NodeRole::AIR) {
                // Ground slot starts at t=0.0ms. Transfer time: ~10ms (0B sync) to ~28.5ms (39B RC)
                uint32_t expectedArrivalUs = (plen == 0) ? 10200 : (10200 + plen * 500);
                if (expectedArrivalUs > 32000) expectedArrivalUs = 32000;
                uint32_t nowUs = micros();
                if (nowUs >= expectedArrivalUs) {
                    uint32_t targetStartUs = nowUs - expectedArrivalUs;
                    if (!wasSync) {
                        _frameStartTimeUs = targetStartUs;
                    } else {
                        // Graduated PLL slew for crystal drift compensation
                        int32_t err = (int32_t)(targetStartUs - _frameStartTimeUs);
                        while (err > 45000) err -= 90000;
                        while (err < -45000) err += 90000;
                        _stats.last_arrival_us = (uint32_t)(nowUs - _frameStartTimeUs);
                        _stats.last_pll_err = err;
                        int32_t absErr = (err < 0) ? -err : err;
                        int32_t step = 0;
                        if      (absErr > 3000) step = 250;
                        else if (absErr > 1000) step = 100;
                        else if (absErr > 500)  step = 25;
                        else if (absErr > 250)  step = 5;
                        if (err > 0) _frameStartTimeUs += step;
                        else if (err < 0) _frameStartTimeUs -= step;
                    }
                }
            }

            // Sequence loss tracking
            uint16_t seqDiff = (uint16_t)(hdr.sequence - _stats.last_rx_seq);
            if (seqDiff > 1 && seqDiff < 1000 && _stats.packets_received > 1) {
                _stats.packets_dropped += (seqDiff - 1);
                _stats.seq_drops += (seqDiff - 1);
            }
            _stats.last_rx_seq = (uint8_t)(hdr.sequence & 0xFF);

            uint32_t totalPackets = _stats.packets_received + _stats.packets_dropped;
            if (totalPackets > 0) {
                _stats.link_quality = (uint8_t)(((uint64_t)_stats.packets_received * 100) / totalPackets);
            }

            // Dispatch native Transport callback if registered
            if (_transportRxCallback != nullptr) {
                _transportRxCallback(hdr, payload, plen);
            }

            // Dispatch legacy callback if registered
            if (_rxCallback != nullptr) {
                LrsPacketType legType = LrsPacketType::HEARTBEAT_SYNC;
                if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK ||
                    hdr.channel == (uint8_t)TransportChannel::MAVLINK_UPLINK) {
                    legType = LrsPacketType::MAVLINK_DATA;
                } else if (hdr.channel == (uint8_t)TransportChannel::RC_CONTROL) {
                    legType = LrsPacketType::RC_OVERRIDE;
                } else if (hdr.channel == (uint8_t)TransportChannel::LINK_CONTROL) {
                    legType = (plen == 0) ? LrsPacketType::HEARTBEAT_SYNC : LrsPacketType::RADIO_STATUS;
                }
                _rxCallback(legType, payload, (uint8_t)min((uint16_t)255, plen));
            }
        }
    }
    _stats.crc_errors = _parser.get_stats().crc_errors;
}
