#pragma once
#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"
#include "transport_protocol.h"
#include "transport_engine.h"

enum class NodeRole : uint8_t {
    AIR = 0,
    GROUND = 1
};

// Approved Phase 2 Bench Schedule: 90ms total cycle (~11.1 Hz)
enum class TdmSlot : uint8_t {
    GROUND_TRANSMIT = 0, // Slot 1: Ground node transmitting RC / commands (0 .. 32ms)
    GUARD_GAP_1     = 1, // Turnaround gap 1 (32 .. 37ms)
    AIR_TRANSMIT    = 2, // Slot 2: Air node transmitting telemetry (37 .. 82ms)
    GUARD_GAP_2     = 3  // Turnaround gap 2 (82 .. 90ms)
};

// Legacy packet types preserved for backwards compatibility with existing callers
enum class LrsPacketType : uint8_t {
    HEARTBEAT_SYNC = 0x01,
    MAVLINK_DATA   = 0x02,
    RC_OVERRIDE    = 0x03,
    RADIO_STATUS   = 0x04
};

// Legacy 5-byte header kept for compatibility where referenced, wire uses TransportHeader
struct __attribute__((packed)) LrsFrameHeader {
    uint8_t magic0;       // DUAL_LRS_MAGIC_0 ('D')
    uint8_t magic1;       // DUAL_LRS_MAGIC_1 ('L')
    uint8_t packet_type;  // LrsPacketType
    uint8_t seq_num;      // Sequence number (0-255)
    uint8_t payload_len;  // Length of following payload
};

struct LinkStats {
    uint32_t packets_sent = 0;
    uint32_t packets_received = 0;
    uint32_t packets_dropped = 0;
    uint32_t crc_errors = 0;
    uint32_t seq_drops = 0;
    uint8_t  last_rx_seq = 0;
    uint8_t  link_quality = 100; // 0-100%
    uint32_t last_sync_ms = 0;
    bool     synchronized = false;
    uint32_t last_arrival_us = 0;
    int32_t  last_pll_err = 0;
};

class TdmEngine {
public:
    TdmEngine(E22Driver& radio, NodeRole role);

    void begin();
    void update();

    // Check if the current node is allowed to transmit right now
    bool canTransmit() const;

    // Send payload wrapped in Dual-LRS Phase 2 frame (bridges LrsPacketType callers)
    bool sendPacket(LrsPacketType type, const uint8_t* payload, uint8_t length);

    // Send native Transport frame directly
    bool sendTransportFrame(TransportChannel channel, uint8_t flags, uint16_t transfer_id,
                           uint16_t frag_offset, const uint8_t* payload, uint16_t length);

    // Send pre-encoded native Transport wire frame directly (<= 64 bytes)
    bool sendRawTransportFrame(const uint8_t* frame, size_t length);

    uint16_t getNextSequence() const { return _txSeqNum; }

    // Callback when payload is successfully received (legacy compatibility)
    using RxCallback = void (*)(LrsPacketType type, const uint8_t* payload, uint8_t length);
    void onPacketReceived(RxCallback cb) { _rxCallback = cb; }

    // Native Transport frame callback
    using TransportRxCallback = void (*)(const TransportHeader& hdr, const uint8_t* payload, uint16_t length);
    void onTransportFrameReceived(TransportRxCallback cb) { _transportRxCallback = cb; }

    const LinkStats& getStats() const { return _stats; }
    TdmSlot getCurrentSlot() const { return _currentSlot; }
    TransportParser& getParser() { return _parser; }

    static uint16_t calculateCrc16(const uint8_t* data, size_t length);

    // Helpers for deterministic testing without hardware
    void setFrameStartTimeUs(uint32_t t) { _frameStartTimeUs = t; }
    uint32_t getFrameStartTimeUs() const { return _frameStartTimeUs; }
    void setSynchronized(bool s) { _stats.synchronized = s; }
    void forceSlot(TdmSlot slot) { _currentSlot = slot; }

private:
    void processIncomingRadioData();
    void updateSlotState();

    E22Driver& _radio;
    NodeRole _role;
    TdmSlot _currentSlot;
    uint32_t _frameStartTimeUs;
    uint16_t _txSeqNum;

    TransportParser _parser;
    RxCallback _rxCallback;
    TransportRxCallback _transportRxCallback;

    LinkStats _stats;
};
