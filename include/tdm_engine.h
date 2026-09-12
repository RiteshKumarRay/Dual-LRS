#pragma once
#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"

enum class NodeRole : uint8_t {
    AIR = 0,
    GROUND = 1
};

enum class TdmSlot : uint8_t {
    AIR_TRANSMIT = 0, // Air node transmitting telemetry
    GUARD_GAP_1  = 1, // Turnaround gap
    GROUND_TRANSMIT = 2, // Ground node transmitting GCS commands / RC
    GUARD_GAP_2  = 3  // Turnaround gap
};

enum class LrsPacketType : uint8_t {
    HEARTBEAT_SYNC = 0x01,
    MAVLINK_DATA   = 0x02,
    RC_OVERRIDE    = 0x03,
    RADIO_STATUS   = 0x04
};

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
    uint8_t  last_rx_seq = 0;
    uint8_t  link_quality = 100; // 0-100%
    uint32_t last_sync_ms = 0;
    bool     synchronized = false;
};

class TdmEngine {
public:
    TdmEngine(E22Driver& radio, NodeRole role);

    void begin();
    void update();

    // Check if the current node is allowed to transmit right now
    bool canTransmit() const;

    // Send payload wrapped in Dual-LRS frame
    bool sendPacket(LrsPacketType type, const uint8_t* payload, uint8_t length);

    // Callback when payload is successfully received
    using RxCallback = void (*)(LrsPacketType type, const uint8_t* payload, uint8_t length);
    void onPacketReceived(RxCallback cb) { _rxCallback = cb; }

    const LinkStats& getStats() const { return _stats; }

    static uint16_t calculateCrc16(const uint8_t* data, size_t length);

private:
    void processIncomingRadioData();
    void updateSlotState();

    E22Driver& _radio;
    NodeRole _role;
    TdmSlot _currentSlot;
    uint32_t _frameStartTimeUs;
    uint8_t _txSeqNum;

    // Rx frame parser state machine
    enum class RxState { WAIT_MAGIC0, WAIT_MAGIC1, WAIT_HEADER, WAIT_PAYLOAD, WAIT_CRC };
    RxState _rxState;
    LrsFrameHeader _rxHeader;
    uint8_t _rxBuffer[MAX_PAYLOAD_PER_SLOT];
    uint8_t _rxBytesCount;
    uint16_t _rxCrc;
    RxCallback _rxCallback;

    LinkStats _stats;
};
