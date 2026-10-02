#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// =============================================================================
// DUAL-LRS PHASE 2 TRANSPORT PROTOCOL DEFINITION
// Grounded in Phase 1 E22 Physical Link Measurements
// Strictly Enforces Single-Burst Framing (<= 64 Bytes Total) for Phase 2 v1
// =============================================================================

#define TRANSPORT_MAGIC0              0x44 // 'D'
#define TRANSPORT_MAGIC1              0x4C // 'L'
#define TRANSPORT_VERSION             0x01

// Hardware sub-packet boundary constants (derived from Phase 1 empirical measurements)
#define TRANSPORT_E22_SUBPACKET_LIMIT 64   // E22 hardware sub-packet boundary (REG1=0x83)
#define TRANSPORT_HEADER_SIZE         13   // Fixed size of TransportHeader in bytes
#define TRANSPORT_CRC_SIZE            2    // CRC-16-CCITT trailing checksum
#define TRANSPORT_OVERHEAD_SIZE       (TRANSPORT_HEADER_SIZE + TRANSPORT_CRC_SIZE) // 15 Bytes

// Maximum payload that guarantees a single-burst RF packet without modem splitting:
// 64 (E22 boundary) - 13 (Header) - 2 (CRC) = 49 Bytes
#define TRANSPORT_MAX_SINGLE_BURST_PAYLOAD 49
#define TRANSPORT_MAX_SINGLE_BURST_FRAME   64

// Phase 2 v1 strictly enforces maximum frame size equal to the single-burst limit:
// Split-frame transport (>=65 bytes) is deferred to future experimental studies.
#define TRANSPORT_MAX_FRAME_SIZE      64

// Reassembly & Reliability Bounds
#define TRANSPORT_MAX_TRANSFER_SIZE   512  // Maximum reassembly buffer size in bytes
#define TRANSPORT_MAX_FRAGMENTS       11   // Max fragments per transfer: ceil(512 / 49) = 11
#define TRANSPORT_TRANSFER_TIMEOUT_MS 1000 // Inactivity timeout for ongoing reassembly (ms)
#define TRANSPORT_RETRY_TIMEOUT_MS    200  // Retransmission timeout (2 * 90ms TDM cycle + 20ms margin)
#define TRANSPORT_MAX_RETRIES         5    // Maximum retransmission retry attempts

// RC Failsafe Timing Constants (Decoupled from Flight Controller policy)
#define TRANSPORT_RC_FAILSAFE_TIMEOUT_MS   500 // Loss of RC frame timeout before asserting failsafe flag
#define TRANSPORT_RC_RESTORE_FRAME_COUNT   3   // Consecutive valid RC frames required to clear failsafe

// =============================================================================
// CHANNEL IDENTIFIERS
// =============================================================================
enum class TransportChannel : uint8_t {
    RC_CONTROL        = 0x01, // High-rate, unfragmented, packed RC (Ground -> Air)
    LINK_CONTROL      = 0x02, // ACKs, NACKs, Sync, Heartbeat, Link Stats (Bidirectional)
    MAVLINK_UPLINK    = 0x03, // MAVLink commands, mission uploads, params (Ground -> Air)
    MAVLINK_DOWNLINK  = 0x04  // MAVLink telemetry, mission acks, param streams (Air -> Ground)
};

// Node role for per-channel direction validation
enum class TransportNodeRole : uint8_t {
    GROUND = 0x01,
    AIR    = 0x02
};

// =============================================================================
// CONTROL FLAGS BITMASK
// =============================================================================
#define TRANSPORT_FLAG_RELIABLE       (1 << 0) // Requires ACK from receiver
#define TRANSPORT_FLAG_IS_ACK         (1 << 1) // Frame carries ACK payload
#define TRANSPORT_FLAG_IS_NACK        (1 << 2) // Frame carries NACK payload
#define TRANSPORT_FLAG_FIRST_FRAG     (1 << 3) // First fragment of segmented transfer
#define TRANSPORT_FLAG_LAST_FRAG      (1 << 4) // Final fragment of segmented transfer

// =============================================================================
// TRANSPORT HEADER SPECIFICATION (13 BYTES EXACT)
// Byte Offsets (Wire Representation, Big-Endian):
//   0:  magic0          ('D' = 0x44)
//   1:  magic1          ('L' = 0x4C)
//   2:  version         (0x01)
//   3:  channel         (TransportChannel enum)
//   4:  flags           (Bitmask of TRANSPORT_FLAG_*)
//   5:  sequence (MSB)  (Hop-by-hop sequence number 0-65535)
//   6:  sequence (LSB)
//   7:  transfer_id (MSB) (Transaction ID for segmented transfers)
//   8:  transfer_id (LSB)
//   9:  fragment_offset (MSB) (Byte offset within logical message)
//  10:  fragment_offset (LSB)
//  11:  payload_length (MSB)  (Payload bytes in this frame: 0 - 49)
//  12:  payload_length (LSB)
// Trailing CRC:
//  13+payload_length:     CRC16 (MSB)
//  13+payload_length + 1: CRC16 (LSB)
// =============================================================================
#pragma pack(push, 1)
struct TransportHeader {
    uint8_t  magic0;          // Byte 0:  'D'
    uint8_t  magic1;          // Byte 1:  'L'
    uint8_t  version;         // Byte 2:  Protocol version (0x01)
    uint8_t  channel;         // Byte 3:  TransportChannel
    uint8_t  flags;           // Byte 4:  Control flags
    uint16_t sequence;        // Bytes 5-6:   Sequence number (Host-Endian in memory, Big-Endian on wire)
    uint16_t transfer_id;     // Bytes 7-8:   Segmented message identifier (Host-Endian in memory, Big-Endian on wire)
    uint16_t fragment_offset; // Bytes 9-10:  Byte offset within message (Host-Endian in memory, Big-Endian on wire)
    uint16_t payload_length;  // Bytes 11-12: Attached payload size in bytes (Host-Endian in memory, Big-Endian on wire)
};
#pragma pack(pop)

static_assert(sizeof(TransportHeader) == 13, "TransportHeader must be exactly 13 bytes");

// =============================================================================
// SUB-PROTOCOL PAYLOAD FORMATS
// =============================================================================

// 1. RC_CONTROL: Custom Packed RC Payload Format (24 Bytes)
// Note: This is a custom packed RF format, NOT a raw CRSF-framed packet over the air.
// Arithmetic: 13 (Header) + 24 (RC) + 2 (CRC) = 39 Bytes <= 64 Bytes (Single burst)
#define TRANSPORT_RC_FLAG_FAILSAFE   (1 << 0) // Transmitter indicates failsafe condition
#define TRANSPORT_RC_FLAG_FRAME_LOST (1 << 1) // Transmitter indicates uplink frame loss

#define TRANSPORT_RC_NUM_CHANNELS    16
#define TRANSPORT_RC_CHANNEL_MIN     0
#define TRANSPORT_RC_CHANNEL_MAX     2047

#pragma pack(push, 1)
struct TransportPackedRc {
    // 16 channels packed into 22 bytes (11 bits per channel, 0 - 2047)
    // 16 * 11 = 176 bits = 22 bytes
    uint8_t  channels[22];    // Bytes 0-21:  Packed 16x 11-bit RC channels
    uint8_t  rc_sequence;     // Byte 22:     Rolling 8-bit RC sequence counter
    uint8_t  flags;           // Byte 23:     Bit 0: Failsafe, Bit 1: Lost, Bits [7:2]: Reserved
};
#pragma pack(pop)

static_assert(sizeof(TransportPackedRc) == 24, "TransportPackedRc must be exactly 24 bytes");
typedef TransportPackedRc TransportRcFrame; // Backward compatibility alias

// 2. LINK_CONTROL: Heartbeat & Link Statistics Payload (6 Bytes)
// Arithmetic: 13 (Header) + 6 (Heartbeat) + 2 (CRC) = 21 Bytes <= 64 Bytes (Single burst)
enum class LinkSubtype : uint8_t {
    HEARTBEAT = 0x01,
    ACK_NACK  = 0x02,
    SYNC_REQ  = 0x03,
    SYNC_RESP = 0x04
};

#pragma pack(push, 1)
struct TransportLinkHeartbeat {
    uint8_t  subtype;      // Byte 0:    LinkSubtype::HEARTBEAT (0x01)
    uint8_t  link_quality; // Byte 1:    0 - 100%
    int8_t   rssi;         // Byte 2:    RSSI in dBm
    int8_t   snr;          // Byte 3:    SNR in dB
    uint16_t rtt_ms;       // Bytes 4-5: Measured single-clock RTT (Host-Endian in memory, Big-Endian on wire)
};
#pragma pack(pop)

static_assert(sizeof(TransportLinkHeartbeat) == 6, "TransportLinkHeartbeat must be exactly 6 bytes");

// 3. LINK_CONTROL: Selective ACK/NACK Payload (8 Bytes)
// Arithmetic: 13 (Header) + 8 (ACK/NACK) + 2 (CRC) = 23 Bytes <= 64 Bytes (Single burst)
enum class TransportNackReason : uint8_t {
    NONE             = 0x00, // Valid ACK (no error)
    BAD_CRC          = 0x01, // Frame CRC failure detected by link
    GAP_DETECTED     = 0x02, // Fragment gap observed (missing fragment / non-first fragment with no active transfer)
    BUFFER_FULL      = 0x03, // Reassembly buffer already occupied by ongoing transfer
    OFFSET_OVERRUN   = 0x04, // Fragment offset exceeds TRANSPORT_MAX_TRANSFER_SIZE (512B)
    TRANSFER_TIMEOUT = 0x05, // Inactivity timeout reached before reassembly finished
    OVERLAP_CONFLICT = 0x06  // Partial range overlap detected with previously received fragment
};

#pragma pack(push, 1)
struct TransportLinkAckNack {
    uint8_t  subtype;         // Byte 0:    LinkSubtype::ACK_NACK (0x02)
    uint8_t  target_channel;  // Byte 1:    TransportChannel of acknowledged stream
    uint16_t transfer_id;     // Bytes 2-3: Transaction ID being acknowledged (Host-Endian in memory, Big-Endian on wire)
    uint16_t fragment_offset; // Bytes 4-5: Specific fragment byte offset (Host-Endian in memory, Big-Endian on wire)
    uint8_t  nack_reason;     // Byte 6:    TransportNackReason (0x00 = ACK, >0 = NACK reason code)
    uint8_t  reserved;        // Byte 7:    Reserved padding byte (0x00)
};
#pragma pack(pop)

static_assert(sizeof(TransportLinkAckNack) == 8, "TransportLinkAckNack must be exactly 8 bytes");

// =============================================================================
// EXPLICIT BIG-ENDIAN WIRE SERIALIZATION HELPERS
// =============================================================================

// Write 16-bit unsigned integer to wire buffer in Big-Endian order
static inline void transport_write_u16_be(uint8_t* dest, uint16_t val) {
    dest[0] = (uint8_t)((val >> 8) & 0xFF);
    dest[1] = (uint8_t)(val & 0xFF);
}

// Read 16-bit unsigned integer from wire buffer in Big-Endian order
static inline uint16_t transport_read_u16_be(const uint8_t* src) {
    return (uint16_t)(((uint16_t)src[0] << 8) | (uint16_t)src[1]);
}

// Encodes TransportHeader to exact 13-byte wire buffer
static inline void transport_encode_header(const TransportHeader* hdr, uint8_t* out_bytes) {
    out_bytes[0] = hdr->magic0;
    out_bytes[1] = hdr->magic1;
    out_bytes[2] = hdr->version;
    out_bytes[3] = hdr->channel;
    out_bytes[4] = hdr->flags;
    transport_write_u16_be(&out_bytes[5], hdr->sequence);
    transport_write_u16_be(&out_bytes[7], hdr->transfer_id);
    transport_write_u16_be(&out_bytes[9], hdr->fragment_offset);
    transport_write_u16_be(&out_bytes[11], hdr->payload_length);
}

// Decodes exact 13-byte wire buffer to TransportHeader struct
static inline bool transport_decode_header(const uint8_t* in_bytes, TransportHeader* hdr) {
    if (!in_bytes || !hdr) return false;
    hdr->magic0 = in_bytes[0];
    hdr->magic1 = in_bytes[1];
    hdr->version = in_bytes[2];
    hdr->channel = in_bytes[3];
    hdr->flags = in_bytes[4];
    hdr->sequence = transport_read_u16_be(&in_bytes[5]);
    hdr->transfer_id = transport_read_u16_be(&in_bytes[7]);
    hdr->fragment_offset = transport_read_u16_be(&in_bytes[9]);
    hdr->payload_length = transport_read_u16_be(&in_bytes[11]);
    return true;
}

// Encodes TransportLinkHeartbeat to exact 6-byte wire buffer
static inline void transport_encode_link_heartbeat(const TransportLinkHeartbeat* hb, uint8_t* out_bytes) {
    out_bytes[0] = hb->subtype;
    out_bytes[1] = hb->link_quality;
    out_bytes[2] = (uint8_t)hb->rssi;
    out_bytes[3] = (uint8_t)hb->snr;
    transport_write_u16_be(&out_bytes[4], hb->rtt_ms);
}

// Decodes exact 6-byte wire buffer to TransportLinkHeartbeat struct
static inline bool transport_decode_link_heartbeat(const uint8_t* in_bytes, TransportLinkHeartbeat* hb) {
    if (!in_bytes || !hb) return false;
    hb->subtype = in_bytes[0];
    hb->link_quality = in_bytes[1];
    hb->rssi = (int8_t)in_bytes[2];
    hb->snr = (int8_t)in_bytes[3];
    hb->rtt_ms = transport_read_u16_be(&in_bytes[4]);
    return true;
}

// Encodes TransportLinkAckNack to exact 8-byte wire buffer
static inline void transport_encode_link_ack_nack(const TransportLinkAckNack* an, uint8_t* out_bytes) {
    out_bytes[0] = an->subtype;
    out_bytes[1] = an->target_channel;
    transport_write_u16_be(&out_bytes[2], an->transfer_id);
    transport_write_u16_be(&out_bytes[4], an->fragment_offset);
    out_bytes[6] = an->nack_reason;
    out_bytes[7] = an->reserved;
}

// Decodes exact 8-byte wire buffer to TransportLinkAckNack struct
static inline bool transport_decode_link_ack_nack(const uint8_t* in_bytes, TransportLinkAckNack* an) {
    if (!in_bytes || !an) return false;
    an->subtype = in_bytes[0];
    an->target_channel = in_bytes[1];
    an->transfer_id = transport_read_u16_be(&in_bytes[2]);
    an->fragment_offset = transport_read_u16_be(&in_bytes[4]);
    an->nack_reason = in_bytes[6];
    an->reserved = in_bytes[7];
    return true;
}

// =============================================================================
// EXACT RC BIT-PACKING SPECIFICATION & ALGORITHM
//
// Rules:
// 1. Channel 0 starts at bit 0 of byte 0.
// 2. Channels are packed consecutively in little-endian bit order.
// 3. Each channel uses exactly 11 bits (range 0..2047).
// 4. API Input Contract:
//    - The C++ API accepts `const uint16_t* channels`. Channel values are unsigned 16-bit
//      integers already normalized to unsigned representation [0..2047] by the RC input
//      driver (e.g., CRSF / SBUS decoder). Negative values cannot reach this unsigned API.
//    - Out-of-range behavior for unsigned values > 2047:
//      * If clamp == true: values > 2047 are clamped to 2047.
//      * If clamp == false: values > 2047 cause the function to return false (rejection).
// =============================================================================

// Packs 16x 11-bit channels into 22 bytes
static inline bool transport_pack_rc_channels(const uint16_t* channels, uint8_t* out_22bytes, bool clamp = true) {
    if (!channels || !out_22bytes) return false;

    for (size_t b = 0; b < 22; ++b) {
        out_22bytes[b] = 0;
    }

    uint32_t bit_buf = 0;
    uint8_t bits_in_buf = 0;
    size_t byte_idx = 0;

    for (size_t ch = 0; ch < TRANSPORT_RC_NUM_CHANNELS; ++ch) {
        uint16_t val = channels[ch];
        if (val > TRANSPORT_RC_CHANNEL_MAX) {
            if (!clamp) return false;
            val = TRANSPORT_RC_CHANNEL_MAX;
        }
        bit_buf |= ((uint32_t)val << bits_in_buf);
        bits_in_buf += 11;
        while (bits_in_buf >= 8) {
            out_22bytes[byte_idx++] = (uint8_t)(bit_buf & 0xFF);
            bit_buf >>= 8;
            bits_in_buf -= 8;
        }
    }
    if (bits_in_buf > 0 && byte_idx < 22) {
        out_22bytes[byte_idx++] = (uint8_t)(bit_buf & 0xFF);
    }
    return (byte_idx == 22);
}

// Unpacks 22 bytes into 16x 11-bit channels
static inline bool transport_unpack_rc_channels(const uint8_t* in_22bytes, uint16_t* out_channels) {
    if (!in_22bytes || !out_channels) return false;

    uint32_t bit_buf = 0;
    uint8_t bits_in_buf = 0;
    size_t byte_idx = 0;

    for (size_t ch = 0; ch < TRANSPORT_RC_NUM_CHANNELS; ++ch) {
        while (bits_in_buf < 11 && byte_idx < 22) {
            bit_buf |= ((uint32_t)in_22bytes[byte_idx++] << bits_in_buf);
            bits_in_buf += 8;
        }
        if (bits_in_buf < 11) {
            return false;
        }
        out_channels[ch] = (uint16_t)(bit_buf & 0x07FF);
        bit_buf >>= 11;
        bits_in_buf -= 11;
    }
    return true;
}

// Encodes entire 24-byte TransportPackedRc wire payload
static inline bool transport_encode_packed_rc(const uint16_t* channels, uint8_t rc_sequence, uint8_t flags, uint8_t* out_24bytes, bool clamp = true) {
    if (!transport_pack_rc_channels(channels, out_24bytes, clamp)) return false;
    out_24bytes[22] = rc_sequence;
    out_24bytes[23] = flags;
    return true;
}

// Decodes entire 24-byte TransportPackedRc wire payload
static inline bool transport_decode_packed_rc(const uint8_t* in_24bytes, uint16_t* out_channels, uint8_t* out_rc_sequence, uint8_t* out_flags) {
    if (!in_24bytes || !out_channels) return false;
    if (!transport_unpack_rc_channels(in_24bytes, out_channels)) return false;
    if (out_rc_sequence) *out_rc_sequence = in_24bytes[22];
    if (out_flags) *out_flags = in_24bytes[23];
    return true;
}

// =============================================================================
// PROTOCOL UTILITIES & VALIDATION
// =============================================================================

// Standard CRC-16-CCITT (polynomial 0x1021, initial value 0xFFFF)
static inline uint16_t transport_crc16(const uint8_t* data, size_t length, uint16_t init_crc = 0xFFFF) {
    uint16_t crc = init_crc;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// Channel validity check
static inline bool transport_is_valid_channel(uint8_t ch) {
    return (ch == (uint8_t)TransportChannel::RC_CONTROL ||
            ch == (uint8_t)TransportChannel::LINK_CONTROL ||
            ch == (uint8_t)TransportChannel::MAVLINK_UPLINK ||
            ch == (uint8_t)TransportChannel::MAVLINK_DOWNLINK);
}

// Per-channel transmission direction check
static inline bool transport_is_valid_direction(TransportNodeRole receiver_role, uint8_t channel) {
    if (receiver_role == TransportNodeRole::AIR) {
        // Air unit receives Uplink channels from Ground
        return (channel == (uint8_t)TransportChannel::RC_CONTROL ||
                channel == (uint8_t)TransportChannel::LINK_CONTROL ||
                channel == (uint8_t)TransportChannel::MAVLINK_UPLINK);
    } else if (receiver_role == TransportNodeRole::GROUND) {
        // Ground unit receives Downlink channels from Air
        return (channel == (uint8_t)TransportChannel::LINK_CONTROL ||
                channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK);
    }
    return false;
}

// NACK reason validity check
static inline bool transport_is_valid_nack_reason(uint8_t reason) {
    return (reason <= (uint8_t)TransportNackReason::OVERLAP_CONFLICT);
}

// Single-burst boundary check
static inline bool transport_is_single_burst(uint16_t payload_len) {
    return (TRANSPORT_HEADER_SIZE + payload_len + TRANSPORT_CRC_SIZE <= TRANSPORT_E22_SUBPACKET_LIMIT);
}

// Total frame size calculator
static inline size_t transport_calculate_frame_size(uint16_t payload_len) {
    return (size_t)(TRANSPORT_HEADER_SIZE + payload_len + TRANSPORT_CRC_SIZE);
}

// Sequence distance with 16-bit wraparound handling
static inline int16_t transport_sequence_diff(uint16_t seq_received, uint16_t seq_expected) {
    return (int16_t)(seq_received - seq_expected);
}
