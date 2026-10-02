#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

// =============================================================================
// DUAL-LRS PHASE 1 RAW E22 DIAGNOSTIC PROTOCOL
// Framing format: [RawDiagHeader: 9B] + [Payload: N B] + [CRC-16: 2B]
// Total frame length = N + 11 bytes.
//
// Frame sizes for target payloads:
//   Payload  0B  -> Total frame: 11B
//   Payload 10B  -> Total frame: 21B
//   Payload 30B  -> Total frame: 41B
//   Payload 53B  -> Total frame: 64B (Fits exactly inside 1 E22 64B sub-packet)
//   Payload 54B  -> Total frame: 65B (Forces 2nd E22 sub-packet for the 65th byte)
//   Payload 55B  -> Total frame: 66B (2 E22 sub-packets: 64B + 2B)
// =============================================================================

#define DIAG_MAGIC_0            0x52 // 'R' (Raw)
#define DIAG_MAGIC_1            0x46 // 'F' (Frame)

#define DIAG_DIR_AIR_TO_GROUND  0x01 // Mode A: Air -> Ground simplex
#define DIAG_DIR_GROUND_TO_AIR  0x02 // Mode B: Ground -> Air simplex
#define DIAG_DIR_PING           0x03 // Mode C: Ground -> Air Ping
#define DIAG_DIR_PONG           0x04 // Mode C: Air -> Ground Pong

#define DIAG_HEADER_SIZE        9
#define DIAG_CRC_SIZE           2
#define DIAG_OVERHEAD_SIZE      (DIAG_HEADER_SIZE + DIAG_CRC_SIZE) // 11 bytes
#define DIAG_BUFFER_SIZE        256  // Minimum >= 128 bytes to handle modem burst/split

struct __attribute__((packed)) RawDiagHeader {
    uint8_t  magic0;          // 0x52 ('R')
    uint8_t  magic1;          // 0x46 ('F')
    uint8_t  test_id;         // Test session / batch ID (0..255)
    uint8_t  direction;       // DIAG_DIR_*
    uint16_t seq_num;         // 16-bit sequence counter (0..65535, little-endian)
    uint8_t  payload_len;        // Payload byte count: 0, 10, 30, 53, 54, 55
    uint16_t turnaround_or_time; // Mode C Pong: turnaround_us (0..65535 us); Other modes/Ping: sender local millis() low 16-bits
};

// CRC-16-CCITT (poly 0x1021, init 0xFFFF)
static inline uint16_t diag_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// Generate deterministic payload pattern: (seq + index) & 0xFF
static inline void diag_fill_payload(uint8_t* payload, uint8_t len, uint16_t seq) {
    for (uint8_t i = 0; i < len; i++) {
        payload[i] = (uint8_t)((seq + i) & 0xFF);
    }
}

// Validate deterministic payload pattern
static inline bool diag_verify_payload(const uint8_t* payload, uint8_t len, uint16_t seq) {
    for (uint8_t i = 0; i < len; i++) {
        if (payload[i] != (uint8_t)((seq + i) & 0xFF)) {
            return false;
        }
    }
    return true;
}

// Check if payload length is one of the supported Phase 1 test values: 0, 10, 30, 53, 54, 55
static inline bool diag_is_supported_payload_len(uint8_t len) {
    return (len == 0 || len == 10 || len == 30 || len == 53 || len == 54 || len == 55);
}

// Check if total frame length is supported and fits safely within buffer size
static inline bool diag_is_valid_frame_len(uint8_t payload_len) {
    if (!diag_is_supported_payload_len(payload_len)) return false;
    if (payload_len > (DIAG_BUFFER_SIZE - DIAG_OVERHEAD_SIZE)) return false;
    size_t total = (size_t)DIAG_HEADER_SIZE + (size_t)payload_len + (size_t)DIAG_CRC_SIZE;
    return (total <= DIAG_BUFFER_SIZE);
}

// Compute total frame length from payload length
static inline size_t diag_total_frame_len(uint8_t payload_len) {
    return (size_t)DIAG_HEADER_SIZE + (size_t)payload_len + (size_t)DIAG_CRC_SIZE;
}
