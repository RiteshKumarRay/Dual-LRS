#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// =============================================================================
// CRSF PROTOCOL DEFINITIONS (TBS Crossfire / ExpressLRS Standard)
// Wire format: 420,000 baud, 8N1, full-duplex / half-duplex UART
// =============================================================================

#define CRSF_BAUDRATE                         420000
#define CRSF_BAUDRATE_SLOW                    115200

#define CRSF_ADDRESS_FLIGHT_CONTROLLER        0xC8
#define CRSF_ADDRESS_TRANSMITTER              0xEE
#define CRSF_ADDRESS_RADIO_TRANSMITTER        0xEA
#define CRSF_ADDRESS_CRSF_RECEIVER            0xEC

#define CRSF_FRAMETYPE_GPS                    0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR         0x08
#define CRSF_FRAMETYPE_HEARTBEAT              0x0B
#define CRSF_FRAMETYPE_LINK_STATISTICS        0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED     0x16
#define CRSF_FRAMETYPE_ATTITUDE               0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE            0x21
#define CRSF_FRAMETYPE_DEVICE_PING            0x28
#define CRSF_FRAMETYPE_DEVICE_INFO            0x29

#define CRSF_FRAME_RC_PAYLOAD_SIZE            22  // 16 channels * 11 bits = 176 bits = 22 bytes
#define CRSF_FRAME_RC_LENGTH                  24  // Type (1B) + Channels (22B) + CRC (1B) = 24
#define CRSF_FRAME_RC_TOTAL_SIZE              26  // Sync (1B) + Len (1B) + Type (1B) + Channels (22B) + CRC (1B)

#define CRSF_NUM_CHANNELS                     16
#define CRSF_CHANNEL_VALUE_MIN                172   // ~988 us
#define CRSF_CHANNEL_VALUE_MID                992   // ~1500 us
#define CRSF_CHANNEL_VALUE_MAX                1811  // ~2012 us

// Standard CRSF CRC-8 DVB-S2 Lookup Table (Polynomial 0xD5)
static const uint8_t CRSF_CRC8_TABLE[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

// Calculate CRSF CRC-8 (DVB-S2, Poly 0xD5) over data buffer
static inline uint8_t crsf_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc = CRSF_CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

// =============================================================================
// CRSF FRAME ENCODING & DECODING
// =============================================================================

// Packs 16x 11-bit channels into 22-byte buffer
static inline bool crsf_pack_channels22(const uint16_t* in_channels16, uint8_t* out_payload22) {
    if (!in_channels16 || !out_payload22) return false;

    uint32_t bit_buf = 0;
    uint8_t bits_in_buf = 0;
    size_t byte_idx = 0;

    for (size_t ch = 0; ch < CRSF_NUM_CHANNELS; ++ch) {
        uint16_t val = in_channels16[ch];
        if (val > 2047) val = 2047;
        bit_buf |= ((uint32_t)val << bits_in_buf);
        bits_in_buf += 11;
        while (bits_in_buf >= 8) {
            out_payload22[byte_idx++] = (uint8_t)(bit_buf & 0xFF);
            bit_buf >>= 8;
            bits_in_buf -= 8;
        }
    }
    if (bits_in_buf > 0 && byte_idx < 22) {
        out_payload22[byte_idx++] = (uint8_t)(bit_buf & 0xFF);
    }
    return true;
}

// Encodes 16 channels into a standard 26-byte CRSF packet ready for UART
// dest_addr: typically CRSF_ADDRESS_FLIGHT_CONTROLLER (0xC8)
static inline size_t crsf_encode_rc_frame(uint8_t dest_addr, const uint16_t* channels16, uint8_t* out_buf26) {
    if (!channels16 || !out_buf26) return 0;

    out_buf26[0] = dest_addr;
    out_buf26[1] = CRSF_FRAME_RC_LENGTH;           // 24 (type + 22 bytes payload + crc)
    out_buf26[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED; // 0x16

    if (!crsf_pack_channels22(channels16, &out_buf26[3])) return 0;

    // CRC is computed over frame type (byte 2) and payload (bytes 3..24) = 23 bytes
    out_buf26[25] = crsf_crc8(&out_buf26[2], CRSF_FRAME_RC_LENGTH - 1);
    return CRSF_FRAME_RC_TOTAL_SIZE;
}

// Unpacks 22-byte raw channels into 16x 11-bit channel array
static inline bool crsf_unpack_channels22(const uint8_t* in_payload22, uint16_t* out_channels16) {
    if (!in_payload22 || !out_channels16) return false;

    uint32_t bit_buf = 0;
    uint8_t bits_in_buf = 0;
    size_t byte_idx = 0;

    for (size_t ch = 0; ch < CRSF_NUM_CHANNELS; ++ch) {
        while (bits_in_buf < 11 && byte_idx < 22) {
            bit_buf |= ((uint32_t)in_payload22[byte_idx++] << bits_in_buf);
            bits_in_buf += 8;
        }
        if (bits_in_buf < 11) return false;
        out_channels16[ch] = (uint16_t)(bit_buf & 0x07FF);
        bit_buf >>= 11;
        bits_in_buf -= 11;
    }
    return true;
}

// =============================================================================
// STREAMING CRSF BYTE PARSER
// Ingests raw bytes from Handset / Receiver UART and emits valid RC frames
// =============================================================================

class CrsfParser {
public:
    CrsfParser() : dest_addr_(0), payload_len_(0), buf_len_(0), valid_frames_(0), crc_errors_(0) {
        memset(payload_, 0, sizeof(payload_));
        memset(buf_, 0, sizeof(buf_));
    }

    void reset() {
        buf_len_ = 0;
    }

    // Feeds a single byte. Returns true if a valid CRSF frame was completed.
    bool feed_byte(uint8_t byte) {
        if (buf_len_ < sizeof(buf_)) {
            buf_[buf_len_++] = byte;
        } else {
            memmove(buf_, &buf_[1], sizeof(buf_) - 1);
            buf_[sizeof(buf_) - 1] = byte;
        }

        while (buf_len_ >= 4) {
            uint8_t addr = buf_[0];
            if (addr != CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                addr != CRSF_ADDRESS_TRANSMITTER &&
                addr != CRSF_ADDRESS_RADIO_TRANSMITTER &&
                addr != CRSF_ADDRESS_CRSF_RECEIVER) {
                memmove(buf_, &buf_[1], --buf_len_);
                continue;
            }

            uint8_t len = buf_[1];
            if (len < 2 || len > 62) {
                memmove(buf_, &buf_[1], --buf_len_);
                continue;
            }

            size_t total_frame_len = (size_t)len + 2;
            if (buf_len_ < total_frame_len) {
                return false;
            }

            uint8_t computed_crc = crsf_crc8(&buf_[2], len - 1);
            uint8_t received_crc = buf_[total_frame_len - 1];

            if (computed_crc == received_crc) {
                dest_addr_ = addr;
                payload_len_ = len;
                memcpy(payload_, &buf_[2], len);
                buf_len_ -= total_frame_len;
                if (buf_len_ > 0) {
                    memmove(buf_, &buf_[total_frame_len], buf_len_);
                }
                valid_frames_++;
                return true;
            } else {
                crc_errors_++;
                memmove(buf_, &buf_[1], --buf_len_);
            }
        }
        return false;
    }

    uint8_t get_dest_addr() const { return dest_addr_; }
    uint8_t get_frame_type() const { return payload_[0]; }
    const uint8_t* get_payload() const { return &payload_[1]; }
    uint8_t get_payload_length() const { return (payload_len_ >= 2) ? (payload_len_ - 2) : 0; }

    // If frame is RC_CHANNELS_PACKED, unpacks channels into out_channels16
    bool get_channels(uint16_t* out_channels16) const {
        if (get_frame_type() != CRSF_FRAMETYPE_RC_CHANNELS_PACKED || get_payload_length() < 22) {
            return false;
        }
        return crsf_unpack_channels22(get_payload(), out_channels16);
    }

    // Direct access to 22-byte packed channels payload
    const uint8_t* get_raw_channels22() const {
        if (get_frame_type() != CRSF_FRAMETYPE_RC_CHANNELS_PACKED || get_payload_length() < 22) {
            return nullptr;
        }
        return get_payload();
    }

    uint32_t get_valid_frames() const { return valid_frames_; }
    uint32_t get_crc_errors() const { return crc_errors_; }

private:
    uint8_t dest_addr_;
    uint8_t payload_len_;
    uint8_t payload_[64];
    uint8_t buf_[128];
    size_t buf_len_;
    uint32_t valid_frames_;
    uint32_t crc_errors_;
};
