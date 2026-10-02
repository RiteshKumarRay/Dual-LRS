#pragma once
#include <stdint.h>
#include <stddef.h>
#include "crsf_protocol.h"
#include "transport_protocol.h"
#include "transport_engine.h"

// =============================================================================
// CONTROLLED RC ADAPTER
// Bridges CRSF handset input on Ground -> RF Transport -> CRSF FC output on Air
// =============================================================================

struct RcAdapterStats {
    uint32_t crsf_frames_in;
    uint32_t crsf_crc_errors;
    uint32_t rf_frames_sent;
    uint32_t rf_frames_received;
    uint32_t failsafe_events;
    uint32_t restore_events;
};

// -----------------------------------------------------------------------------
// GROUND RC ADAPTER
// Ingests raw CRSF bytes from FS-i6X transmitter and packs them for RF uplink
// -----------------------------------------------------------------------------
class RcGroundAdapter {
public:
    RcGroundAdapter() : rc_sequence_(0), has_new_frame_(false), using_sbus_(false), sbus_enabled_(false), has_ping_(false), has_handset_(false), last_handset_time_ms_(0), sbus_idx_(0), stats_{} {
        memset(last_channels_, 0, sizeof(last_channels_));
        memset(sbus_raw22_, 0, sizeof(sbus_raw22_));
        memset(sbus_buf_, 0, sizeof(sbus_buf_));
    }

    void enable_sbus(bool en) { sbus_enabled_ = en; }
    bool pop_ping_request() { bool h = has_ping_; has_ping_ = false; return h; }

    // Feeds raw byte from handset UART (handles standard CRSF and OpenI6X 26B framing)
    bool feed_byte(uint8_t byte, uint32_t current_time_ms = 0, uint32_t dt_us = 0) {
        if (dt_us > 2000) {
            parser_.reset();
            sbus_idx_ = 0;
        }
        if (parser_.feed_byte(byte)) {
            if (parser_.get_frame_type() == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
                stats_.crsf_frames_in++;
                if (parser_.get_channels(last_channels_)) {
                    has_new_frame_ = true;
                    using_sbus_ = false;
                    has_handset_ = true;
                    last_handset_time_ms_ = current_time_ms;
                    return true;
                }
            } else if (parser_.get_frame_type() == CRSF_FRAMETYPE_DEVICE_PING) {
                has_ping_ = true;
            }
        }
        if (feed_sbus_byte(byte)) {
            has_handset_ = true;
            last_handset_time_ms_ = current_time_ms;
            return true;
        }
        if (parser_.get_crc_errors() > stats_.crsf_crc_errors) {
            stats_.crsf_crc_errors = parser_.get_crc_errors();
        }
        return false;
    }

    // Reset framing state only when switching baud-rate profiles during scan.
    // Keeps accumulated frame counters intact so lock detection can still work.
    void reset_protocol() {
        parser_.reset();          // Clear incomplete CRSF frame in buffer
        using_sbus_ = false;      // Clear SBUS mode — let new baud attempt CRSF first
        has_new_frame_ = false;
        sbus_idx_ = 0;            // Clear partial SBUS accumulator
        memset(sbus_buf_, 0, sizeof(sbus_buf_));
        // NOTE: Do NOT clear stats, last_channels_, or has_handset_ here —
        // keeping crsf_frames_in / valid_frames across switches is required
        // for the lock-detection threshold to accumulate properly.
    }

    const uint8_t* get_raw_channels22() const {
        return using_sbus_ ? sbus_raw22_ : parser_.get_raw_channels22();
    }

    // Generates 39-byte RF transport frame for the 32ms Ground uplink slot.
    // Returns frame length (39) or 0 if no valid frame to send.
    size_t get_uplink_frame(uint8_t* out_frame_buf, uint16_t transport_seq) {
        if (!out_frame_buf) return 0;

        TransportPackedRc packed_rc{};
        if (has_new_frame_) {
            if (using_sbus_) {
                memcpy(packed_rc.channels, sbus_raw22_, 22);
            } else {
                const uint8_t* raw_ch22 = parser_.get_raw_channels22();
                if (raw_ch22) {
                    memcpy(packed_rc.channels, raw_ch22, 22);
                } else {
                    transport_pack_rc_channels(last_channels_, packed_rc.channels);
                }
            }
        } else {
            // Send last known channels
            transport_pack_rc_channels(last_channels_, packed_rc.channels);
        }

        packed_rc.rc_sequence = rc_sequence_++;
        packed_rc.flags = 0;

        TransportHeader hdr{};
        hdr.magic0 = TRANSPORT_MAGIC0;
        hdr.magic1 = TRANSPORT_MAGIC1;
        hdr.version = TRANSPORT_VERSION;
        hdr.channel = (uint8_t)TransportChannel::RC_CONTROL;
        hdr.flags = 0; // Unreliable / low-latency
        hdr.sequence = transport_seq;
        hdr.transfer_id = 0;
        hdr.fragment_offset = 0;
        hdr.payload_length = sizeof(TransportPackedRc); // 24 bytes

        transport_encode_header(&hdr, out_frame_buf);
        memcpy(&out_frame_buf[TRANSPORT_HEADER_SIZE], &packed_rc, sizeof(TransportPackedRc));
        uint16_t crc = transport_crc16(wire_buf_ptr(out_frame_buf), TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc));
        transport_write_u16_be(&out_frame_buf[TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc)], crc);

        stats_.rf_frames_sent++;
        has_new_frame_ = false;
        return TRANSPORT_HEADER_SIZE + sizeof(TransportPackedRc) + TRANSPORT_CRC_SIZE; // 39 bytes
    }

    bool has_handset_signal(uint32_t current_time_ms = 0) const {
        if (!has_handset_) return false;
        if (current_time_ms > 0 && last_handset_time_ms_ > 0 && (current_time_ms - last_handset_time_ms_ > 500)) {
            return false;
        }
        return true;
    }

    // Fills out_packed with the 24-byte TransportPackedRc payload
    bool get_packed_rc(TransportPackedRc* out_packed, uint32_t current_time_ms = 0) {
        if (!out_packed || !has_handset_signal(current_time_ms)) return false;
        if (has_new_frame_) {
            if (using_sbus_) {
                memcpy(out_packed->channels, sbus_raw22_, 22);
            } else {
                const uint8_t* raw_ch22 = parser_.get_raw_channels22();
                if (raw_ch22) {
                    memcpy(out_packed->channels, raw_ch22, 22);
                } else {
                    transport_pack_rc_channels(last_channels_, out_packed->channels);
                }
            }
        } else {
            transport_pack_rc_channels(last_channels_, out_packed->channels);
        }
        out_packed->rc_sequence = rc_sequence_++;
        out_packed->flags = 0;
        stats_.rf_frames_sent++;
        has_new_frame_ = false;
        return true;
    }

    bool has_new_frame() const { return has_new_frame_; }
    const uint16_t* get_channels() const { return last_channels_; }
    const RcAdapterStats& get_stats() const { return stats_; }
    bool is_using_sbus() const { return using_sbus_; }
    uint32_t get_crsf_valid_frames() const { return parser_.get_valid_frames(); }
    uint32_t get_total_valid_frames() const { return stats_.crsf_frames_in; }

private:
    bool feed_sbus_byte(uint8_t byte) {
        if (sbus_idx_ < 26) {
            sbus_buf_[sbus_idx_++] = byte;
        } else {
            memmove(sbus_buf_, &sbus_buf_[1], 25);
            sbus_buf_[25] = byte;
        }

        // Case A: 26-byte OpenI6X extended frame on TX2 (0x0F 0x64 at idx 0..1, 22B channels at idx 2..23, footer byte 25 == 0x00)
        if (sbus_idx_ == 26 && sbus_buf_[0] == 0x0F && sbus_buf_[1] == 0x64 && sbus_buf_[25] == 0x00) {
            uint16_t tmp_ch[16];
            if (crsf_unpack_channels22(&sbus_buf_[2], tmp_ch)) {
                // Route clean Roll stick from handset Ch6 (index 5) into canonical FC Roll Ch1 (index 0)
                // Channel 6 is freed / parked at neutral (992 counts = 1500 us)
                tmp_ch[0] = tmp_ch[5];
                tmp_ch[5] = 992;
                crsf_pack_channels22(tmp_ch, sbus_raw22_);
                memcpy(last_channels_, tmp_ch, sizeof(tmp_ch));
                stats_.crsf_frames_in++;
                has_new_frame_ = true;
                using_sbus_ = true;
                sbus_idx_ = 0; // Consume frame cleanly
                return true;
            }
        }

        // Case B: Standard 25-byte SBUS frame (only if sbus_enabled_ is set)
        if (sbus_enabled_ && sbus_idx_ >= 25 && sbus_buf_[0] == 0x0F && sbus_buf_[24] == 0x00) {
            uint16_t tmp_ch[16];
            if (crsf_unpack_channels22(&sbus_buf_[1], tmp_ch)) {
                memcpy(sbus_raw22_, &sbus_buf_[1], 22);
                memcpy(last_channels_, tmp_ch, sizeof(tmp_ch));
                stats_.crsf_frames_in++;
                has_new_frame_ = true;
                using_sbus_ = true;
                sbus_idx_ = 0;
                return true;
            }
        }

        // Case C: Standard 26-byte SBUS frame (only if sbus_enabled_ is set)
        if (sbus_enabled_ && sbus_idx_ == 26 && sbus_buf_[0] == 0x0F && sbus_buf_[25] == 0x00) {
            uint16_t tmp_ch[16];
            if (crsf_unpack_channels22(&sbus_buf_[1], tmp_ch)) {
                memcpy(sbus_raw22_, &sbus_buf_[1], 22);
                memcpy(last_channels_, tmp_ch, sizeof(tmp_ch));
                stats_.crsf_frames_in++;
                has_new_frame_ = true;
                using_sbus_ = true;
                sbus_idx_ = 0; // Consume frame cleanly
                return true;
            }
        }
        return false;
    }

    static inline const uint8_t* wire_buf_ptr(const uint8_t* p) { return p; }
    CrsfParser parser_;
    uint8_t rc_sequence_;
    bool has_new_frame_;
    bool using_sbus_;
    bool sbus_enabled_;
    bool has_ping_;
    bool has_handset_;
    uint32_t last_handset_time_ms_;
    uint8_t sbus_idx_;
    uint8_t sbus_buf_[32];
    uint8_t sbus_raw22_[22];
    uint16_t last_channels_[CRSF_NUM_CHANNELS];
    RcAdapterStats stats_;
};

// -----------------------------------------------------------------------------
// AIR RC ADAPTER
// Receives 39-byte RF frames from Ground, updates failsafe, formats CRSF for FC
// -----------------------------------------------------------------------------
class RcAirAdapter {
public:
    RcAirAdapter() : has_channels_(false), has_new_frame_(false), was_failsafe_(false), last_valid_time_ms_(0), stats_{} {
        memset(last_channels_, 0, sizeof(last_channels_));
    }

    // Ingests verified RC_CONTROL frame from TransportParser
    bool ingest_rc_frame(const TransportPackedRc& packed_rc, uint32_t current_time_ms) {
        stats_.rf_frames_received++;
        last_valid_time_ms_ = current_time_ms;
        mailbox_.update_from_wire(reinterpret_cast<const uint8_t*>(&packed_rc));
        failsafe_.record_frame_arrival(current_time_ms);
        failsafe_.update(current_time_ms);

        if (failsafe_.is_failsafe_active() != was_failsafe_) {
            if (failsafe_.is_failsafe_active()) {
                stats_.failsafe_events++;
            } else {
                stats_.restore_events++;
            }
            was_failsafe_ = failsafe_.is_failsafe_active();
        }

        if (crsf_unpack_channels22(packed_rc.channels, last_channels_)) {
            has_channels_ = true;
            has_new_frame_ = true;
            return true;
        }
        return false;
    }

    // Periodic tick to check for RC timeout
    void update(uint32_t current_time_ms) {
        failsafe_.update(current_time_ms);
        if (failsafe_.is_failsafe_active() != was_failsafe_) {
            if (failsafe_.is_failsafe_active()) {
                stats_.failsafe_events++;
            } else {
                stats_.restore_events++;
            }
            was_failsafe_ = failsafe_.is_failsafe_active();
        }
    }

    // Formats a 26-byte CRSF frame for the Flight Controller.
    // Returns 26 if frame generated, 0 if failsafe active (standard "No Pulses" policy).
    size_t get_fc_frame(uint8_t* out_frame26) {
        if (!out_frame26 || !has_channels_) return 0;

        // In failsafe: suppress CRSF pulses so Flight Controller detects RC loss immediately
        if (failsafe_.is_failsafe_active()) {
            return 0;
        }

        return crsf_encode_rc_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, last_channels_, out_frame26);
    }

    bool has_new_frame() const { return has_new_frame_; }
    void clear_new_frame() { has_new_frame_ = false; }
    bool is_failsafe_active() const { return failsafe_.is_failsafe_active(); }
    const uint16_t* get_channels() const { return last_channels_; }
    const RcAdapterStats& get_stats() const { return stats_; }
    uint32_t get_last_valid_time_ms() const { return last_valid_time_ms_; }

private:
    TransportRcMailbox mailbox_;
    TransportRcFailsafe failsafe_;
    bool has_channels_;
    bool has_new_frame_;
    bool was_failsafe_;
    uint32_t last_valid_time_ms_;
    uint16_t last_channels_[CRSF_NUM_CHANNELS];
    RcAdapterStats stats_;
};
