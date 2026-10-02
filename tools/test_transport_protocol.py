#!/usr/bin/env python3
"""
Dual-LRS Phase 2 Transport Protocol Host-Side Verification Suite
Validates encoding, decoding, CRC, bounds checks, reassembly, direction validation,
NACK reason codes, explicit big-endian wire serialization, and exact RC bit packing.
"""

import struct
import unittest
from typing import Optional, Tuple, Dict, List

# Protocol Constants
TRANSPORT_MAGIC0 = 0x44  # 'D'
TRANSPORT_MAGIC1 = 0x4C  # 'L'
TRANSPORT_VERSION = 0x01
TRANSPORT_HEADER_SIZE = 13
TRANSPORT_CRC_SIZE = 2
TRANSPORT_OVERHEAD_SIZE = TRANSPORT_HEADER_SIZE + TRANSPORT_CRC_SIZE  # 15 bytes
TRANSPORT_E22_SUBPACKET_LIMIT = 64
TRANSPORT_MAX_SINGLE_BURST_PAYLOAD = 49  # 64 - 13 - 2 = 49 bytes
TRANSPORT_MAX_FRAME_SIZE = 64            # Phase 2 v1 strict single-burst limit
TRANSPORT_MAX_TRANSFER_SIZE = 512
TRANSPORT_MAX_FRAGMENTS = 11             # ceil(512 / 49) = 11

# Channels
CHANNEL_RC_CONTROL = 0x01
CHANNEL_LINK_CONTROL = 0x02
CHANNEL_MAVLINK_UPLINK = 0x03
CHANNEL_MAVLINK_DOWNLINK = 0x04
VALID_CHANNELS = {CHANNEL_RC_CONTROL, CHANNEL_LINK_CONTROL, CHANNEL_MAVLINK_UPLINK, CHANNEL_MAVLINK_DOWNLINK}

# Node Roles
ROLE_GROUND = 0x01
ROLE_AIR = 0x02

# Flags
FLAG_RELIABLE = 1 << 0
FLAG_IS_ACK = 1 << 1
FLAG_IS_NACK = 1 << 2
FLAG_FIRST_FRAG = 1 << 3
FLAG_LAST_FRAG = 1 << 4

# Link Subtypes
LINK_SUBTYPE_HEARTBEAT = 0x01
LINK_SUBTYPE_ACK_NACK = 0x02
LINK_SUBTYPE_SYNC_REQ = 0x03
LINK_SUBTYPE_SYNC_RESP = 0x04

# NACK Reason Codes
NACK_REASON_NONE = 0x00
NACK_REASON_BAD_CRC = 0x01
NACK_REASON_GAP_DETECTED = 0x02
NACK_REASON_BUFFER_FULL = 0x03
NACK_REASON_OFFSET_OVERRUN = 0x04
NACK_REASON_TRANSFER_TIMEOUT = 0x05
NACK_REASON_OVERLAP_CONFLICT = 0x06

# Packed RC Constants & Flags
RC_NUM_CHANNELS = 16
RC_CHANNEL_MIN = 0
RC_CHANNEL_MAX = 2047
RC_FLAG_FAILSAFE = 1 << 0
RC_FLAG_FRAME_LOST = 1 << 1


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT (poly 0x1021, init 0xFFFF). Matches C++ transport_crc16()."""
    crc = init
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def sequence_diff(seq_rx: int, seq_exp: int) -> int:
    """Sequence difference with 16-bit wraparound handling."""
    diff = (seq_rx - seq_exp) & 0xFFFF
    if diff >= 0x8000:
        diff -= 0x10000
    return diff


def is_valid_direction(receiver_role: int, channel: int) -> bool:
    """Validates if channel is permissible for the receiving role."""
    if receiver_role == ROLE_AIR:
        return channel in (CHANNEL_RC_CONTROL, CHANNEL_LINK_CONTROL, CHANNEL_MAVLINK_UPLINK)
    elif receiver_role == ROLE_GROUND:
        return channel in (CHANNEL_LINK_CONTROL, CHANNEL_MAVLINK_DOWNLINK)
    return False


# =============================================================================
# EXPLICIT BIG-ENDIAN WIRE SERIALIZATION HELPERS
# =============================================================================

def write_u16_be(val: int) -> bytes:
    """Explicit Big-Endian 16-bit unsigned serialization."""
    return bytes([(val >> 8) & 0xFF, val & 0xFF])


def read_u16_be(data: bytes) -> int:
    """Explicit Big-Endian 16-bit unsigned deserialization."""
    return ((data[0] << 8) | data[1]) & 0xFFFF


def encode_header(magic0: int, magic1: int, version: int, channel: int, flags: int,
                  sequence: int, transfer_id: int, fragment_offset: int, payload_length: int) -> bytes:
    """Encodes TransportHeader to exact 13-byte wire format (Big-Endian)."""
    out = bytearray(13)
    out[0] = magic0 & 0xFF
    out[1] = magic1 & 0xFF
    out[2] = version & 0xFF
    out[3] = channel & 0xFF
    out[4] = flags & 0xFF
    out[5:7] = write_u16_be(sequence)
    out[7:9] = write_u16_be(transfer_id)
    out[9:11] = write_u16_be(fragment_offset)
    out[11:13] = write_u16_be(payload_length)
    return bytes(out)


def decode_header(raw: bytes) -> Dict[str, int]:
    """Decodes 13-byte wire buffer to header dictionary."""
    if len(raw) < 13:
        raise ValueError("Header raw buffer must be at least 13 bytes")
    return {
        "magic0": raw[0],
        "magic1": raw[1],
        "version": raw[2],
        "channel": raw[3],
        "flags": raw[4],
        "sequence": read_u16_be(raw[5:7]),
        "transfer_id": read_u16_be(raw[7:9]),
        "fragment_offset": read_u16_be(raw[9:11]),
        "payload_length": read_u16_be(raw[11:13]),
    }


def encode_link_heartbeat(subtype: int, link_quality: int, rssi: int, snr: int, rtt_ms: int) -> bytes:
    """Encodes TransportLinkHeartbeat to exact 6-byte wire format."""
    out = bytearray(6)
    out[0] = subtype & 0xFF
    out[1] = link_quality & 0xFF
    out[2] = struct.pack("b", rssi)[0]
    out[3] = struct.pack("b", snr)[0]
    out[4:6] = write_u16_be(rtt_ms)
    return bytes(out)


def decode_link_heartbeat(raw: bytes) -> Dict[str, int]:
    """Decodes 6-byte wire buffer to heartbeat dictionary."""
    if len(raw) < 6:
        raise ValueError("Heartbeat buffer must be at least 6 bytes")
    return {
        "subtype": raw[0],
        "link_quality": raw[1],
        "rssi": struct.unpack("b", bytes([raw[2]]))[0],
        "snr": struct.unpack("b", bytes([raw[3]]))[0],
        "rtt_ms": read_u16_be(raw[4:6]),
    }


def encode_link_ack_nack(subtype: int, target_channel: int, transfer_id: int,
                         fragment_offset: int, nack_reason: int, reserved: int = 0) -> bytes:
    """Encodes TransportLinkAckNack to exact 8-byte wire format."""
    out = bytearray(8)
    out[0] = subtype & 0xFF
    out[1] = target_channel & 0xFF
    out[2:4] = write_u16_be(transfer_id)
    out[4:6] = write_u16_be(fragment_offset)
    out[6] = nack_reason & 0xFF
    out[7] = reserved & 0xFF
    return bytes(out)


def decode_link_ack_nack(raw: bytes) -> Dict[str, int]:
    """Decodes 8-byte wire buffer to ACK/NACK dictionary."""
    if len(raw) < 8:
        raise ValueError("ACK/NACK buffer must be at least 8 bytes")
    return {
        "subtype": raw[0],
        "target_channel": raw[1],
        "transfer_id": read_u16_be(raw[2:4]),
        "fragment_offset": read_u16_be(raw[4:6]),
        "nack_reason": raw[6],
        "reserved": raw[7],
    }


# =============================================================================
# EXACT RC BIT-PACKING SPECIFICATION & ALGORITHM
# =============================================================================

def pack_rc_channels(channels: List[int], sequence: int, flags: int, clamp: bool = True) -> bytes:
    """
    Packs 16 x 11-bit RC channels into 22 bytes + 1B sequence + 1B flags = 24 bytes.
    Rules:
    - Channel 0 starts at bit 0 of byte 0.
    - Channels packed consecutively in little-endian bit order.
    - Each channel uses exactly 11 bits (range 0..2047).
    - If clamp == True: values outside 0..2047 are clamped.
    - If clamp == False: values outside 0..2047 raise ValueError.
    """
    if len(channels) != RC_NUM_CHANNELS:
        raise ValueError(f"Must provide exactly {RC_NUM_CHANNELS} channels")

    bit_buf = 0
    bits_in_buf = 0
    out = bytearray()

    for ch_val in channels:
        if ch_val < RC_CHANNEL_MIN or ch_val > RC_CHANNEL_MAX:
            if not clamp:
                raise ValueError(f"Channel value {ch_val} outside 0..2047 range")
            val = max(RC_CHANNEL_MIN, min(RC_CHANNEL_MAX, ch_val))
        else:
            val = ch_val

        bit_buf |= (val << bits_in_buf)
        bits_in_buf += 11
        while bits_in_buf >= 8:
            out.append(bit_buf & 0xFF)
            bit_buf >>= 8
            bits_in_buf -= 8

    if bits_in_buf > 0:
        out.append(bit_buf & 0xFF)

    assert len(out) == 22, f"Expected 22 bytes, got {len(out)}"
    out.append(sequence & 0xFF)
    out.append(flags & 0xFF)
    assert len(out) == 24
    return bytes(out)


def unpack_rc_channels(data: bytes) -> Tuple[List[int], int, int]:
    """Unpacks 24-byte TransportPackedRc into (channels[16], sequence, flags)."""
    if len(data) != 24:
        raise ValueError("TransportPackedRc must be exactly 24 bytes")
    channels = []
    bit_buf = 0
    bits_in_buf = 0
    byte_idx = 0
    for _ in range(RC_NUM_CHANNELS):
        while bits_in_buf < 11 and byte_idx < 22:
            bit_buf |= (data[byte_idx] << bits_in_buf)
            bits_in_buf += 8
            byte_idx += 1
        ch = bit_buf & 0x07FF
        bit_buf >>= 11
        bits_in_buf -= 11
        channels.append(ch)
    sequence = data[22]
    flags = data[23]
    return channels, sequence, flags


class TransportFrame:
    def __init__(self, channel: int, flags: int, sequence: int, transfer_id: int,
                 fragment_offset: int, payload: bytes, version: int = TRANSPORT_VERSION):
        self.magic0 = TRANSPORT_MAGIC0
        self.magic1 = TRANSPORT_MAGIC1
        self.version = version
        self.channel = channel
        self.flags = flags
        self.sequence = sequence
        self.transfer_id = transfer_id
        self.fragment_offset = fragment_offset
        self.payload_length = len(payload)
        self.payload = payload

    def encode(self) -> bytes:
        """Serializes TransportHeader (13B) + Payload + CRC-16 (2B) with explicit big-endian order."""
        hdr = encode_header(
            self.magic0,
            self.magic1,
            self.version,
            self.channel,
            self.flags,
            self.sequence,
            self.transfer_id,
            self.fragment_offset,
            self.payload_length
        )
        body = hdr + self.payload
        crc = crc16_ccitt(body)
        return body + write_u16_be(crc)

    @classmethod
    def decode(cls, raw: bytes, receiver_role: Optional[int] = None) -> Tuple[Optional["TransportFrame"], str]:
        """Decodes and validates a raw transport frame with Phase 2 v1 single-burst rules."""
        if len(raw) < TRANSPORT_HEADER_SIZE + TRANSPORT_CRC_SIZE:
            return None, "FRAME_TOO_SHORT"

        hdr_dict = decode_header(raw[:TRANSPORT_HEADER_SIZE])
        magic0 = hdr_dict["magic0"]
        magic1 = hdr_dict["magic1"]
        version = hdr_dict["version"]
        channel = hdr_dict["channel"]
        flags = hdr_dict["flags"]
        seq = hdr_dict["sequence"]
        xfer_id = hdr_dict["transfer_id"]
        frag_off = hdr_dict["fragment_offset"]
        plen = hdr_dict["payload_length"]

        if magic0 != TRANSPORT_MAGIC0 or magic1 != TRANSPORT_MAGIC1:
            return None, "INVALID_MAGIC"

        if version != TRANSPORT_VERSION:
            return None, "UNSUPPORTED_VERSION"

        if channel not in VALID_CHANNELS:
            return None, "UNSUPPORTED_CHANNEL"

        if receiver_role is not None and not is_valid_direction(receiver_role, channel):
            return None, "INVALID_DIRECTION"

        if plen > TRANSPORT_MAX_SINGLE_BURST_PAYLOAD:
            return None, "PAYLOAD_TOO_LARGE_FOR_SINGLE_BURST"

        expected_total = TRANSPORT_HEADER_SIZE + plen + TRANSPORT_CRC_SIZE
        if len(raw) != expected_total:
            return None, "LENGTH_MISMATCH"

        payload = raw[TRANSPORT_HEADER_SIZE:TRANSPORT_HEADER_SIZE + plen]
        received_crc = read_u16_be(raw[TRANSPORT_HEADER_SIZE + plen:TRANSPORT_HEADER_SIZE + plen + 2])
        computed_crc = crc16_ccitt(raw[:TRANSPORT_HEADER_SIZE + plen])

        if received_crc != computed_crc:
            return None, "CRC_MISMATCH"

        frame = cls(channel, flags, seq, xfer_id, frag_off, payload, version)
        return frame, "OK"


class ReassemblyBuffer:
    """Simulates the receiver's multi-fragment reassembly contract."""
    def __init__(self, max_size: int = TRANSPORT_MAX_TRANSFER_SIZE):
        self.max_size = max_size
        self.active_channel = 0
        self.active_transfer_id = 0
        self.has_active_transfer = False
        self.total_size = 0
        self.buffer = bytearray()
        self.received_ranges: List[Tuple[int, int]] = []
        self.is_complete = False

    def reset(self):
        self.active_channel = 0
        self.active_transfer_id = 0
        self.has_active_transfer = False
        self.total_size = 0
        self.buffer = bytearray()
        self.received_ranges = []
        self.is_complete = False

    def mark_complete_consumed(self):
        self.has_active_transfer = False
        self.is_complete = False
        self.active_transfer_id = 0
        self.total_size = 0
        self.received_ranges = []
        self.buffer = bytearray()

    def process_fragment(self, frame: TransportFrame) -> Tuple[bool, str]:
        start_offset = frame.fragment_offset
        end_offset = frame.fragment_offset + frame.payload_length
        if start_offset >= self.max_size or end_offset > self.max_size or end_offset < start_offset:
            return False, "OFFSET_OVERRUN"

        # Check when no active transfer
        if not self.has_active_transfer:
            if not (frame.flags & FLAG_FIRST_FRAG):
                return False, "MISSING_FIRST_FRAGMENT"
            # Initialize new transfer
            self.has_active_transfer = True
            self.active_channel = frame.channel
            self.active_transfer_id = frame.transfer_id
            self.buffer = bytearray()
            self.received_ranges = []
            self.is_complete = False
            self.total_size = 0
        elif self.is_complete:
            # Previous transfer already completed
            if frame.channel == self.active_channel and frame.transfer_id == self.active_transfer_id:
                for r_start, r_end in self.received_ranges:
                    if r_start == start_offset and r_end == end_offset:
                        return False, "DUPLICATE_FRAGMENT"
                return False, "OVERLAP_CONFLICT"
            else:
                if not (frame.flags & FLAG_FIRST_FRAG):
                    return False, "MISSING_FIRST_FRAGMENT"
                # Atomically start new transfer, replacing completed result
                self.has_active_transfer = True
                self.active_channel = frame.channel
                self.active_transfer_id = frame.transfer_id
                self.buffer = bytearray()
                self.received_ranges = []
                self.is_complete = False
                self.total_size = 0
        else:
            # Active transfer in progress:
            # A new transfer (or different transfer_id) must NOT preempt active transfer.
            # Preserve active transfer and reject with BUFFER_FULL.
            if frame.channel != self.active_channel or frame.transfer_id != self.active_transfer_id:
                return False, "BUFFER_FULL"

            # Check for exact duplicate vs partial overlap
            for r_start, r_end in self.received_ranges:
                if r_start == start_offset and r_end == end_offset:
                    return False, "DUPLICATE_FRAGMENT"
                # If intervals overlap but are not exact duplicates:
                if max(start_offset, r_start) < min(end_offset, r_end):
                    return False, "OVERLAP_CONFLICT"

        # Expand buffer if needed
        if len(self.buffer) < end_offset:
            self.buffer.extend(b"\x00" * (end_offset - len(self.buffer)))

        # Copy payload bytes
        self.buffer[start_offset:end_offset] = frame.payload
        self.received_ranges.append((start_offset, end_offset))

        if frame.flags & FLAG_LAST_FRAG:
            self.total_size = end_offset

        # Check coverage from 0 to total_size
        if self.total_size > 0:
            coverage = [False] * self.total_size
            for r_start, r_end in self.received_ranges:
                for idx in range(r_start, min(r_end, self.total_size)):
                    coverage[idx] = True
            if all(coverage):
                self.is_complete = True
                return True, "TRANSFER_COMPLETE"

        return True, "FRAGMENT_ACCEPTED"


class TestTransportProtocol(unittest.TestCase):
    def test_valid_frame_encode_decode(self):
        payload = b"Hello, Dual-LRS Transport!"
        frame = TransportFrame(
            channel=CHANNEL_MAVLINK_UPLINK,
            flags=FLAG_FIRST_FRAG | FLAG_LAST_FRAG,
            sequence=100,
            transfer_id=1,
            fragment_offset=0,
            payload=payload
        )
        encoded = frame.encode()
        self.assertEqual(len(encoded), TRANSPORT_HEADER_SIZE + len(payload) + TRANSPORT_CRC_SIZE)

        decoded, status = TransportFrame.decode(encoded)
        self.assertEqual(status, "OK")
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.channel, CHANNEL_MAVLINK_UPLINK)
        self.assertEqual(decoded.sequence, 100)
        self.assertEqual(decoded.transfer_id, 1)
        self.assertEqual(decoded.fragment_offset, 0)
        self.assertEqual(decoded.payload, payload)

    def test_crc_failure(self):
        payload = b"Test CRC integrity"
        frame = TransportFrame(CHANNEL_LINK_CONTROL, 0, 1, 1, 0, payload)
        encoded = bytearray(frame.encode())
        # Corrupt 1 byte in payload
        encoded[TRANSPORT_HEADER_SIZE + 2] ^= 0xFF

        decoded, status = TransportFrame.decode(bytes(encoded))
        self.assertIsNone(decoded)
        self.assertEqual(status, "CRC_MISMATCH")

    def test_unsupported_channel(self):
        payload = b"Invalid Channel"
        # Channel 0x00 is invalid
        frame = TransportFrame(0x00, 0, 1, 1, 0, payload)
        encoded = frame.encode()

        decoded, status = TransportFrame.decode(encoded)
        self.assertIsNone(decoded)
        self.assertEqual(status, "UNSUPPORTED_CHANNEL")

    def test_direction_validation(self):
        # Air receiving MAVLINK_DOWNLINK should be rejected
        f_downlink = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, 0, 1, 1, 0, b"telemetry").encode()
        decoded, status = TransportFrame.decode(f_downlink, receiver_role=ROLE_AIR)
        self.assertIsNone(decoded)
        self.assertEqual(status, "INVALID_DIRECTION")

        # Ground receiving RC_CONTROL should be rejected
        f_rc = TransportFrame(CHANNEL_RC_CONTROL, 0, 1, 1, 0, b"rc_data").encode()
        decoded, status = TransportFrame.decode(f_rc, receiver_role=ROLE_GROUND)
        self.assertIsNone(decoded)
        self.assertEqual(status, "INVALID_DIRECTION")

        # Valid directions
        decoded_air, status_air = TransportFrame.decode(f_rc, receiver_role=ROLE_AIR)
        self.assertEqual(status_air, "OK")
        self.assertIsNotNone(decoded_air)

        decoded_gnd, status_gnd = TransportFrame.decode(f_downlink, receiver_role=ROLE_GROUND)
        self.assertEqual(status_gnd, "OK")
        self.assertIsNotNone(decoded_gnd)

    def test_invalid_payload_length(self):
        # 50 bytes payload exceeds 49 bytes single-burst limit
        payload = b"A" * 50
        frame = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, 0, 1, 1, 0, payload)
        encoded = frame.encode()

        decoded, status = TransportFrame.decode(encoded)
        self.assertIsNone(decoded)
        self.assertEqual(status, "PAYLOAD_TOO_LARGE_FOR_SINGLE_BURST")

    def test_maximum_single_burst_payload(self):
        # 49 bytes payload + 13B header + 2B CRC = 64B exact
        payload = b"X" * TRANSPORT_MAX_SINGLE_BURST_PAYLOAD
        frame = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, 0, 1, 1, 0, payload)
        encoded = frame.encode()

        self.assertEqual(len(encoded), TRANSPORT_E22_SUBPACKET_LIMIT)  # 64 bytes
        decoded, status = TransportFrame.decode(encoded)
        self.assertEqual(status, "OK")
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.payload_length, 49)

    def test_sequence_wraparound(self):
        # Normal increment
        self.assertEqual(sequence_diff(10, 9), 1)
        # Stale/duplicate
        self.assertEqual(sequence_diff(9, 10), -1)
        # Wraparound from 65535 -> 0
        self.assertEqual(sequence_diff(0, 65535), 1)
        self.assertEqual(sequence_diff(65535, 0), -1)
        # Multi-frame advance over wrap
        self.assertEqual(sequence_diff(5, 65530), 11)
        self.assertEqual(sequence_diff(65530, 5), -11)

    def test_duplicate_fragment(self):
        buf = ReassemblyBuffer()
        f1 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 1, 10, 0, b"Chunk1")
        ok, st = buf.process_fragment(f1)
        self.assertTrue(ok)
        self.assertEqual(st, "FRAGMENT_ACCEPTED")

        # Duplicate fragment
        ok2, st2 = buf.process_fragment(f1)
        self.assertFalse(ok2)
        self.assertEqual(st2, "DUPLICATE_FRAGMENT")

    def test_missing_first_fragment(self):
        buf = ReassemblyBuffer()
        # Non-first fragment arriving when no transfer is active
        f_mid = TransportFrame(CHANNEL_MAVLINK_UPLINK, 0, 2, 50, 49, b"MiddleChunk")
        ok, st = buf.process_fragment(f_mid)
        self.assertFalse(ok)
        self.assertEqual(st, "MISSING_FIRST_FRAGMENT")

    def test_offset_overrun(self):
        buf = ReassemblyBuffer(max_size=512)
        # Offset 500 + payload 20 = 520 > 512
        f_over = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 1, 10, 500, b"X" * 20)
        ok, st = buf.process_fragment(f_over)
        self.assertFalse(ok)
        self.assertEqual(st, "OFFSET_OVERRUN")

        # Boundary case: offset 512, payload 1 -> 513 > 512
        f_512 = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 1, 10, 512, b"X")
        ok, st = buf.process_fragment(f_512)
        self.assertFalse(ok)
        self.assertEqual(st, "OFFSET_OVERRUN")

        # Boundary case: offset 65535, payload 1 -> wrap attack
        f_wrap = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 1, 10, 65535, b"X")
        ok, st = buf.process_fragment(f_wrap)
        self.assertFalse(ok)
        self.assertEqual(st, "OFFSET_OVERRUN")

        # Boundary case: offset 500, payload 13 -> 513 > 512
        f_513 = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 1, 10, 500, b"X" * 13)
        ok, st = buf.process_fragment(f_513)
        self.assertFalse(ok)
        self.assertEqual(st, "OFFSET_OVERRUN")

        # Boundary case: offset 0, payload 49 -> valid!
        f_valid = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 1, 10, 0, b"Y" * 49)
        ok, st = buf.process_fragment(f_valid)
        self.assertTrue(ok)
        self.assertEqual(len(buf.buffer), 49)

    def test_out_of_order_fragment(self):
        buf = ReassemblyBuffer()
        # Message of 3 chunks delivered out of order: Chunk 0, then Chunk 2, verifying Chunk 1 remains pending
        f0 = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 0, 100, 0, b"Part0-")
        f2 = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_LAST_FRAG, 2, 100, 12, b"Part2")

        ok0, st0 = buf.process_fragment(f0)
        self.assertTrue(ok0)
        self.assertFalse(buf.is_complete)

        ok2, st2 = buf.process_fragment(f2)
        self.assertTrue(ok2)
        # Even though LAST_FRAG arrived, buffer is incomplete because chunk 1 (offset 6..11) is missing
        self.assertFalse(buf.is_complete)

        # Now deliver chunk 1
        f1 = TransportFrame(CHANNEL_MAVLINK_UPLINK, 0, 1, 100, 6, b"Part1-")
        ok1, st1 = buf.process_fragment(f1)
        self.assertTrue(ok1)
        self.assertTrue(buf.is_complete)
        self.assertEqual(bytes(buf.buffer), b"Part0-Part1-Part2")

    def test_transfer_completion(self):
        buf = ReassemblyBuffer()
        f0 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 0, 200, 0, b"Alpha-")
        f1 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, 0, 1, 200, 6, b"Beta-")
        f2 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_LAST_FRAG, 2, 200, 11, b"Gamma")

        buf.process_fragment(f0)
        self.assertFalse(buf.is_complete)
        buf.process_fragment(f1)
        self.assertFalse(buf.is_complete)
        ok, st = buf.process_fragment(f2)

        self.assertTrue(ok)
        self.assertEqual(st, "TRANSFER_COMPLETE")
        self.assertTrue(buf.is_complete)
        self.assertEqual(bytes(buf.buffer), b"Alpha-Beta-Gamma")

    def test_ack_nack_parsing(self):
        # Format: subtype(1B), target_channel(1B), transfer_id(2B), fragment_offset(2B), nack_reason(1B), reserved(1B)
        # Total: 8 bytes, matching sizeof(TransportLinkAckNack)
        ack_payload = encode_link_ack_nack(LINK_SUBTYPE_ACK_NACK, CHANNEL_MAVLINK_UPLINK, 42, 49, NACK_REASON_NONE, 0x00)
        self.assertEqual(len(ack_payload), 8)

        frame = TransportFrame(
            channel=CHANNEL_LINK_CONTROL,
            flags=FLAG_IS_ACK,
            sequence=5,
            transfer_id=0,
            fragment_offset=0,
            payload=ack_payload
        )
        encoded = frame.encode()
        decoded, status = TransportFrame.decode(encoded)
        self.assertEqual(status, "OK")
        self.assertTrue(decoded.flags & FLAG_IS_ACK)

        ack_dict = decode_link_ack_nack(decoded.payload)
        self.assertEqual(ack_dict["subtype"], LINK_SUBTYPE_ACK_NACK)
        self.assertEqual(ack_dict["target_channel"], CHANNEL_MAVLINK_UPLINK)
        self.assertEqual(ack_dict["transfer_id"], 42)
        self.assertEqual(ack_dict["fragment_offset"], 49)
        self.assertEqual(ack_dict["nack_reason"], NACK_REASON_NONE)
        self.assertEqual(ack_dict["reserved"], 0x00)

        # Test NACK with GAP_DETECTED
        nack_payload = encode_link_ack_nack(LINK_SUBTYPE_ACK_NACK, CHANNEL_MAVLINK_DOWNLINK, 99, 98, NACK_REASON_GAP_DETECTED, 0x00)
        frame_nack = TransportFrame(CHANNEL_LINK_CONTROL, FLAG_IS_NACK, 6, 0, 0, nack_payload)
        dec_nack, st_nack = TransportFrame.decode(frame_nack.encode())
        self.assertEqual(st_nack, "OK")
        self.assertTrue(dec_nack.flags & FLAG_IS_NACK)
        nack_dict = decode_link_ack_nack(dec_nack.payload)
        self.assertEqual(nack_dict["nack_reason"], NACK_REASON_GAP_DETECTED)

    # =========================================================================
    # EXPLICIT BIG-ENDIAN SERIALIZATION EXACT BYTE TESTS
    # =========================================================================

    def test_explicit_big_endian_header_exact_bytes(self):
        """Asserts exact encoded wire bytes for TransportHeader with known multi-byte values."""
        # sequence = 0x1234 (4660) -> MSB=0x12, LSB=0x34
        # transfer_id = 0x5678 (22136) -> MSB=0x56, LSB=0x78
        # fragment_offset = 0x0031 (49) -> MSB=0x00, LSB=0x31
        # payload_length = 0x002A (42) -> MSB=0x00, LSB=0x2A
        flags = FLAG_RELIABLE | FLAG_FIRST_FRAG | FLAG_LAST_FRAG  # 0x01 | 0x08 | 0x10 = 0x19
        raw_hdr = encode_header(
            magic0=TRANSPORT_MAGIC0,  # 0x44
            magic1=TRANSPORT_MAGIC1,  # 0x4C
            version=TRANSPORT_VERSION,  # 0x01
            channel=CHANNEL_MAVLINK_UPLINK,  # 0x03
            flags=flags,  # 0x19
            sequence=0x1234,
            transfer_id=0x5678,
            fragment_offset=0x0031,
            payload_length=0x002A
        )
        self.assertEqual(len(raw_hdr), 13)

        expected_bytes = bytes([
            0x44, 0x4C,        # magic0, magic1
            0x01,              # version
            0x03,              # channel
            0x19,              # flags
            0x12, 0x34,        # sequence (MSB, LSB)
            0x56, 0x78,        # transfer_id (MSB, LSB)
            0x00, 0x31,        # fragment_offset (MSB, LSB)
            0x00, 0x2A         # payload_length (MSB, LSB)
        ])
        self.assertEqual(raw_hdr, expected_bytes)

        # Verify decode produces identical values
        d = decode_header(raw_hdr)
        self.assertEqual(d["magic0"], 0x44)
        self.assertEqual(d["magic1"], 0x4C)
        self.assertEqual(d["version"], 0x01)
        self.assertEqual(d["channel"], 0x03)
        self.assertEqual(d["flags"], 0x19)
        self.assertEqual(d["sequence"], 0x1234)
        self.assertEqual(d["transfer_id"], 0x5678)
        self.assertEqual(d["fragment_offset"], 0x0031)
        self.assertEqual(d["payload_length"], 0x002A)

    def test_explicit_big_endian_heartbeat_exact_bytes(self):
        """Asserts exact encoded wire bytes for TransportLinkHeartbeat."""
        # rtt_ms = 0x01A4 (420 ms) -> MSB=0x01, LSB=0xA4
        # rssi = -75 -> signed int8 (0xB5)
        # snr = 10 -> signed int8 (0x0A)
        raw_hb = encode_link_heartbeat(
            subtype=LINK_SUBTYPE_HEARTBEAT,  # 0x01
            link_quality=98,                 # 0x62
            rssi=-75,
            snr=10,
            rtt_ms=0x01A4
        )
        self.assertEqual(len(raw_hb), 6)

        expected_bytes = bytes([
            0x01,        # subtype
            0x62,        # link_quality (98)
            0xB5,        # rssi (-75 in two's complement)
            0x0A,        # snr (+10)
            0x01, 0xA4   # rtt_ms (420 ms, MSB, LSB)
        ])
        self.assertEqual(raw_hb, expected_bytes)

        # Verify decode
        d = decode_link_heartbeat(raw_hb)
        self.assertEqual(d["subtype"], 0x01)
        self.assertEqual(d["link_quality"], 98)
        self.assertEqual(d["rssi"], -75)
        self.assertEqual(d["snr"], 10)
        self.assertEqual(d["rtt_ms"], 420)

    def test_explicit_big_endian_ack_nack_exact_bytes(self):
        """Asserts exact encoded wire bytes for TransportLinkAckNack."""
        # transfer_id = 0xA1B2 (41394) -> MSB=0xA1, LSB=0xB2
        # fragment_offset = 0x0092 (146) -> MSB=0x00, LSB=0x92
        raw_an = encode_link_ack_nack(
            subtype=LINK_SUBTYPE_ACK_NACK,  # 0x02
            target_channel=CHANNEL_MAVLINK_DOWNLINK,  # 0x04
            transfer_id=0xA1B2,
            fragment_offset=0x0092,
            nack_reason=NACK_REASON_GAP_DETECTED,  # 0x02
            reserved=0x00
        )
        self.assertEqual(len(raw_an), 8)

        expected_bytes = bytes([
            0x02,        # subtype
            0x04,        # target_channel
            0xA1, 0xB2,  # transfer_id (MSB, LSB)
            0x00, 0x92,  # fragment_offset (MSB, LSB)
            0x02,        # nack_reason
            0x00         # reserved
        ])
        self.assertEqual(raw_an, expected_bytes)

        d = decode_link_ack_nack(raw_an)
        self.assertEqual(d["subtype"], 0x02)
        self.assertEqual(d["target_channel"], 0x04)
        self.assertEqual(d["transfer_id"], 0xA1B2)
        self.assertEqual(d["fragment_offset"], 0x0092)
        self.assertEqual(d["nack_reason"], NACK_REASON_GAP_DETECTED)
        self.assertEqual(d["reserved"], 0x00)

    # =========================================================================
    # EXACT RC BIT-PACKING ALGORITHM TESTS
    # =========================================================================

    def test_rc_bit_packing_all_zero(self):
        """All 16 channels set to 0 must pack to exactly 22 bytes of 0x00."""
        channels = [0] * 16
        seq = 0
        flags = 0
        packed = pack_rc_channels(channels, seq, flags)
        self.assertEqual(len(packed), 24)
        self.assertEqual(packed[:22], bytes(22))
        self.assertEqual(packed[22], 0)
        self.assertEqual(packed[23], 0)

        unpacked_channels, unpacked_seq, unpacked_flags = unpack_rc_channels(packed)
        self.assertEqual(unpacked_channels, channels)
        self.assertEqual(unpacked_seq, seq)
        self.assertEqual(unpacked_flags, flags)

    def test_rc_bit_packing_all_maximum(self):
        """All 16 channels set to 2047 (0x7FF, 11 ones) must pack to exactly 22 bytes of 0xFF."""
        channels = [2047] * 16
        seq = 0xAA
        flags = RC_FLAG_FAILSAFE | RC_FLAG_FRAME_LOST
        packed = pack_rc_channels(channels, seq, flags)
        self.assertEqual(len(packed), 24)
        self.assertEqual(packed[:22], bytes([0xFF] * 22))
        self.assertEqual(packed[22], 0xAA)
        self.assertEqual(packed[23], flags)

        unpacked_channels, unpacked_seq, unpacked_flags = unpack_rc_channels(packed)
        self.assertEqual(unpacked_channels, channels)
        self.assertEqual(unpacked_seq, seq)
        self.assertEqual(unpacked_flags, flags)

    def test_rc_bit_packing_alternating(self):
        """Alternating 0 and 2047 channels."""
        channels = [0, 2047] * 8
        seq = 42
        flags = 0
        packed = pack_rc_channels(channels, seq, flags)
        self.assertEqual(len(packed), 24)

        unpacked_channels, unpacked_seq, unpacked_flags = unpack_rc_channels(packed)
        self.assertEqual(unpacked_channels, channels)
        self.assertEqual(unpacked_seq, seq)

    def test_rc_bit_packing_boundaries(self):
        """Assert exact byte values when only single boundary channels are active."""
        # 1. Channel 0 = 2047 (bits 0..10 = 1, bits 11..175 = 0)
        # Byte 0: bits 0..7 = 1 -> 0xFF
        # Byte 1: bits 0..2 = 1 -> 0x07 (0b00000111)
        # Bytes 2..21: 0x00
        ch0_only = [2047] + [0] * 15
        packed0 = pack_rc_channels(ch0_only, 0, 0)
        self.assertEqual(packed0[0], 0xFF)
        self.assertEqual(packed0[1], 0x07)
        self.assertEqual(packed0[2:22], bytes(20))
        ch_out, _, _ = unpack_rc_channels(packed0)
        self.assertEqual(ch_out, ch0_only)

        # 2. Channel 1 = 2047 (bits 11..21 = 1, all other bits = 0)
        # Byte 0: 0x00
        # Byte 1: bits 3..7 = 1 -> 0xF8 (0b11111000)
        # Byte 2: bits 0..5 = 1 -> 0x3F (0b00111111)
        # Bytes 3..21: 0x00
        ch1_only = [0, 2047] + [0] * 14
        packed1 = pack_rc_channels(ch1_only, 0, 0)
        self.assertEqual(packed1[0], 0x00)
        self.assertEqual(packed1[1], 0xF8)
        self.assertEqual(packed1[2], 0x3F)
        self.assertEqual(packed1[3:22], bytes(19))
        ch_out, _, _ = unpack_rc_channels(packed1)
        self.assertEqual(ch_out, ch1_only)

        # 3. Channel 15 = 2047 (bits 165..175 = 1, all other bits = 0)
        # 165 // 8 = 20, 165 % 8 = 5 (bits 5..7 of byte 20 -> 0b11100000 = 0xE0)
        # Byte 21: remaining 8 bits (bits 0..7 -> 0xFF)
        # Bytes 0..19: 0x00
        ch15_only = [0] * 15 + [2047]
        packed15 = pack_rc_channels(ch15_only, 0, 0)
        self.assertEqual(packed15[:20], bytes(20))
        self.assertEqual(packed15[20], 0xE0)
        self.assertEqual(packed15[21], 0xFF)
        ch_out, _, _ = unpack_rc_channels(packed15)
        self.assertEqual(ch_out, ch15_only)

    def test_rc_bit_packing_out_of_range(self):
        """Tests out-of-range behavior: clamping vs rejection."""
        invalid_channels = [1000] * 15 + [2048]

        # With clamp=False: must raise ValueError
        with self.assertRaises(ValueError):
            pack_rc_channels(invalid_channels, 0, 0, clamp=False)

        # Negative value with clamp=False: must raise ValueError
        negative_channels = [-1] + [1000] * 15
        with self.assertRaises(ValueError):
            pack_rc_channels(negative_channels, 0, 0, clamp=False)

        # With clamp=True: 2048 clamps to 2047, -1 clamps to 0
        clamped_packed = pack_rc_channels([3000] + [1000] * 14 + [-50], 1, 0, clamp=True)
        unpacked_channels, _, _ = unpack_rc_channels(clamped_packed)
        self.assertEqual(unpacked_channels[0], 2047)  # 3000 clamped to 2047
        self.assertEqual(unpacked_channels[15], 0)    # -50 clamped to 0

    def test_active_transfer_preservation_rejects_new_transfer(self):
        """Active transfer in progress must reject a new FIRST_FRAG with BUFFER_FULL and preserve active data."""
        buf = ReassemblyBuffer()
        # Transfer 10 starts
        f0 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 0, 10, 0, b"Transfer10-Chunk0-")
        ok, st = buf.process_fragment(f0)
        self.assertTrue(ok)
        self.assertEqual(buf.active_transfer_id, 10)

        # New transfer 20 attempts to preempt with FIRST_FRAG
        f_preempt = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 1, 20, 0, b"Transfer20-Chunk0-")
        ok_preempt, st_preempt = buf.process_fragment(f_preempt)
        self.assertFalse(ok_preempt)
        self.assertEqual(st_preempt, "BUFFER_FULL")

        # Original transfer 10 is preserved and can complete
        f1 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_LAST_FRAG, 2, 10, 18, b"Transfer10-Chunk1")
        ok1, st1 = buf.process_fragment(f1)
        self.assertTrue(ok1)
        self.assertEqual(st1, "TRANSFER_COMPLETE")
        self.assertEqual(bytes(buf.buffer), b"Transfer10-Chunk0-Transfer10-Chunk1")

    def test_partial_overlap_rejected(self):
        """Partial overlap must be rejected with OVERLAP_CONFLICT and must not overwrite buffer bytes."""
        buf = ReassemblyBuffer()
        # Transfer 30 starts with chunk [0..20)
        f0 = TransportFrame(CHANNEL_MAVLINK_UPLINK, FLAG_FIRST_FRAG, 0, 30, 0, b"01234567890123456789")
        ok0, _ = buf.process_fragment(f0)
        self.assertTrue(ok0)

        # Conflicting chunk [15..35) partially overlaps [0..20)
        f_overlap = TransportFrame(CHANNEL_MAVLINK_UPLINK, 0, 1, 30, 15, b"XXXXX567890123456789")
        ok_ov, st_ov = buf.process_fragment(f_overlap)
        self.assertFalse(ok_ov)
        self.assertEqual(st_ov, "OVERLAP_CONFLICT")

        # Verify original bytes in [15..20) were NOT overwritten
        self.assertEqual(bytes(buf.buffer[:20]), b"01234567890123456789")

    def test_sequential_transfers_and_lifecycle(self):
        """Sequential completed transfers must succeed, duplicate chunks re-ACKed, and preemption blocked when incomplete."""
        buf = ReassemblyBuffer()

        # Transfer 1: 1 chunk (FIRST_FRAG | LAST_FRAG)
        f1 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG | FLAG_LAST_FRAG, 0, 1, 0, b"MSG_1")
        ok1, st1 = buf.process_fragment(f1)
        self.assertTrue(ok1)
        self.assertEqual(st1, "TRANSFER_COMPLETE")
        self.assertEqual(bytes(buf.buffer), b"MSG_1")

        # Duplicate of completed transfer 1: re-ACKed idempotently
        ok_dup, st_dup = buf.process_fragment(f1)
        self.assertFalse(ok_dup)
        self.assertEqual(st_dup, "DUPLICATE_FRAGMENT")

        # Transfer 2: arrives after transfer 1 is complete -> atomically takes over
        f2 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG | FLAG_LAST_FRAG, 1, 2, 0, b"MSG_2")
        ok2, st2 = buf.process_fragment(f2)
        self.assertTrue(ok2)
        self.assertEqual(st2, "TRANSFER_COMPLETE")
        self.assertEqual(bytes(buf.buffer), b"MSG_2")

        # Explicit consumption
        buf.mark_complete_consumed()
        self.assertFalse(buf.has_active_transfer)
        self.assertFalse(buf.is_complete)

        # Multi-fragment Transfer 3: Chunk 0 (49 bytes)
        f3_0 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 2, 3, 0, b"A" * 49)
        ok3_0, st3_0 = buf.process_fragment(f3_0)
        self.assertTrue(ok3_0)
        self.assertFalse(buf.is_complete)

        # Preemption attempt while Transfer 3 is incomplete -> BUFFER_FULL
        f4_preempt = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG, 3, 4, 0, b"PREEMPT")
        ok_p, st_p = buf.process_fragment(f4_preempt)
        self.assertFalse(ok_p)
        self.assertEqual(st_p, "BUFFER_FULL")
        self.assertEqual(buf.active_transfer_id, 3)

        # Complete Transfer 3 with Chunk 1
        f3_1 = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_LAST_FRAG, 4, 3, 49, b"B" * 20)
        ok3_1, st3_1 = buf.process_fragment(f3_1)
        self.assertTrue(ok3_1)
        self.assertEqual(st3_1, "TRANSFER_COMPLETE")
        self.assertEqual(len(buf.buffer), 69)

        # Now Transfer 4 can start since Transfer 3 completed
        f4_valid = TransportFrame(CHANNEL_MAVLINK_DOWNLINK, FLAG_FIRST_FRAG | FLAG_LAST_FRAG, 5, 4, 0, b"VALID_4")
        ok4, st4 = buf.process_fragment(f4_valid)
        self.assertTrue(ok4)
        self.assertEqual(st4, "TRANSFER_COMPLETE")
        self.assertEqual(bytes(buf.buffer), b"VALID_4")

    def test_direct_cpp_serialization_tests(self):
        """Compiles and executes the direct C++ host test suite tools/test_transport_protocol.cpp."""
        import subprocess
        res = subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-I", "include",
             "tools/test_transport_protocol.cpp", "-o", "tools/test_transport_protocol_cpp"],
            capture_output=True, text=True
        )
        self.assertEqual(res.returncode, 0, f"C++ compilation failed: {res.stderr}")
        run_res = subprocess.run(["./tools/test_transport_protocol_cpp"], capture_output=True, text=True)
        self.assertEqual(run_res.returncode, 0, f"C++ test execution failed: {run_res.stderr}")
        self.assertIn("ALL C++ TRANSPORT TESTS PASSED", run_res.stdout)


if __name__ == "__main__":
    unittest.main()
