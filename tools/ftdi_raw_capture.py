#!/usr/bin/env python3
"""
FTDI Direct Capture: reads raw bytes from FS-i6X TX2 pad via FTDI adapter.
Bypasses the entire ESP32 + Dual-LRS chain to prove where the problem is.

WIRING:
  FS-i6X TX2 pad  →  FTDI RX pin
  FS-i6X GND      →  FTDI GND

SIGNAL INVERSION NOTE:
  OpenI6X TX2 pad outputs INVERTED UART (S.BUS-style, idle LOW).
  Most FTDI / CH340 / CP2102 adapters expect normal UART (idle HIGH).

  If the signal is inverted, you will see garbage. Two options:
    A) Hardware inverter: one NPN transistor + 2x 10K resistors between TX2 and FTDI RX
    B) Set OpenI6X → Radio Setup → Hardware → Baudrate to a non-inverted setting
       (check OpenI6X docs for your version)

  This script tries BOTH interpretations (raw + bit-inverted) automatically.

Usage:
  python3 ftdi_raw_capture.py [/dev/ttyUSBx] [baudrate]
  Default: /dev/ttyUSB0 at 115200
"""

import sys, serial, time, struct

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

CRSF_MID = 992
US_OFFSET = 880

def unpack_11bit_channels(payload22):
    """Unpack 16x 11-bit channels from 22 bytes (CRSF/SBUS standard packing)."""
    channels = []
    bit_buf = 0
    bits_in = 0
    idx = 0
    for _ in range(16):
        while bits_in < 11 and idx < 22:
            bit_buf |= payload22[idx] << bits_in
            idx += 1
            bits_in += 8
        channels.append(bit_buf & 0x07FF)
        bit_buf >>= 11
        bits_in -= 11
    return channels

def raw_to_us(raw_val):
    """Convert 11-bit raw value to µs using ArduPilot's formula."""
    return raw_val * 5 // 8 + US_OFFSET

def try_decode_sbus25(buf25):
    """Try standard 25-byte S.BUS: 0x0F + 22ch + flags + 0x00."""
    if buf25[0] != 0x0F or buf25[24] != 0x00:
        return None
    ch_data = buf25[1:23]  # 22 bytes starting at index 1
    return unpack_11bit_channels(ch_data)

def try_decode_sbus26_openi6x(buf26):
    """Try 26-byte OpenI6X extended: 0x0F + 0x64 + 22ch + flags + 0x00."""
    if buf26[0] != 0x0F or buf26[25] != 0x00:
        return None
    ch_data = buf26[2:24]  # 22 bytes starting at index 2
    return unpack_11bit_channels(ch_data)

def try_decode_crsf(buf26):
    """Try standard 26-byte CRSF: addr + len(24) + type(0x16) + 22ch + CRC."""
    if buf26[1] != 24 or buf26[2] != 0x16:
        return None
    ch_data = buf26[3:25]  # 22 bytes starting at index 3
    return unpack_11bit_channels(ch_data)

def channels_plausible(channels):
    """Check if decoded channels look reasonable (not all zeros, values in range)."""
    if not channels:
        return False
    in_range = sum(1 for c in channels if 100 < c < 2000)
    return in_range >= 4  # at least 4 channels should be in a reasonable range

def print_channels(channels, label):
    """Print decoded channels in µs with Roll/Pitch/Thr/Yaw labels."""
    names = ["Roll", "Pitch", "Thr", "Yaw"] + [f"Aux{i}" for i in range(1, 13)]
    us = [raw_to_us(c) for c in channels]
    # Highlight channels that are notably off-center
    parts = []
    for i, (name, raw, u) in enumerate(zip(names[:8], channels[:8], us[:8])):
        marker = " " if abs(u - 1500) < 50 or (i == 2 and u < 1050) else "*"
        parts.append(f"{name}={u:4d}µs(raw={raw:4d}){marker}")
    print(f"  [{label}] {' | '.join(parts)}")

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    print("Connect FTDI RX → FS-i6X TX2 pad, FTDI GND → FS-i6X GND")
    print("(If you see only garbage/no frames, the signal may need a hardware inverter)\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"ERROR: Cannot open {PORT}: {e}")
        print("If the ESP32 is on /dev/ttyUSB0, try /dev/ttyUSB1 for FTDI")
        sys.exit(1)

    ring = bytearray(64)
    ring_pos = 0
    ring_fill = 0

    sbus25_count = 0
    sbus26_count = 0
    crsf_count = 0
    garbage_bytes = 0
    last_print = time.time()
    start = time.time()

    print("Scanning for valid frames... (10 seconds)\n")
    print("Frame format detection:")
    print("  - Standard S.BUS 25-byte (0x0F + 22ch + flags + 0x00)")
    print("  - OpenI6X 26-byte (0x0F + 0x64 + 22ch + flags + 0x00)")
    print("  - CRSF 26-byte (addr + 24 + 0x16 + 22ch + CRC)")
    print()

    while time.time() - start < 15.0:
        data = ser.read(64)
        if not data:
            continue

        for b in data:
            ring[ring_pos % 64] = b
            ring_pos += 1
            ring_fill = min(ring_fill + 1, 64)
            garbage_bytes += 1

            if ring_fill >= 26:
                # Build contiguous view of last 26 bytes
                start_idx = (ring_pos - 26) % 64
                buf26 = bytes([ring[(start_idx + i) % 64] for i in range(26)])

                # Also build 25-byte view
                start_idx25 = (ring_pos - 25) % 64
                buf25 = bytes([ring[(start_idx25 + i) % 64] for i in range(25)])

                # Try all three decodings
                ch_sbus25 = try_decode_sbus25(buf25)
                ch_sbus26 = try_decode_sbus26_openi6x(buf26)
                ch_crsf = try_decode_crsf(buf26)

                decoded_any = False

                if ch_sbus25 and channels_plausible(ch_sbus25):
                    sbus25_count += 1
                    decoded_any = True
                    if time.time() - last_print > 0.25:
                        print_channels(ch_sbus25, "S.BUS-25")
                        last_print = time.time()

                if ch_sbus26 and channels_plausible(ch_sbus26):
                    sbus26_count += 1
                    decoded_any = True
                    if time.time() - last_print > 0.25:
                        print_channels(ch_sbus26, "OpenI6X-26")
                        last_print = time.time()

                if ch_crsf and channels_plausible(ch_crsf):
                    crsf_count += 1
                    decoded_any = True
                    if time.time() - last_print > 0.25:
                        print_channels(ch_crsf, "CRSF-26")
                        last_print = time.time()

                # Also try bit-inverted (for inverted UART read by non-inverting FTDI)
                inv_buf26 = bytes([b ^ 0xFF for b in buf26])
                inv_buf25 = bytes([b ^ 0xFF for b in buf25])

                ch_inv25 = try_decode_sbus25(inv_buf25)
                ch_inv26 = try_decode_sbus26_openi6x(inv_buf26)
                ch_inv_crsf = try_decode_crsf(inv_buf26)

                if ch_inv25 and channels_plausible(ch_inv25):
                    decoded_any = True
                    if time.time() - last_print > 0.25:
                        print("  *** INVERTED SIGNAL DETECTED ***")
                        print_channels(ch_inv25, "INV-SBUS25")
                        last_print = time.time()

                if ch_inv26 and channels_plausible(ch_inv26):
                    decoded_any = True
                    if time.time() - last_print > 0.25:
                        print("  *** INVERTED SIGNAL DETECTED ***")
                        print_channels(ch_inv26, "INV-OpenI6X26")
                        last_print = time.time()

    print(f"\n{'='*70}")
    print(f"CAPTURE SUMMARY ({time.time()-start:.1f}s)")
    print(f"  Standard S.BUS-25 frames detected: {sbus25_count}")
    print(f"  OpenI6X extended-26 frames detected: {sbus26_count}")
    print(f"  CRSF-26 frames detected:            {crsf_count}")
    print(f"  Total bytes received:                {garbage_bytes}")

    if sbus25_count == 0 and sbus26_count == 0 and crsf_count == 0:
        print("\n  ⚠  NO VALID FRAMES DETECTED!")
        print("  Possible causes:")
        print("    1. Signal is inverted — need hardware inverter (transistor + 2 resistors)")
        print("    2. Wrong baud rate — try: python3 ftdi_raw_capture.py /dev/ttyUSBx 400000")
        print("    3. No connection — check wiring TX2→FTDI_RX, GND→FTDI_GND")
        print("    4. Wrong serial port — check 'ls /dev/ttyUSB*'")
    else:
        winner = max([("S.BUS-25", sbus25_count), ("OpenI6X-26", sbus26_count), ("CRSF-26", crsf_count)], key=lambda x: x[1])
        print(f"\n  ✓ Best match: {winner[0]} ({winner[1]} frames)")

    ser.close()

if __name__ == "__main__":
    main()
