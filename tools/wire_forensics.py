#!/usr/bin/env python3
"""
ESP32 Wire Forensics: reads the raw wire hex from ESP32 RC_GROUND_IN
and decodes channels under MULTIPLE frame alignment hypotheses.

This is the KEY diagnostic: if Hypothesis B (standard S.BUS, channels
starting at byte 1 instead of byte 2) gives perfect centered Roll,
then the bug is a 1-byte frame offset in rc_adapter.h.

Usage: python3 wire_forensics.py [/dev/ttyUSB0]
"""

import sys, serial, time, json

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

def unpack_11bit(payload_bytes):
    """Unpack 16x 11-bit channels from bytes."""
    channels = []
    bit_buf = 0
    bits_in = 0
    idx = 0
    for _ in range(16):
        while bits_in < 11 and idx < len(payload_bytes):
            bit_buf |= payload_bytes[idx] << bits_in
            idx += 1
            bits_in += 8
        if bits_in < 11:
            break
        channels.append(bit_buf & 0x07FF)
        bit_buf >>= 11
        bits_in -= 11
    return channels

def raw_to_us(raw_val):
    return raw_val * 5 // 8 + 880

CYAN   = "\033[1;36m"
GREEN  = "\033[1;32m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def main():
    print(f"{CYAN}{'='*74}")
    print(f"  WIRE FORENSICS — Dual frame alignment diagnosis")
    print(f"  Reading ESP32 raw wire bytes from {PORT}")
    print(f"{'='*74}{RESET}\n")

    ser = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.3)

    # Drain buffer
    ser.read(ser.in_waiting or 1)

    print(f"{BOLD}Keep ALL sticks CENTERED and switches in default position.{RESET}")
    print(f"Collecting 20 frames to compare frame alignment hypotheses...\n")

    frames = []
    t0 = time.time()

    while len(frames) < 20 and time.time() - t0 < 10:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if 'RC_GROUND_IN' not in line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue

        wire_hex = d.get('wire', '')
        ch_hex = d.get('hex', '')
        ch_raw = d.get('ch', [])

        if not wire_hex or len(wire_hex) < 50:
            continue

        wire_bytes = bytes.fromhex(wire_hex)
        frames.append({
            'wire': wire_bytes,
            'hex': ch_hex,
            'ch': ch_raw,
        })

    if not frames:
        print(f"{RED}ERROR: No RC_GROUND_IN frames received. Is the ESP32 connected?{RESET}")
        ser.close()
        return

    print(f"Captured {len(frames)} frames. Analyzing...\n")

    # The ESP32 raw ring buffer is 32 bytes of the most recent UART bytes.
    # We need to find the 26-byte frame in it. The ESP32 code already identified
    # the frame and extracted 'ch' and 'hex'. We'll reconstruct the full frame
    # from 'wire' and also try alternative alignments.

    print(f"{BOLD}=== Current Code (Hypothesis A): 0x0F 0x64 [22 ch bytes at offset 2] ==={RESET}")
    print(f"    This is what the firmware currently decodes.\n")

    for i, f in enumerate(frames[:5]):
        ch = f['ch']
        us = [raw_to_us(c) for c in ch[:4]]
        mark_roll = RED + "⚠ OFFSET" + RESET if abs(us[0] - 1500) > 30 else GREEN + "✓ OK" + RESET
        mark_pitch = RED + "⚠ CROSS-TALK?" + RESET if abs(us[1] - 1500) > 30 else GREEN + "✓ OK" + RESET
        print(f"  Frame {i}: Roll={us[0]:4d}µs(raw={ch[0]:4d}) {mark_roll}  "
              f"Pitch={us[1]:4d}µs(raw={ch[1]:4d}) {mark_pitch}  "
              f"Thr={us[2]:4d}µs  Yaw={us[3]:4d}µs")

    # Now try alternative: what if 0x64 is actually the first byte of channel data
    # and channels should be unpacked from byte 1 (not byte 2)?
    print(f"\n{BOLD}=== Hypothesis B: 0x64 IS channel data (standard S.BUS, offset 1) ==={RESET}")
    print(f"    What if 0x64 is part of CH1, not a frame identifier?\n")

    wire_hex_str = frames[0].get('hex', '')
    if wire_hex_str and len(wire_hex_str) >= 44:
        # The 'hex' field IS the 22 bytes currently used (from offset 2).
        # To test offset 1, we need to prepend byte 0x64 and drop the last byte.
        hex_bytes = bytes.fromhex(wire_hex_str)

        # Construct the alternative 22 bytes: [0x64] + hex_bytes[0:21]
        alt_payload = bytes([0x64]) + hex_bytes[:21]
        alt_ch = unpack_11bit(alt_payload)

        if alt_ch and len(alt_ch) >= 4:
            us_alt = [raw_to_us(c) for c in alt_ch[:4]]
            for i, f in enumerate(frames[:5]):
                hex_b = bytes.fromhex(f['hex']) if f['hex'] else b''
                if len(hex_b) >= 21:
                    alt_p = bytes([0x64]) + hex_b[:21]
                    alt_c = unpack_11bit(alt_p)
                    us_a = [raw_to_us(c) for c in alt_c[:4]]
                    mark_roll = GREEN + "✓ CENTERED!" + RESET if abs(us_a[0] - 1500) < 30 else YELLOW + f"~{us_a[0]}" + RESET
                    mark_pitch = GREEN + "✓ OK" + RESET if abs(us_a[1] - 1500) < 30 else YELLOW + f"~{us_a[1]}" + RESET
                    print(f"  Frame {i}: Roll={us_a[0]:4d}µs(raw={alt_c[0]:4d}) {mark_roll}  "
                          f"Pitch={us_a[1]:4d}µs(raw={alt_c[1]:4d}) {mark_pitch}  "
                          f"Thr={us_a[2]:4d}µs  Yaw={us_a[3]:4d}µs")

    # Also scan the raw wire ring for standard S.BUS 25-byte frames
    print(f"\n{BOLD}=== Hypothesis C: Scan wire ring for standard S.BUS-25 frames ==={RESET}")
    print(f"    Looking for 0x0F + 22 ch bytes + flags + 0x00 (25 bytes)\n")

    found_sbus25 = False
    for fi, f in enumerate(frames[:5]):
        wire = f['wire']
        for offset in range(len(wire) - 24):
            if wire[offset] == 0x0F and wire[offset + 24] == 0x00:
                ch_data = wire[offset+1:offset+23]
                alt_ch = unpack_11bit(ch_data)
                if alt_ch and len(alt_ch) >= 4:
                    us_a = [raw_to_us(c) for c in alt_ch[:4]]
                    if 500 < us_a[0] < 2200 and 500 < us_a[2] < 2200:
                        found_sbus25 = True
                        mark_roll = GREEN + "✓ CENTERED!" + RESET if abs(us_a[0] - 1500) < 30 else YELLOW + f"~{us_a[0]}" + RESET
                        print(f"  Frame {fi} @offset {offset}: Roll={us_a[0]:4d}µs  "
                              f"Pitch={us_a[1]:4d}µs  Thr={us_a[2]:4d}µs  Yaw={us_a[3]:4d}µs  {mark_roll}")

    if not found_sbus25:
        print(f"  No standard S.BUS-25 frames found in wire ring buffer.")

    # Summary / Verdict
    print(f"\n{CYAN}{'='*74}")
    print(f"  VERDICT")
    print(f"{'='*74}{RESET}\n")

    # Check if Hypothesis A has offset
    ch_a = frames[0]['ch']
    us_a = [raw_to_us(c) for c in ch_a[:4]]
    a_offset = abs(us_a[0] - 1500)

    print(f"  Current firmware Roll reading:  {us_a[0]} µs  (expected: 1500 µs)")
    print(f"  Roll offset:                    {a_offset} µs")
    print()

    if a_offset > 30:
        print(f"  {RED}⚠  Roll is NOT centered with current frame parsing.{RESET}")
        print(f"  {YELLOW}   The FS-i6X TX2 output may have a different frame layout")
        print(f"   than what rc_adapter.h expects, OR the transmitter TX2 output")
        print(f"   itself has the offset baked in.{RESET}")
        print()
        print(f"  {BOLD}NEXT STEP:{RESET} Connect FTDI directly to TX2 pad to confirm.")
        print(f"  Run: python3 ftdi_raw_capture.py /dev/ttyUSBx 115200")
    else:
        print(f"  {GREEN}✓  Roll appears centered. Current parsing is correct.{RESET}")

    # Dump raw wire for manual inspection
    print(f"\n{BOLD}Raw wire hex dump (first 3 frames):{RESET}")
    for i, f in enumerate(frames[:3]):
        wire = f['wire']
        hex_str = wire.hex().upper()
        # Highlight 0F positions
        marked = ""
        for j in range(0, len(hex_str), 2):
            byte_hex = hex_str[j:j+2]
            if byte_hex == "0F":
                marked += f"{RED}{byte_hex}{RESET}"
            elif byte_hex == "64":
                marked += f"{YELLOW}{byte_hex}{RESET}"
            elif byte_hex == "00":
                marked += f"{GREEN}{byte_hex}{RESET}"
            else:
                marked += byte_hex
            marked += " "
        print(f"  Wire[{i}]: {marked}")
        print(f"  Hex[{i}]: {f['hex'][:44].upper()}")
        print()

    ser.close()

if __name__ == "__main__":
    main()
