#!/usr/bin/env python3
"""
Calibrates the RC channel offsets by capturing raw center values from the
FS-i6X TX2 pad (via ESP32) and generating a correction table for rc_adapter.h.

Instructions:
1. Set all sticks to CENTER position (throttle mid if spring-loaded, or low)
2. Set all switches to default (low/off) position
3. Run this script — it captures 50 frames and computes per-channel offsets
4. Apply the generated correction to the firmware

Usage: python3 calibrate_tx2_offset.py [/dev/ttyUSB0]
"""

import sys, serial, time, json

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
SBUS_CENTER = 992  # Standard S.BUS neutral value

CYAN   = "\033[1;36m"
GREEN  = "\033[1;32m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def main():
    print(f"{CYAN}{'='*70}")
    print(f"  TX2 Channel Offset Calibration Tool")
    print(f"{'='*70}{RESET}\n")

    print(f"{BOLD}Instructions:{RESET}")
    print(f"  1. Set Roll, Pitch, Yaw sticks to CENTER")
    print(f"  2. Set Throttle to LOW (if no spring return) or CENTER")
    print(f"  3. Set all switches to their DEFAULT position")
    print(f"  4. Press Enter when ready...\n")
    input("Press Enter to start capturing...")

    ser = serial.Serial(PORT, 115200, timeout=0.1)
    time.sleep(0.3)
    ser.read(ser.in_waiting or 1)

    # Collect samples
    samples = {i: [] for i in range(16)}
    t0 = time.time()

    print(f"\nCapturing 50 frames (keep sticks centered)...")
    while len(samples[0]) < 50 and time.time() - t0 < 15:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if 'RC_GROUND_IN' not in line:
            continue
        try:
            d = json.loads(line)
            ch = d.get('ch', [])
            if len(ch) >= 16:
                for i in range(16):
                    samples[i].append(ch[i])
        except Exception:
            pass

    ser.close()

    if not samples[0]:
        print(f"{RED}ERROR: No frames captured!{RESET}")
        return

    print(f"Captured {len(samples[0])} frames.\n")

    # Calculate per-channel averages
    names = ['Roll/CH1', 'Pitch/CH2', 'Thr/CH3', 'Yaw/CH4'] + [f'Aux{i}/CH{i+4}' for i in range(1, 13)]

    # For stick channels (Roll, Pitch, Yaw), expected center = 992
    # For throttle: expected low = ~172-198 (or center if spring return)
    # For aux switches: depends on position (don't calibrate)

    stick_channels = [0, 1, 3]  # Roll, Pitch, Yaw (not Throttle)

    print(f"{BOLD}{'Channel':15s} {'Avg Raw':>10s} {'Expected':>10s} {'Offset':>10s} {'Status':>10s}{RESET}")
    print("-" * 60)

    offsets = [0] * 16
    for i in range(16):
        avg = sum(samples[i]) / len(samples[i])
        expected = SBUS_CENTER if i in stick_channels else avg  # Only calibrate stick channels
        offset = round(avg - expected)

        if i in stick_channels:
            offsets[i] = offset
            status = f"{GREEN}✓ OK{RESET}" if abs(offset) < 5 else f"{RED}⚠ OFFSET {offset:+d}{RESET}"
        else:
            status = f"(not calibrated)"

        print(f"  {names[i]:13s} {avg:10.1f} {expected:10.0f} {offset:+10d} {status}")

    # Report
    print()
    needs_fix = any(abs(offsets[i]) >= 5 for i in stick_channels)

    if needs_fix:
        print(f"{RED}{'='*70}")
        print(f"  OFFSET DETECTED — Correction needed")
        print(f"{'='*70}{RESET}\n")

        print(f"The following channels have offsets from the expected center ({SBUS_CENTER}):\n")
        for i in stick_channels:
            if abs(offsets[i]) >= 5:
                avg = sum(samples[i]) / len(samples[i])
                us_actual = round(avg * 5/8 + 880)
                us_expected = round(SBUS_CENTER * 5/8 + 880)
                print(f"  {names[i]:13s}: raw {avg:.0f} (= {us_actual} µs) instead of {SBUS_CENTER} (= {us_expected} µs)")
                print(f"  {'':13s}  Correction: {-offsets[i]:+d} raw counts ({round(-offsets[i]*5/8):+d} µs)")
                print()

        # Generate C++ correction code
        print(f"\n{BOLD}=== Generated Firmware Fix ==={RESET}")
        print(f"Add this to rc_adapter.h after line 181 (after channel unpack):\n")
        print(f"```cpp")
        print(f"// TX2 channel offset correction (auto-calibrated {time.strftime('%Y-%m-%d')})")
        print(f"// Compensates for OpenI6X TX2 SBUS output offset on FS-i6X")
        print(f"static const int16_t TX2_CH_OFFSET[{len(stick_channels)}] = {{", end="")
        parts = []
        for i in stick_channels:
            parts.append(f"{-offsets[i]:+d}")
        print(f" {', '.join(parts)} }};  // Roll, Pitch, Yaw corrections")
        print(f"static const uint8_t TX2_OFFSET_CHANNELS[] = {{ {', '.join(str(i) for i in stick_channels)} }};")
        print(f"for (uint8_t k = 0; k < {len(stick_channels)}; ++k) {{")
        print(f"    uint8_t ci = TX2_OFFSET_CHANNELS[k];")
        print(f"    int16_t corrected = (int16_t)tmp_ch[ci] + TX2_CH_OFFSET[k];")
        print(f"    if (corrected < 0) corrected = 0;")
        print(f"    if (corrected > 2047) corrected = 2047;")
        print(f"    tmp_ch[ci] = (uint16_t)corrected;")
        print(f"}}")
        print(f"```")
        print()
        print(f"{YELLOW}NOTE: This is a firmware-level workaround. The real fix is to{RESET}")
        print(f"{YELLOW}recalibrate OpenI6X sticks (Radio Setup → Calibration) or update{RESET}")
        print(f"{YELLOW}the OpenI6X firmware if this is a known bug.{RESET}")
    else:
        print(f"{GREEN}{'='*70}")
        print(f"  ALL CHANNELS CENTERED — No correction needed!")
        print(f"{'='*70}{RESET}")

if __name__ == "__main__":
    main()
