#!/usr/bin/env python3
"""
Clean Step-by-Step Stick & Switch Identification Wizard
No terminal flood: guides user through 1 stick at a time.
"""

import sys
import time
import json
import os
import serial
from pymavlink import mavutil

def read_current_channels():
    # Reads from the configured MAVLink bridge, which represents true FC state.
    try:
        fc_host = os.getenv("DUAL_LRS_FC_HOST")
        if not fc_host:
            raise RuntimeError("Set DUAL_LRS_FC_HOST before using the MAVLink bridge")
        mav = mavutil.mavlink_connection(f"tcp:{fc_host}:5760")
        mav.wait_heartbeat(timeout=2)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            m = mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.5)
            if m:
                d = m.to_dict()
                return [d[f'chan{i}_raw'] for i in range(1, 17)]
    except Exception:
        pass

    # Fallback to Ground ESP32 (/dev/ttyUSB0)
    try:
        s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            l = s.readline()
            if b'RC_GROUND_IN' in l:
                data = json.loads(l.decode('utf-8', errors='replace').strip())
                raw_ch = data.get('ch', [])
                s.close()
                return [round((c - 992) * 5/8 + 1500) if c > 0 else 0 for c in raw_ch]
        s.close()
    except Exception:
        pass
    return None

def wait_for_movement(prompt_text, baseline):
    print(f"\n>>> {prompt_text}")
    print("    (Waiting for stick/switch movement...)")

    # Wait for a channel to move by at least 250 us from baseline
    t0 = time.time()
    detected_ch = None
    peak_delta = 0
    final_val = 0
    base_val = 0

    while time.time() - t0 < 15.0:
        ch = read_current_channels()
        if not ch:
            time.sleep(0.05)
            continue

        for i in range(16):
            delta = ch[i] - baseline[i]
            if abs(delta) > 200 and abs(delta) > abs(peak_delta):
                peak_delta = delta
                detected_ch = i + 1
                final_val = ch[i]
                base_val = baseline[i]

        if detected_ch and abs(peak_delta) > 300:
            # Found clear movement!
            time.sleep(0.3)
            break
        time.sleep(0.05)

    if detected_ch:
        print(f"    [DETECTED] Channel {detected_ch} moved: {base_val} us -> {final_val} us (Δ = {peak_delta:+d} us)")
        return detected_ch
    else:
        print("    [TIMEOUT] No significant movement detected (>200us).")
        return None

def main():
    print("=" * 70)
    print("DUAL-LRS CLEAN STICK IDENTIFICATION WIZARD")
    print("=" * 70)
    print("Make sure your transmitter is ON.")
    print("Step 0: Center all sticks and pots now.")
    input("Press Enter when ready...")

    # Establish baseline
    print("\nReading baseline...")
    time.sleep(0.5)
    baseline = read_current_channels()
    if not baseline:
        print("[-] Could not read RC channels from FC or Ground. Check connections.")
        sys.exit(1)

    print("Baseline channels 1..8 :", baseline[:8])
    print("Baseline channels 9..16:", baseline[8:])

    mapping = {}

    # Test 1: Throttle
    mapping['Throttle'] = wait_for_movement("Push THROTTLE stick all the way UP and hold it.", baseline)
    print("Release Throttle back to center.")
    time.sleep(1.0)
    b2 = read_current_channels() or baseline

    # Test 2: Aileron (Roll)
    mapping['Roll (Ail)'] = wait_for_movement("Push AILERON (Roll) stick all the way RIGHT and hold it.", b2)
    print("Release Aileron back to center.")
    time.sleep(1.0)
    b3 = read_current_channels() or baseline

    # Test 3: Elevator (Pitch)
    mapping['Pitch (Ele)'] = wait_for_movement("Push ELEVATOR (Pitch) stick all the way UP and hold it.", b3)
    print("Release Elevator back to center.")
    time.sleep(1.0)
    b4 = read_current_channels() or baseline

    # Test 4: Rudder (Yaw)
    mapping['Yaw (Rud)'] = wait_for_movement("Push RUDDER (Yaw) stick all the way RIGHT and hold it.", b4)
    print("Release Rudder back to center.")

    print("\n" + "=" * 70)
    print("FINAL DETECTED CHANNEL MAPPING:")
    print("=" * 70)
    for control, ch in mapping.items():
        print(f"  {control:15s} -> Channel {ch if ch else 'NOT DETECTED'}")
    print("=" * 70)

if __name__ == "__main__":
    main()
