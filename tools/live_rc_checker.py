#!/usr/bin/env python3
"""
Live Flight Controller RC Channel Monitor via Radxa Companion Computer
Monitors ArduPilot RC_CHANNELS and SYS_STATUS sensor health live via MAVLink TCP bridge.
"""

import sys
import time
import os
from pymavlink import mavutil

RADXA_TCP = os.environ.get("RADXA_TCP", "tcp:10.233.23.186:5760")

def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 25
    print(f"Connecting to FC via Radxa ({RADXA_TCP})...")
    mav = mavutil.mavlink_connection(RADXA_TCP)
    hb = mav.wait_heartbeat(timeout=5)
    if not hb:
        print("ERROR: No heartbeat from Flight Controller!")
        sys.exit(1)

    print(f"Connected to Autopilot System {mav.target_system}, Component {mav.target_component}")

    # Request RC_CHANNELS stream at 10 Hz
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1
    )

    # Check initial health
    msg = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    if msg:
        rc_bit = 1 << 16
        healthy = bool(msg.onboard_control_sensors_health & rc_bit)
        enabled = bool(msg.onboard_control_sensors_enabled & rc_bit)
        present = bool(msg.onboard_control_sensors_present & rc_bit)
        print(f"SYS_STATUS: Present={present}, Enabled={enabled}, Healthy={healthy}")

    print("\n" + "="*75)
    print(f" MONITORING LIVE RC INPUT FOR {duration} SECONDS — MOVE STICKS ON FS-i6X NOW!")
    print("="*75)

    t0 = time.time()
    last_vals = None
    change_count = 0

    while time.time() - t0 < duration:
        rc = mav.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.5)
        if rc:
            vals = [rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                    rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw]

            elapsed = time.time() - t0
            if last_vals is None:
                last_vals = vals
                print(f"[{elapsed:5.1f}s Baseline] Roll={vals[0]:4d} Pitch={vals[1]:4d} Thr={vals[2]:4d} Yaw={vals[3]:4d} | Ch5={vals[4]:4d} Ch6={vals[5]:4d} Ch7={vals[6]:4d} Ch8={vals[7]:4d}")
            else:
                diffs = [abs(a - b) for a, b in zip(vals, last_vals)]
                if any(d >= 10 for d in diffs):
                    change_count += 1
                    diff_str = " ".join([f"d{i+1}:{diffs[i]}" for i in range(4) if diffs[i] >= 10])
                    print(f"[{elapsed:5.1f}s LIVE MOVE #{change_count:02d}] Roll={vals[0]:4d} Pitch={vals[1]:4d} Thr={vals[2]:4d} Yaw={vals[3]:4d} | {diff_str}")
                    last_vals = vals

    print("="*75)
    print(f"Monitoring complete. Total stick changes captured: {change_count}")
    if change_count > 0:
        print(">> SUCCESS: Flight Controller is actively receiving and updating live stick movements!")
    else:
        print(">> WARNING: No stick changes detected during the window.")

if __name__ == "__main__":
    main()
