#!/usr/bin/env python3
"""
Clean In-Place 16-Channel Visualizer (No Scrolling, No Terminal Flood)
Displays all 16 channels with graphical progress bars updating in place.
"""

import sys
import os
import time
import json
import serial
from pymavlink import mavutil

GREEN  = "\033[1;32m"
CYAN   = "\033[1;36m"
YELLOW = "\033[1;33m"
RESET  = "\033[0m"
CLEAR  = "\033[H\033[2J"

def make_bar(val_us, width=28):
    # Clamps to 988 .. 2012 us (center = 1500)
    clamped = max(988, min(2012, val_us))
    ratio = (clamped - 988) / (2012 - 988)
    pos = int(ratio * width)
    center = width // 2

    bar = list(" " * (width + 1))
    bar[center] = "|" # Center marker at 1500
    if pos < center:
        for i in range(pos, center): bar[i] = "="
    else:
        for i in range(center + 1, min(pos + 1, width + 1)): bar[i] = "="
    return "".join(bar)

def main():
    mav = None
    try:
        mav = mavutil.mavlink_connection('tcp:10.94.163.186:5760')
        mav.wait_heartbeat(timeout=2)
    except Exception:
        mav = None

    s = None
    if not mav:
        try:
            s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        except Exception:
            pass

    labels = ["Roll/Ch1",  "Pitch/Ch2", "Thr/Ch3",   "Yaw/Ch4",
              "PotA/Ch5",  "Radio 6",   "Aux3/Ch7",  "Aux4/Ch8",
              "Radio 9",   "Radio 10",  "PotB/Ch11", "Radio 12",
              "Radio 13",  "Radio 14",  "Radio 15",  "Radio 16"]

    channels = [1500] * 16

    print(CLEAR)
    try:
        while True:
            # Read from MAVLink if available
            if mav:
                m = mav.recv_match(type='RC_CHANNELS', blocking=False)
                if m:
                    d = m.to_dict()
                    channels = [d[f'chan{i}_raw'] for i in range(1, 17)]
            elif s:
                l = s.readline()
                if b'RC_GROUND_IN' in l:
                    try:
                        data = json.loads(l.decode('utf-8', errors='replace').strip())
                        raw_ch = data.get('ch', [])
                        channels = [round((c - 992) * 5/8 + 1500) if c > 0 else 0 for c in raw_ch]
                    except Exception:
                        pass

            # Render in place
            out = ["\033[H"] # Move cursor to top-left
            out.append(f"{CYAN}======================================================================{RESET}")
            out.append(f"{CYAN}   DUAL-LRS REAL-TIME 16-CHANNEL MONITOR (Clean In-Place Display)     {RESET}")
            out.append(f"{CYAN}======================================================================{RESET}\n")

            for i in range(8):
                ch_left = i
                ch_right = i + 8

                v_left = channels[ch_left]
                bar_left = make_bar(v_left, 18)
                col_left = GREEN if abs(v_left - 1500) < 30 else YELLOW

                v_right = channels[ch_right]
                bar_right = make_bar(v_right, 18)
                col_right = GREEN if abs(v_right - 1500) < 30 else YELLOW

                line = f"{labels[ch_left]:10s} [{col_left}{bar_left}{RESET}] {v_left:4d} µs  |  {labels[ch_right]:10s} [{col_right}{bar_right}{RESET}] {v_right:4d} µs"
                out.append(line)

            out.append(f"\n{CYAN}----------------------------------------------------------------------{RESET}")
            out.append("Controls: Move sticks / toggle switches. Watch the bars move in real-time.")
            out.append("Press Ctrl+C to exit.")
            sys.stdout.write("\n".join(out))
            sys.stdout.flush()
            time.sleep(0.08) # 12 Hz refresh
    except KeyboardInterrupt:
        print("\nExiting monitor.")

if __name__ == "__main__":
    main()
