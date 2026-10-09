#!/usr/bin/env python3
"""
Ultra-RC 55-60 Hz Live Bench Latency & Smoothness Monitor
Tracks Ground ESP32 (/dev/ttyUSB0), Air BlackPill (/dev/ttyACM0), and FC MAVLink (TCP 5760).
Displays in-place high-refresh ASCII progress bars and end-to-end latency metrics.
"""

import sys
import os
import time
import json
import threading
import serial

GREEN  = "\033[1;32m"
CYAN   = "\033[1;36m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
RESET  = "\033[0m"
CLEAR  = "\033[H\033[2J"

def make_bar(val_us, width=28):
    clamped = max(988, min(2012, val_us))
    ratio = (clamped - 988) / (2012 - 988)
    pos = int(ratio * width)
    center = width // 2

    bar = list(" " * (width + 1))
    bar[center] = "|"
    if pos < center:
        for i in range(pos, center): bar[i] = "="
    else:
        for i in range(center + 1, min(pos + 1, width + 1)): bar[i] = "="
    return "".join(bar)

def crsf_to_us(counts):
    # CRSF count (172..1811) -> Microseconds (988..2012 us, 992 -> 1500 us)
    return int(988 + (counts - 172) * (2012 - 988) / (1811 - 172))

def main():
    ground_port = "/dev/ttyUSB0"
    air_port = "/dev/ttyACM0"

    ground_ser = None
    air_ser = None

    ground_channels = [992] * 16
    air_channels = [992] * 16
    ground_tx_count = 0
    air_rx_count = 0
    last_ground_time = 0
    last_air_time = 0
    air_dt_us = 0
    ground_fps = 0.0
    air_fps = 0.0

    print(CLEAR)

    t_calc = time.time()
    g_pkts_sec = 0
    a_pkts_sec = 0
    last_reconnect_time = 0

    try:
        while True:
            t_now = time.time()

            # Dynamic port reconnection
            if t_now - last_reconnect_time >= 1.0:
                last_reconnect_time = t_now
                if ground_ser is None and os.path.exists(ground_port):
                    try:
                        ground_ser = serial.Serial(ground_port, 115200, timeout=0.01)
                    except:
                        ground_ser = None
                if air_ser is None and os.path.exists(air_port):
                    try:
                        air_ser = serial.Serial(air_port, 115200, timeout=0.01)
                    except:
                        air_ser = None

            # Read Ground
            if ground_ser:
                try:
                    while ground_ser.in_waiting > 0:
                        l = ground_ser.readline().decode('utf-8', errors='ignore').strip()
                        if 'ULTRA_RC_GROUND' in l or 'PURE_RC_GROUND' in l or 'RC_GROUND_IN' in l:
                            d = json.loads(l)
                            ground_channels = d.get('ch', ground_channels)
                            ground_tx_count = d.get('rf_tx', ground_tx_count)
                            last_ground_time = t_now
                            g_pkts_sec += 1
                except:
                    ground_ser = None

            # Read Air
            if air_ser:
                try:
                    while air_ser.in_waiting > 0:
                        l = air_ser.readline().decode('utf-8', errors='ignore').strip()
                        if 'ULTRA_RC_AIR' in l or 'PURE_RC_AIR' in l or 'RC_AIR_RX' in l:
                            d = json.loads(l)
                            air_channels = d.get('ch', air_channels)
                            air_rx_count = d.get('rx', d.get('rf_rx', air_rx_count))
                            air_dt_us = d.get('dt_us', d.get('dt_rf_us', air_dt_us))
                            last_air_time = t_now
                            a_pkts_sec += 1
                except:
                    air_ser = None

            # FPS calculate every second
            if t_now - t_calc >= 1.0:
                ground_fps = g_pkts_sec / (t_now - t_calc)
                air_fps = a_pkts_sec / (t_now - t_calc)
                g_pkts_sec = 0
                a_pkts_sec = 0
                t_calc = t_now

            # Render display at ~30 FPS
            sys.stdout.write("\033[H")
            print(f"{GREEN}========================================================================{RESET}")
            print(f"   {GREEN}ULTRA-RC 55.5 HZ HIGH-SPEED LINK BENCH MONITOR{RESET}")
            print(f"{GREEN}========================================================================{RESET}")
            print(f" Ground TX (ESP32):   {CYAN}{ground_tx_count:6d} pkts{RESET}  Cadence: {CYAN}18.0 ms{RESET} ({ground_fps:4.1f} Hz report)")
            air_alive = (t_now - last_air_time < 0.5 and air_rx_count > 0)
            air_status = f"{GREEN}ACTIVE (dt={air_dt_us/1000.0:4.1f}ms / {1000000.0/air_dt_us if air_dt_us>0 else 0:4.1f}Hz){RESET}" if air_alive else f"{RED}WAITING AIR USB (/dev/ttyACM0){RESET}"
            print(f" Air RX (BlackPill):  {YELLOW}{air_rx_count:6d} pkts{RESET}  Status:  {air_status}")
            
            active_src = "AIR UNIT (OVER-THE-AIR RF)" if air_alive else "GROUND UNIT (HANDSET INGEST)"
            active_src_color = GREEN if air_alive else CYAN
            print(f" Displayed Channels:  {active_src_color}{active_src}{RESET}")
            print(f"------------------------------------------------------------------------")
            print(f" CH   NAME        INPUT COUNTS   PULSE (us)   STICK GRAPH [1000 .. 2000 us]")
            print(f"------------------------------------------------------------------------")

            labels = [
                "CH1  (Roll)      ", "CH2  (Pitch)     ", "CH3  (Throttle)  ", "CH4  (Yaw)       ",
                "CH5  (Mode/PotA) ", "CH6  (Parked)    ", "CH7  (Aux1)      ", "CH8  (Aux2)      ",
                "CH9  (Aux3)      ", "CH10 (Aux4)      ", "CH11 (Aux5/PotB) ", "CH12 (Aux6)      ",
                "CH13 (Aux7)      ", "CH14 (Aux8)      ", "CH15 (Aux9)      ", "CH16 (Aux10)     "
            ]

            disp_channels = air_channels if air_alive else ground_channels
            for i in range(16):
                cnt = disp_channels[i] if i < len(disp_channels) else 992
                us_val = crsf_to_us(cnt)
                bar = make_bar(us_val, width=24)
                print(f" {i+1:2d}   {labels[i]}  {cnt:5d} counts  {us_val:5d} us   [{bar}]")

            print(f"------------------------------------------------------------------------")
            print(f" Move sticks on FS-i6X to see immediate, zero-lag response.")
            print(f" Press Ctrl+C to exit.")
            sys.stdout.flush()
            time.sleep(0.033)

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if ground_ser: ground_ser.close()
        if air_ser: air_ser.close()

if __name__ == "__main__":
    main()
