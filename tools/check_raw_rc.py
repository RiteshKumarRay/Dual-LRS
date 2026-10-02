#!/usr/bin/env python3
"""
Dual-LRS Real-Time Raw Hardware Pin & Byte Sniffer
Displays electrical levels, transition edges, raw UART bytes, and hex streams
on ESP32 GPIO 13 and GPIO 14 to diagnose transmitter connections live.
"""

import sys
import time
import json
import serial

GREEN  = "\033[1;32m"
RED    = "\033[1;31m"
YELLOW = "\033[1;33m"
CYAN   = "\033[1;36m"
WHITE  = "\033[1;37m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

def main():
    port = "/dev/ttyUSB0"
    baud = 115200

    print(f"{CYAN}Connecting to Ground ESP32 on {port}...{RESET}")
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:
        print(f"{RED}Cannot open {port}: {e}{RESET}")
        sys.exit(1)

    time.sleep(0.3)
    # Start Ground RC Mode
    ser.write(b"CMD:RC_START_GROUND\n")
    print(f"{GREEN}Sent CMD:RC_START_GROUND. Listening for raw GPIO 13/14 transitions and bytes...{RESET}\n")

    print(f"{BOLD}{WHITE}{'TIME':8s} | {'GPIO 13':18s} | {'GPIO 14':18s} | {'SCANNING / UART':25s} | {'RAW BYTES':10s} | {'HEX DUMP'}{RESET}")
    print("-" * 105)

    start_time = time.time()
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue

            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    ev = data.get("event")

                    if ev == "RC_GROUND_IN":
                        elapsed = time.time() - start_time
                        p13_lvl = data.get("p13_lvl", 0)
                        p13_edges = data.get("p13_edges", 0)
                        p14_lvl = data.get("p14_lvl", 0)
                        p14_edges = data.get("p14_edges", 0)
                        rx_pin = data.get("rx_pin", 13)
                        cur_baud = data.get("baud", 420000)
                        inv = data.get("invert", False)
                        raw_bytes = data.get("raw_bytes", 0)
                        hex_str = data.get("hex", "")
                        frames = data.get("frames", 0)

                        # Highlight pin with edge transitions
                        if p13_edges > 0:
                            s13 = f"{GREEN}LVL:{p13_lvl} EDGES:{p13_edges:4d}{RESET}"
                        else:
                            s13 = f"LVL:{p13_lvl} EDGES:{p13_edges:4d}"

                        if p14_edges > 0:
                            s14 = f"{GREEN}LVL:{p14_lvl} EDGES:{p14_edges:4d}{RESET}"
                        else:
                            s14 = f"LVL:{p14_lvl} EDGES:{p14_edges:4d}"

                        inv_str = "INV" if inv else "NORM"
                        cfg_str = f"RX:{rx_pin} @ {cur_baud//1000}k ({inv_str})"

                        if frames > 0:
                            bytes_str = f"{GREEN}{raw_bytes:6d} (CRSF!){RESET}"
                        elif raw_bytes > 0:
                            bytes_str = f"{YELLOW}{raw_bytes:6d}{RESET}"
                        else:
                            bytes_str = f"{DIM}{raw_bytes:6d}{RESET}"

                        hex_disp = hex_str if hex_str else f"{DIM}(no bytes){RESET}"

                        print(f"{elapsed:6.1f}s  | {s13:27s} | {s14:27s} | {cfg_str:25s} | {bytes_str:18s} | {hex_disp}")

                    elif ev == "CRSF_SCAN_TRY":
                        rx = data.get("rx_pin")
                        bd = data.get("baud")
                        iv = data.get("invert")
                        print(f"{YELLOW}>>> AUTO-SCAN SWITCH: Testing GPIO {rx} at {bd} baud (inverted={iv}){RESET}")

                    elif ev == "CRSF_LOCKED":
                        rx = data.get("rx_pin")
                        bd = data.get("baud")
                        iv = data.get("invert")
                        print(f"{GREEN}{BOLD}>>> CRSF LOCKED: Valid frames found on GPIO {rx} at {bd} baud (inverted={iv})! <<<{RESET}")

                except json.JSONDecodeError:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{YELLOW}Stopping and closing port...{RESET}")
        try:
            ser.write(b"CMD:STOP\n")
            time.sleep(0.2)
            ser.close()
        except Exception:
            pass
        print(f"{GREEN}Done.{RESET}")

if __name__ == "__main__":
    main()
