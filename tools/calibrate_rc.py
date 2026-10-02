#!/usr/bin/env python3
"""
Dual-LRS Interactive 10-Channel RC Calibration & Mapping Tool
Calibrates FlySky FS-i6X / OpenI6X sticks, pots (VRA/VRB), and switches (SwA-SwD),
eliminates ADC crosstalk/jitter, and saves calibrated microsecond mappings.
"""

import sys
import os
import time
import json
import threading
import signal
import termios
import tty
import select
import serial

GREEN  = "\033[1;32m"
RED    = "\033[1;31m"
YELLOW = "\033[1;33m"
CYAN   = "\033[1;36m"
WHITE  = "\033[1;37m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"
CLEAR  = "\033[2J\033[H"

CONFIG_FILE = os.path.join(os.path.dirname(__file__), "rc_calibration.json")

DEFAULT_LABELS = [
    "Roll (Ail)",
    "Pitch (Ele)",
    "Throttle",
    "Yaw (Rud)",
    "Aux 1 (VRA)",
    "Aux 2 (VRB)",
    "SwA",
    "SwB",
    "SwC",
    "SwD",
    "Ch 11",
    "Ch 12",
    "Ch 13",
    "Ch 14",
    "Ch 15",
    "Ch 16"
]

class ChannelCalibration:
    def __init__(self, ch_idx, label=""):
        self.idx = ch_idx
        self.label = label if label else f"Ch {ch_idx+1}"
        self.raw_min = 172
        self.raw_mid = 992
        self.raw_max = 1811
        self.deadband = 4  # Raw count deadband around center
        self.is_throttle = False

    def to_dict(self):
        return {
            "idx": self.idx,
            "label": self.label,
            "raw_min": self.raw_min,
            "raw_mid": self.raw_mid,
            "raw_max": self.raw_max,
            "deadband": self.deadband,
            "is_throttle": self.is_throttle
        }

    @classmethod
    def from_dict(cls, d):
        c = cls(d.get("idx", 0), d.get("label", ""))
        c.raw_min = d.get("raw_min", 172)
        c.raw_mid = d.get("raw_mid", 992)
        c.raw_max = d.get("raw_max", 1811)
        c.deadband = d.get("deadband", 4)
        c.is_throttle = d.get("is_throttle", False)
        return c

    def raw_to_us(self, raw_val):
        """Piecewise linear 3-point calibration: Min -> 1000us, Mid -> 1500us, Max -> 2000us."""
        if raw_val <= 0:
            return 1500

        # Deadband around center
        if abs(raw_val - self.raw_mid) <= self.deadband:
            return 1500

        if raw_val < self.raw_mid:
            span = self.raw_mid - self.raw_min
            if span <= 0:
                span = 1
            ratio = (raw_val - self.raw_min) / span
            us = 1000.0 + ratio * 500.0
        else:
            span = self.raw_max - self.raw_mid
            if span <= 0:
                span = 1
            ratio = (raw_val - self.raw_mid) / span
            us = 1500.0 + ratio * 500.0

        return int(max(980, min(2020, round(us))))

def load_calibration():
    calibs = [ChannelCalibration(i, DEFAULT_LABELS[i] if i < len(DEFAULT_LABELS) else f"Ch {i+1}") for i in range(16)]
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                for item in data.get("channels", []):
                    idx = item.get("idx", 0)
                    if 0 <= idx < 16:
                        calibs[idx] = ChannelCalibration.from_dict(item)
        except Exception:
            pass
    return calibs

def save_calibration(calibs):
    data = {
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "channels": [c.to_dict() for c in calibs]
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=2)

def render_bar(us_val, width=24):
    clamped = max(1000, min(2000, us_val))
    norm = (clamped - 1000) / 1000.0
    pos = int(round(norm * (width - 1)))
    mid = int(round(0.5 * (width - 1)))

    chars = [' '] * width
    chars[mid] = '|'
    if pos < mid:
        for i in range(pos, mid):
            chars[i] = '#'
    elif pos > mid:
        for i in range(mid + 1, pos + 1):
            chars[i] = '#'
    else:
        chars[mid] = 'O'
    return '[' + ''.join(chars) + ']'

def main():
    port = "/dev/ttyUSB0"
    print(f"{CYAN}Connecting to Ground ESP32 on {port}...{RESET}")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 0.1
        ser.dtr = False
        ser.rts = False
        ser.open()
    except Exception as e:
        print(f"{RED}Cannot open {port}: {e}{RESET}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"CMD:RC_START_GROUND\n")

    calibs = load_calibration()
    live_channels = [992] * 16
    min_seen = [9999] * 16
    max_seen = [0] * 16
    lock = threading.Lock()
    stop_event = threading.Event()

    def reader():
        while not stop_event.is_set():
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                if line.startswith('{'):
                    data = json.loads(line)
                    if data.get("event") == "RC_GROUND_IN":
                        ch = data.get("ch", [])
                        if len(ch) == 16:
                            with lock:
                                for i in range(16):
                                    live_channels[i] = ch[i]
                                    if ch[i] > 0:
                                        if ch[i] < min_seen[i]: min_seen[i] = ch[i]
                                        if ch[i] > max_seen[i]: max_seen[i] = ch[i]
            except Exception:
                pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    def cleanup():
        stop_event.set()
        try:
            ser.write(b"CMD:STOP\n")
            time.sleep(0.2)
            ser.close()
        except Exception:
            pass

    def run_wizard():
        print(f"\n{BOLD}{YELLOW}=== STEP 1: NEUTRAL / CENTER CALIBRATION ==={RESET}")
        print("1. Leave all 4 sticks in their center rest positions (put Throttle at center if spring-centered, or leave at bottom).")
        print("2. Set both knobs VRA and VRB to the 12 o'clock (center) notch.")
        print("3. Set switches to their default positions.")
        input(f"\n{CYAN}Press [ENTER] when controls are centered...{RESET}")

        print(f"{GREEN}Sampling centers for 2 seconds...{RESET}")
        center_samples = [[] for _ in range(16)]
        t_end = time.time() + 2.0
        while time.time() < t_end:
            with lock:
                for i in range(16):
                    if live_channels[i] > 0:
                        center_samples[i].append(live_channels[i])
            time.sleep(0.05)

        for i in range(16):
            if center_samples[i]:
                avg_center = int(round(sum(center_samples[i]) / len(center_samples[i])))
                calibs[i].raw_mid = avg_center

        print(f"{GREEN}Center positions recorded successfully!{RESET}\n")

        print(f"{BOLD}{YELLOW}=== STEP 2: FULL TRAVEL / ENDPOINT CALIBRATION ==={RESET}")
        print("Slowly stir both gimbals in full circles, rotate VRA & VRB fully end-to-end, and toggle all switches.")
        print("Press [ENTER] when you have moved all controls to their limits.")
        input(f"\n{CYAN}Press [ENTER] to start 8-second travel scan...{RESET}")

        t_end = time.time() + 8.0
        while time.time() < t_end:
            rem = int(t_end - time.time())
            with lock:
                ch_disp = " | ".join(f"C{k+1}:{live_channels[k]}" for k in range(6))
            sys.stdout.write(f"\r{YELLOW}Scanning limits... {rem:2d}s remaining | {ch_disp}{RESET}    ")
            sys.stdout.flush()
            time.sleep(0.1)

        with lock:
            for i in range(16):
                if min_seen[i] < 9999 and max_seen[i] > 0 and (max_seen[i] - min_seen[i] > 100):
                    calibs[i].raw_min = min_seen[i]
                    calibs[i].raw_max = max_seen[i]

        save_calibration(calibs)
        print(f"\n\n{GREEN}{BOLD}>>> Calibration successfully saved to {CONFIG_FILE}! <<<{RESET}\n")
        time.sleep(2.0)

    # Check CLI argument
    if "--wizard" in sys.argv or "-w" in sys.argv:
        time.sleep(1.0)
        run_wizard()

    print(f"{GREEN}Live Calibration Monitor active. Press 'c' for wizard, 'q' to quit.{RESET}")
    time.sleep(1.0)

    try:
        # Set terminal to non-blocking for keypress
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        while True:
            # Check keyboard input
            r, _, _ = select.select([sys.stdin], [], [], 0.08)
            if r:
                ch = sys.stdin.read(1)
                if ch in ('q', 'Q', '\x03'):
                    break
                elif ch in ('c', 'C'):
                    # Restore terminal temporarily
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    run_wizard()
                    tty.setcbreak(sys.stdin.fileno())

            with lock:
                ch_copy = list(live_channels)

            buf = [CLEAR]
            buf.append(f"{BOLD}{CYAN}================================================================================={RESET}")
            buf.append(f"{BOLD}{WHITE}             DUAL-LRS 10-CHANNEL CALIBRATED RC MONITOR                           {RESET}")
            buf.append(f"{BOLD}{CYAN}================================================================================={RESET}")
            buf.append(f"{DIM}Press [C] to run Calibration Wizard | Press [Q] or Ctrl+C to quit{RESET}\n")

            buf.append(f"{BOLD}{WHITE}{'CH':4s} | {'FUNCTION / NAME':15s} | {'POSITION':26s} | {'CALIBRATED':11s} | {'RAW':6s} | {'LIMITS (MIN..MID..MAX)'}{RESET}")
            buf.append("-" * 81)

            # Display active 10 channels of FS-i6X
            for i in range(10):
                c = calibs[i]
                raw = ch_copy[i]
                us = c.raw_to_us(raw)
                bar = render_bar(us)

                # Microsecond color coding
                if abs(us - 1500) <= 10:
                    us_str = f"{GREEN}{us:4d} us (CTR){RESET}"
                elif us <= 1010:
                    us_str = f"{CYAN}{us:4d} us (MIN){RESET}"
                elif us >= 1990:
                    us_str = f"{YELLOW}{us:4d} us (MAX){RESET}"
                else:
                    us_str = f"{WHITE}{us:4d} us      {RESET}"

                limits_str = f"{c.raw_min:4d} .. {c.raw_mid:4d} .. {c.raw_max:4d}"
                buf.append(f"Ch{i+1:2d} | {c.label:15s} | {bar} | {us_str} | {raw:5d}  | {limits_str}")

            buf.append(f"\n{BOLD}{CYAN}---------------------------------------------------------------------------------{RESET}")
            buf.append(f"{DIM}Note: Calibrated values map directly to 1000us (Min), 1500us (Mid), 2000us (Max).{RESET}")

            sys.stdout.write("\n".join(buf))
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        cleanup()
        print(f"\n{GREEN}Calibration monitor closed cleanly.{RESET}")

if __name__ == "__main__":
    main()
