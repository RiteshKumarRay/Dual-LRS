#!/usr/bin/env python3
"""
Dual-LRS Live RC Monitor & Channel Visualizer
Bridges OpenI6X CRSF on Ground (ESP32) -> E22 900MHz RF -> Air (STM32 BlackPill)
Displays real-time stick inputs, RF transmission, Air reception, and failsafe states.
"""

import sys
import os
import time
import json
import signal
import threading
import serial

# Color definitions
GREEN  = "\033[1;32m"
RED    = "\033[1;31m"
YELLOW = "\033[1;33m"
CYAN   = "\033[1;36m"
WHITE  = "\033[1;37m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"
CLEAR  = "\033[H\033[2J"

class RcState:
    def __init__(self):
        self.lock = threading.Lock()
        # Ground (ESP32) state
        self.ground_connected = False
        self.crsf_frames_in = 0
        self.rf_tx_sent = 0
        self.crsf_crc_errors = 0
        self.ground_rx_pin = 13
        self.ground_channels = [992] * 16
        self.ground_last_time = 0
        self.ground_fps = 0.0

        # Air (STM32) state
        self.air_connected = False
        self.rf_rx_received = 0
        self.air_failsafe = True
        self.failsafe_events = 0
        self.restore_events = 0
        self.air_channels = [992] * 16
        self.air_last_time = 0
        self.air_fps = 0.0

        # Rate tracking
        self.prev_ground_frames = 0
        self.prev_air_frames = 0
        self.prev_rate_calc_time = time.time()

# Calibration support
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "rc_calibration.json")

def load_calibration():
    default_limits = [(172, 992, 1811)] * 16
    labels = ["Roll", "Pitch", "Thr", "Yaw", "VRA", "VRB", "SwA", "SwB", "SwC", "SwD"] + [f"Ch{i+1}" for i in range(10, 16)]
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                d = json.load(f)
                for item in d.get("channels", []):
                    idx = item.get("idx", 0)
                    if 0 <= idx < 16:
                        default_limits[idx] = (item.get("raw_min", 172), item.get("raw_mid", 992), item.get("raw_max", 1811))
                        labels[idx] = item.get("label", labels[idx])
        except Exception:
            pass
    return default_limits, labels

calib_limits, calib_labels = load_calibration()

def crsf_to_us(crsf_val, ch_idx=0):
    """Convert 11-bit CRSF channel value to calibrated servo microseconds (1000..2000)."""
    if crsf_val == 0:
        return 1500  # Default neutral/uninitialized

    rmin, rmid, rmax = calib_limits[ch_idx] if ch_idx < len(calib_limits) else (172, 992, 1811)

    # 4-count deadband around center to filter analog pot crosstalk
    if abs(crsf_val - rmid) <= 4:
        return 1500

    if crsf_val < rmid:
        span = rmid - rmin
        if span <= 0: span = 1
        ratio = (crsf_val - rmin) / span
        us = 1000.0 + ratio * 500.0
    else:
        span = rmax - rmid
        if span <= 0: span = 1
        ratio = (crsf_val - rmid) / span
        us = 1500.0 + ratio * 500.0

    return int(max(980, min(2020, round(us))))

def render_bar(us_val, width=28):
    """Render an ASCII bar centered at 1500us."""
    clamped = max(1000, min(2000, us_val))
    norm = (clamped - 1000) / 1000.0  # 0.0 to 1.0
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

def ground_reader(ser_ground, state, stop_event):
    while not stop_event.is_set():
        try:
            line = ser_ground.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    ev = data.get("event")
                    with state.lock:
                        state.ground_connected = True
                        if ev == "RC_GROUND_IN":
                            state.crsf_frames_in = data.get("frames", 0)
                            state.rf_tx_sent = data.get("rf_tx", 0)
                            state.crsf_crc_errors = data.get("crc_err", 0)
                            ch = data.get("ch", [])
                            if len(ch) == 16:
                                state.ground_channels = ch
                            state.ground_last_time = time.time()
                        elif ev == "CRSF_PIN_AUTO_SWAP":
                            state.ground_rx_pin = data.get("rx_pin", 13)
                except json.JSONDecodeError:
                    pass
        except Exception:
            time.sleep(0.02)

CRSF_CRC8_TABLE = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
]

def crsf_crc(data):
    c = 0
    for b in data:
        c = CRSF_CRC8_TABLE[c ^ b]
    return c

def unpack11(data):
    bit_buf, bits, channels = 0, 0, []
    for b in data:
        bit_buf |= b << bits
        bits += 8
        while bits >= 11:
            channels.append(bit_buf & 0x7FF)
            bit_buf >>= 11
            bits -= 11
    return channels

def air_reader_crsf(ser_air, state, stop_event):
    buf = bytearray()
    while not stop_event.is_set():
        try:
            chunk = ser_air.read(50)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 26:
                if buf[0] != 0xC8 or buf[1] != 24 or buf[2] != 0x16:
                    buf.pop(0)
                    continue
                frame = buf[:26]
                calc = crsf_crc(frame[2:25])
                if calc == frame[25]:
                    ch = unpack11(frame[3:25])
                    with state.lock:
                        state.air_connected = True
                        state.rf_rx_received += 1
                        state.air_failsafe = False
                        if len(ch) == 16:
                            state.air_channels = ch
                        state.air_last_time = time.time()
                    del buf[:26]
                else:
                    buf.pop(0)
        except Exception:
            time.sleep(0.02)

def air_reader(ser_air, state, stop_event):
    while not stop_event.is_set():
        try:
            line = ser_air.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    ev = data.get("event")
                    with state.lock:
                        state.air_connected = True
                        if ev == "RC_AIR_RX":
                            state.rf_rx_received = data.get("rf_rx", 0)
                            state.air_failsafe = data.get("failsafe", True)
                            state.failsafe_events = data.get("fs_events", 0)
                            ch = data.get("ch", [])
                            if len(ch) == 16:
                                state.air_channels = ch
                            state.air_last_time = time.time()
                except json.JSONDecodeError:
                    pass
        except Exception:
            time.sleep(0.02)

def main():
    import serial.tools.list_ports
    ground_port = "/dev/ttyUSB0"
    air_port = None
    is_ftdi_crsf = False

    for p in serial.tools.list_ports.comports():
        desc = p.description.lower()
        hwid = p.hwid.lower()
        if "cp210" in desc or "cp210" in hwid or "10c4:ea60" in hwid:
            ground_port = p.device
        elif "ft232" in desc or "ftdi" in desc or "0403:6001" in hwid:
            air_port = p.device
            is_ftdi_crsf = True
        elif "blackpill" in desc or "0483:5740" in hwid:
            if not air_port:
                air_port = p.device
                is_ftdi_crsf = False

    if not air_port and os.path.exists("/dev/ttyACM0"):
        air_port = "/dev/ttyACM0"
        is_ftdi_crsf = False

    air_mode_str = "CRSF 420k via FTDI" if is_ftdi_crsf else "Telemetry JSON"
    print(f"{CYAN}Connecting to Ground ({ground_port}) and Air ({air_port} [{air_mode_str}])...{RESET}")
    try:
        ser_ground = serial.Serial()
        ser_ground.port = ground_port
        ser_ground.baudrate = 115200
        ser_ground.timeout = 0.1
        ser_ground.dtr = False
        ser_ground.rts = False
        ser_ground.open()
    except Exception as e:
        print(f"{RED}Failed to open Ground port {ground_port}: {e}{RESET}")
    ser_air = None
    if air_port:
        try:
            air_baud = 420000 if is_ftdi_crsf else 115200
            ser_air = serial.Serial(air_port, air_baud, timeout=0.1)
            ser_air.dtr = True
            ser_air.rts = True
        except Exception as e:
            print(f"{YELLOW}Air port {air_port} not available: {e} (Running in Ground-only mode){RESET}")

    # Allow ESP32 to settle boot state cleanly
    time.sleep(0.5)
    ser_ground.reset_input_buffer()
    ser_ground.write(b"CMD:RC_START_GROUND\n")
    if ser_air and not is_ftdi_crsf:
        ser_air.reset_input_buffer()
        ser_air.write(b"CMD:RC_START_AIR\n")

    state = RcState()
    stop_event = threading.Event()

    t_ground = threading.Thread(target=ground_reader, args=(ser_ground, state, stop_event), daemon=True)
    t_ground.start()
    if ser_air:
        target_fn = air_reader_crsf if is_ftdi_crsf else air_reader
        t_air = threading.Thread(target=target_fn, args=(ser_air, state, stop_event), daemon=True)
        t_air.start()

    def signal_handler(sig, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    ch_names = ["Roll", "Pitch", "Thr", "Yaw"]

    try:
        while not stop_event.is_set():
            time.sleep(0.08)  # ~12.5 Hz refresh rate
            now = time.time()

            with state.lock:
                # Calculate frame rates
                dt = now - state.prev_rate_calc_time
                if dt >= 1.0:
                    state.ground_fps = (state.crsf_frames_in - state.prev_ground_frames) / dt
                    state.air_fps = (state.rf_rx_received - state.prev_air_frames) / dt
                    state.prev_ground_frames = state.crsf_frames_in
                    state.prev_air_frames = state.rf_rx_received
                    state.prev_rate_calc_time = now

                g_connected = (now - state.ground_last_time < 1.0) and (state.crsf_frames_in > 0)
                a_connected = (now - state.air_last_time < 1.5) and state.air_connected
                g_frames = state.crsf_frames_in
                g_tx = state.rf_tx_sent
                g_crc_err = state.crsf_crc_errors
                g_pin = state.ground_rx_pin
                g_fps = state.ground_fps

                a_rx = state.rf_rx_received
                a_fs = state.air_failsafe
                a_fs_events = state.failsafe_events
                a_fps = state.air_fps

                g_ch = list(state.ground_channels)
                a_ch = list(state.air_channels)

            # Build UI output
            buf = []
            buf.append(CLEAR)
            buf.append(f"{BOLD}{CYAN}========================================================================={RESET}")
            buf.append(f"{BOLD}{WHITE}              DUAL-LRS REAL-TIME RC TRANSPORT MONITOR                    {RESET}")
            buf.append(f"{BOLD}{CYAN}========================================================================={RESET}")

            # Handset Ingest Status
            if g_frames == 0:
                handset_status = f"{YELLOW}[ WAITING FOR TRANSMITTER ]{RESET} (Turn on FS-i6X with OpenI6X CRSF)"
            else:
                handset_status = f"{GREEN}[ ACTIVE ]{RESET} Ingesting OpenI6X CRSF on ESP32 GPIO {g_pin}"

            total_eval = g_frames + g_crc_err
            err_pct = (g_crc_err / total_eval * 100.0) if total_eval > 0 else 0.0
            health_color = RED if err_pct > 5.0 else GREEN
            crc_str = f"{health_color}{100.0 - err_pct:5.1f}% Health ({g_crc_err} err){RESET}"
            buf.append(f"{BOLD}Handset Ingest:{RESET}  {handset_status}")
            buf.append(f"  CRSF Frames In : {CYAN}{g_frames:6d}{RESET}  | Rate: {GREEN}{g_fps:5.1f} Hz{RESET} | Link Quality: {crc_str}")
            buf.append(f"  Ground RF TX   : {CYAN}{g_tx:6d}{RESET} pkts | Active CRSF Pin: GPIO {g_pin}")
            buf.append("")

            # Air Link Status
            if not ser_air:
                air_status = f"{YELLOW}[ GROUND-ONLY BENCH MODE ]{RESET}"
            elif not a_connected:
                air_status = f"{YELLOW}[ AIR DISCONNECTED / WAITING ]{RESET}"
            elif a_rx == 0:
                air_status = f"{YELLOW}[ WAITING FOR RF PACKETS ]{RESET}"
            elif a_fs:
                air_status = f"{RED}[ FAILSAFE / NO SIGNAL ]{RESET}"
            else:
                air_status = f"{GREEN}[ RF LINK ACTIVE ]{RESET}"

            buf.append(f"{BOLD}Air Receiver:  {RESET}  {air_status}")
            buf.append(f"  Air RF RX      : {CYAN}{a_rx:6d}{RESET} pkts | Rate: {GREEN}{a_fps:5.1f} Hz{RESET} | Failsafe Events: {a_fs_events}")
            buf.append(f"{BOLD}{CYAN}-------------------------------------------------------------------------{RESET}")

            # Channels 1-4 (Primary Sticks) Visualizer
            buf.append(f"{BOLD}{WHITE}PRIMARY CHANNELS (Sticks):{RESET}")
            for i in range(4):
                g_val = g_ch[i]
                a_val = a_ch[i]
                g_us = crsf_to_us(g_val, i)
                a_us = crsf_to_us(a_val, i)
                bar = render_bar(g_us, width=20)

                lbl = calib_labels[i] if i < len(calib_labels) else f"Ch {i+1}"
                if a_connected and a_rx > 0:
                    delta_us = abs(g_us - a_us)
                    status_str = f"[{GREEN}MATCH{RESET}]" if delta_us <= 4 else f"[{YELLOW}lag={delta_us}us{RESET}]"
                    air_part = f"AIR: {WHITE}{a_us:4d} us{RESET} ({a_val:4d}) {status_str}"
                elif a_connected:
                    air_part = f"{YELLOW}[AIR FAILSAFE]{RESET}"
                else:
                    air_part = f"{DIM}[AIR OFFLINE]{RESET}"

                buf.append(f"  Ch {i+1:2d} ({lbl:11s}): {bar}  GND: {CYAN}{g_us:4d} us{RESET} ({g_val:4d})  ->  {air_part}")

            buf.append("")
            # Channels 5-10 (Switches / Pots)
            buf.append(f"{BOLD}{WHITE}AUXILIARY CHANNELS (Pots & Switches 5-10):{RESET}")
            for i in range(4, 10):
                g_val = g_ch[i]
                a_val = a_ch[i]
                g_us = crsf_to_us(g_val, i)
                a_us = crsf_to_us(a_val, i)
                bar = render_bar(g_us, width=20)

                lbl = calib_labels[i] if i < len(calib_labels) else f"Ch {i+1}"
                if a_connected and a_rx > 0:
                    delta_us = abs(g_us - a_us)
                    status_str = f"[{GREEN}MATCH{RESET}]" if delta_us <= 4 else f"[{YELLOW}lag={delta_us}us{RESET}]"
                    air_part = f"AIR: {WHITE}{a_us:4d} us{RESET} ({a_val:4d}) {status_str}"
                elif a_connected:
                    air_part = f"{YELLOW}[AIR FAILSAFE]{RESET}"
                else:
                    air_part = f"{DIM}[AIR OFFLINE]{RESET}"

                buf.append(f"  Ch {i+1:2d} ({lbl:11s}): {bar}  GND: {CYAN}{g_us:4d} us{RESET} ({g_val:4d})  ->  {air_part}")

            # Channels 11-16 (Summary)
            gnd_str = " ".join(f"{crsf_to_us(g_ch[k], k):4d}" for k in range(10, 16))
            air_str = " ".join(f"{crsf_to_us(a_ch[k], k):4d}" for k in range(10, 16)) if (a_connected and a_rx > 0) else "N/A"
            buf.append(f"\n  Ch 11..16 GND : {CYAN}{gnd_str}{RESET}")
            buf.append(f"  Ch 11..16 AIR : {WHITE}{air_str}{RESET}")

            buf.append(f"{BOLD}{CYAN}========================================================================={RESET}")
            buf.append(f"{DIM}Press Ctrl+C to stop monitoring.{RESET}\n")

            sys.stdout.write("\n".join(buf))
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{YELLOW}Stopping RC monitor session...{RESET}")
        try:
            ser_ground.write(b"CMD:STOP\n")
            if ser_air:
                ser_air.write(b"CMD:STOP\n")
                ser_air.close()
            ser_ground.close()
        except Exception:
            pass
        print(f"{GREEN}Monitoring session ended cleanly.{RESET}")

if __name__ == "__main__":
    main()
