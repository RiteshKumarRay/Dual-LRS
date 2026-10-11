#!/usr/bin/env python3
"""
Dual-LRS Bench RF Cut-and-Restore Failsafe Verification Tool (High-Resolution)
Connects to Flight Controller via Radxa TCP bridge (tcp:10.233.23.186:5760).
Features:
  - Millisecond-precision timestamps (HH:MM:SS.mmm)
  - Records FC firmware, vehicle type, and failsafe parameters
  - Tracks exact elapsed milliseconds from last valid RC packet to FC failsafe declaration
  - Tracks exact elapsed milliseconds from first recovery packet to FC failsafe cleared
  - Monitors STATUSTEXT, SYS_STATUS sensor health, and channel values
"""

import sys
import time
from pymavlink import mavutil

GREEN  = "\033[1;32m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
CYAN   = "\033[1;36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

RADXA_TCP = "tcp:10.233.23.186:5760"

def get_ms_ts(t=None):
    if t is None: t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1)*1000):03d}"

def main():
    print(BOLD + "="*78 + RESET)
    print(BOLD + "   Dual-LRS High-Resolution Failsafe Verification Tool (ms precision)" + RESET)
    print(BOLD + "="*78 + RESET)
    print(f"Connecting to Flight Controller via Radxa ({RADXA_TCP})...")

    try:
        m = mavutil.mavlink_connection(RADXA_TCP)
        hb = m.wait_heartbeat(timeout=5)
        if not hb:
            print(RED + "ERROR: No heartbeat received from Flight Controller!" + RESET)
            sys.exit(1)
    except Exception as e:
        print(RED + f"Connection failed: {e}" + RESET)
        sys.exit(1)

    vehicle_types = {1: "Airplane", 2: "Quadcopter / Multirotor", 3: "Coaxial", 4: "Helicopter"}
    v_type = vehicle_types.get(hb.type, f"Type {hb.type}")
    print(GREEN + f"Connected! Vehicle: {v_type}, Autopilot: ArduPilot, System ID: {m.target_system}" + RESET)

    # Query Version & Failsafe Parameters
    fs_params = ['FS_THR_ENABLE', 'FS_THR_VALUE', 'FS_OPTIONS', 'DISARM_DELAY']
    for p in fs_params:
        m.mav.param_request_read_send(m.target_system, m.target_component, p.encode('utf-8'), -1)

    param_vals = {}
    t_param = time.time()
    while time.time() - t_param < 1.0:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
        if msg:
            pid = msg.param_id
            if isinstance(pid, bytes): pid = pid.decode()
            param_vals[pid.strip('\x00')] = msg.param_value

    param_str = ", ".join([f"{k}={v:.0f}" for k, v in param_vals.items()])
    print(f"FC Failsafe Config: {param_str}")

    # Request high-rate streams
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10, 1)

    rc_bit = 1 << 16  # MAV_SYS_STATUS_SENSOR_RC_RECEIVER
    last_rc_healthy = None
    in_failsafe = False
    last_rc_packet_time = None
    t_failsafe_engaged = None
    t_first_recovery_rc = None

    print("\n" + CYAN + "[BASELINE] Waiting for initial RC link state..." + RESET)

    last_print = 0

    while True:
        msg = m.recv_match(blocking=True, timeout=0.05)
        now = time.time()

        if msg:
            mtype = msg.get_type()

            if mtype == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes): text = text.decode(errors="ignore")
                severity = msg.severity
                ts = get_ms_ts(now)
                print(f"[{ts}] {YELLOW}STATUSTEXT (sev={severity}): {text}{RESET}")

            elif mtype == "SYS_STATUS":
                healthy = bool(msg.onboard_control_sensors_health & rc_bit)
                present = bool(msg.onboard_control_sensors_present & rc_bit)
                enabled = bool(msg.onboard_control_sensors_enabled & rc_bit)

                if last_rc_healthy is None:
                    last_rc_healthy = healthy
                    in_failsafe = not healthy
                    status_str = f"{GREEN}HEALTHY (LINK ACTIVE){RESET}" if healthy else f"{RED}FAILSAFE (NO LINK){RESET}"
                    print(f"\n[BASELINE CONFIRMED] RC Sensor: {status_str} | Present={present}, Enabled={enabled}")
                    print("\n" + BOLD + "TEST PROCEDURE (HANDSET-ONLY CUT OR RF CUT):" + RESET)
                    print("  1. Verify PROPELLERS ARE REMOVED.")
                    print("  2. When ready, TURN OFF HANDSET (or unplug Ground ESP32).")
                    print("  3. Observe high-resolution time-to-failsafe alert below.")
                    print("  4. After 5-10 seconds, TURN HANDSET ON (or re-plug Ground ESP32).")
                    print("  5. Observe high-resolution recovery alert below.\n" + "-"*78)

                elif healthy != last_rc_healthy:
                    ts = get_ms_ts(now)

                    if not healthy:
                        in_failsafe = True
                        t_failsafe_engaged = now
                        elapsed_from_rc = (now - last_rc_packet_time)*1000.0 if last_rc_packet_time else 0.0
                        print(f"\n[{ts}] {RED}>>> [FAILSAFE ENGAGED] RC Sensor Healthy -> FALSE <<<{RESET}")
                        if last_rc_packet_time:
                            print(f"       Elapsed from last valid RC frame to FC failsafe: {RED}{elapsed_from_rc:.1f} ms{RESET}")
                    else:
                        in_failsafe = False
                        t_recovery = now
                        fs_duration = (now - t_failsafe_engaged) if t_failsafe_engaged else 0.0
                        print(f"\n[{ts}] {GREEN}>>> [LINK RESTORED] RC Sensor Healthy -> TRUE <<<{RESET}")
                        print(f"       Total time in failsafe: {CYAN}{fs_duration:.3f} seconds{RESET}")
                        if t_first_recovery_rc:
                            rec_lag = (now - t_first_recovery_rc)*1000.0
                            print(f"       Time from first recovery RC frame to FC clear: {GREEN}{rec_lag:.1f} ms{RESET}")

                    last_rc_healthy = healthy

            elif mtype == "RC_CHANNELS":
                if in_failsafe and t_first_recovery_rc is None:
                    t_first_recovery_rc = now
                elif not in_failsafe:
                    t_first_recovery_rc = None

                last_rc_packet_time = now

                if now - last_print >= 1.0:
                    last_print = now
                    ts = get_ms_ts(now)
                    ch1 = msg.chan1_raw
                    ch2 = msg.chan2_raw
                    ch3 = msg.chan3_raw
                    ch4 = msg.chan4_raw
                    fs_tag = f"{RED}[FAILSAFE]{RESET}" if in_failsafe else f"{GREEN}[ACTIVE]{RESET}"
                    print(f"[{ts}] {fs_tag} Live RC: Roll={ch1} Pitch={ch2} Thr={ch3} Yaw={ch4}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nMonitor stopped by user.")
