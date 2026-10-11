#!/usr/bin/env python3
"""
Dual-LRS Saturated Traffic & RC Priority Bench Test
Monitors:
  - Ground GCS over-the-air endpoint: /dev/ttyUSB0 (baud 115200)
  - Vehicle FC companion truth endpoint: tcp:10.233.23.186:5760
Phases:
  1. Baseline Telemetry (Normal state): Measures normal ATTITUDE & telemetry rates over RF.
  2. Saturated Parameter Download: Sends MAVLink parameter requests over RF to saturate downlink.
     Verifies:
       - Streaming telemetry yields 100% to parameter frames
       - RC control maintains priority with zero starvation / zero failsafe
       - Parameter frames reassemble without corruption
  3. Post-Saturation Recovery: Verifies streaming telemetry immediately resumes after parameter burst.
"""

import sys
import time
import threading
from collections import defaultdict
from pymavlink import mavutil

GREEN  = "\033[1;32m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
CYAN   = "\033[1;36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

GND_PORT = "/dev/ttyUSB0"
RADXA_TCP = "tcp:10.233.23.186:5760"

def get_ms_ts():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1)*1000):03d}"

class TrafficMonitor:
    def __init__(self):
        self.running = True
        self.phase = "INIT"
        self.gnd_counts = defaultdict(int)
        self.fc_counts = defaultdict(int)
        self.param_values_received = 0
        self.bad_data_count = 0
        self.last_rc_fc_time = time.time()
        self.max_rc_gap_ms = 0.0
        self.rc_gaps = []
        self.failsafe_events = []
        self.lock = threading.Lock()

    def run_gnd(self, gnd):
        while self.running:
            try:
                msg = gnd.recv_match(blocking=True, timeout=0.05)
                if not msg:
                    continue
                mtype = msg.get_type()
                with self.lock:
                    if mtype == "BAD_DATA":
                        self.bad_data_count += 1
                    else:
                        self.gnd_counts[f"{self.phase}_{mtype}"] += 1
                        if mtype == "PARAM_VALUE":
                            self.param_values_received += 1
            except Exception as e:
                pass

    def run_fc(self, fc):
        last_rc_time = time.time()
        rc_bit = 1 << 16
        while self.running:
            try:
                msg = fc.recv_match(blocking=True, timeout=0.05)
                if not msg:
                    continue
                mtype = msg.get_type()
                now = time.time()
                with self.lock:
                    self.fc_counts[f"{self.phase}_{mtype}"] += 1
                    if mtype == "RC_CHANNELS":
                        gap = (now - last_rc_time) * 1000.0
                        if last_rc_time > 0 and gap > 5.0:
                            self.rc_gaps.append(gap)
                            if gap > self.max_rc_gap_ms:
                                self.max_rc_gap_ms = gap
                        last_rc_time = now
                    elif mtype == "SYS_STATUS":
                        healthy = bool(msg.onboard_control_sensors_health & rc_bit)
                        if not healthy:
                            self.failsafe_events.append((get_ms_ts(), "RC Unhealthy in SYS_STATUS"))
                    elif mtype == "STATUSTEXT":
                        text = msg.text
                        if isinstance(text, bytes): text = text.decode(errors="ignore")
                        if "failsafe" in text.lower():
                            self.failsafe_events.append((get_ms_ts(), text))
            except Exception as e:
                pass

def main():
    print(BOLD + "="*78 + RESET)
    print(BOLD + "   Dual-LRS Saturated Traffic & RC Priority Bench Test" + RESET)
    print(BOLD + "="*78 + RESET)

    print("Connecting to Ground station on", GND_PORT, "...")
    try:
        gnd = mavutil.mavlink_connection(GND_PORT, baud=115200)
        hb_gnd = gnd.wait_heartbeat(timeout=3)
        if not hb_gnd:
            print(RED + "ERROR: No heartbeat on " + GND_PORT + RESET)
            sys.exit(1)
        print(GREEN + f"Ground connected! sys={gnd.target_system}, comp={gnd.target_component}" + RESET)
    except Exception as e:
        print(RED + f"Ground connection failed: {e}" + RESET)
        sys.exit(1)

    print("Connecting to Vehicle FC via Radxa (" + RADXA_TCP + ")...")
    try:
        fc = mavutil.mavlink_connection(RADXA_TCP)
        hb_fc = fc.wait_heartbeat(timeout=3)
        if not hb_fc:
            print(RED + "ERROR: No heartbeat on " + RADXA_TCP + RESET)
            sys.exit(1)
        print(GREEN + f"Vehicle FC connected! sys={fc.target_system}, comp={fc.target_component}" + RESET)
    except Exception as e:
        print(RED + f"FC connection failed: {e}" + RESET)
        sys.exit(1)

    # Request FC streams
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10, 1)

    mon = TrafficMonitor()
    t_gnd = threading.Thread(target=mon.run_gnd, args=(gnd,), daemon=True)
    t_fc = threading.Thread(target=mon.run_fc, args=(fc,), daemon=True)
    t_gnd.start()
    t_fc.start()

    # PHASE 1: Baseline Normal Telemetry
    print("\n" + CYAN + BOLD + ">>> PHASE 1: Monitoring Baseline Normal Telemetry (8 seconds) <<<" + RESET)
    mon.phase = "BASELINE"
    t_start = time.time()
    while time.time() - t_start < 8.0:
        time.sleep(1.0)
        with mon.lock:
            att = mon.gnd_counts["BASELINE_ATTITUDE"]
            vfr = mon.gnd_counts["BASELINE_VFR_HUD"]
            elapsed = time.time() - t_start
            print(f"[{get_ms_ts()}] [BASELINE] Elapsed: {elapsed:.1f}s | Over-the-air ATTITUDE: {att} ({att/elapsed:.2f} Hz) | VFR_HUD: {vfr} ({vfr/elapsed:.2f} Hz)")

    # PHASE 2: Saturated Traffic
    print("\n" + YELLOW + BOLD + ">>> PHASE 2: Initiating Saturated Parameter Download over RF Link (12 seconds) <<<" + RESET)
    mon.phase = "SATURATED"
    t_sat_start = time.time()

    # Send parameter list request over Ground RF link
    print(f"[{get_ms_ts()}] Sending PARAM_REQUEST_LIST via {GND_PORT}...")
    gnd.mav.param_request_list_send(gnd.target_system, gnd.target_component)

    while time.time() - t_sat_start < 12.0:
        time.sleep(1.0)
        with mon.lock:
            elapsed_sat = time.time() - t_sat_start
            params = mon.param_values_received
            att_sat = mon.gnd_counts["SATURATED_ATTITUDE"]
            max_rc_gap = mon.max_rc_gap_ms
            fs_cnt = len(mon.failsafe_events)
            print(f"[{get_ms_ts()}] [SATURATED] Params Received: {params} | ATTITUDE Yielded Count: {att_sat} | Max RC Gap at FC: {max_rc_gap:.1f} ms | Failsafe Events: {fs_cnt}")

    # PHASE 3: Post-Saturation Telemetry Resumption
    print("\n" + CYAN + BOLD + ">>> PHASE 3: Monitoring Post-Transfer Telemetry Resumption (8 seconds) <<<" + RESET)
    mon.phase = "POST"
    t_post_start = time.time()
    while time.time() - t_post_start < 8.0:
        time.sleep(1.0)
        with mon.lock:
            elapsed_post = time.time() - t_post_start
            att_post = mon.gnd_counts["POST_ATTITUDE"]
            vfr_post = mon.gnd_counts["POST_VFR_HUD"]
            print(f"[{get_ms_ts()}] [POST-SAT] Elapsed: {elapsed_post:.1f}s | Resumed ATTITUDE: {att_post} ({att_post/elapsed_post:.2f} Hz) | VFR_HUD: {vfr_post} ({vfr_post/elapsed_post:.2f} Hz)")

    mon.running = False
    gnd.close()
    fc.close()

    print("\n" + BOLD + "="*78 + RESET)
    print(BOLD + "   Saturated Traffic & RC Priority Verification Summary" + RESET)
    print(BOLD + "="*78 + RESET)
    print(f"1. Parameters Downloaded over RF: {GREEN}{mon.param_values_received} parameters{RESET}")
    sat_att = mon.gnd_counts["SATURATED_ATTITUDE"]
    att_supp_str = "PASS (Yielded 100%)" if sat_att == 0 else f"Observed {sat_att} frames"
    print(f"2. Normal Telemetry Suppression during Transfer: {att_supp_str}")
    print(f"3. Over-the-air Corruption: {GREEN}{mon.bad_data_count} BAD_DATA frames{RESET}")
    print(f"4. Maximum RC Arrival Gap at FC during Saturation: {CYAN}{mon.max_rc_gap_ms:.1f} ms{RESET}")
    print(f"5. Failsafe Declared during Saturation: {GREEN if len(mon.failsafe_events)==0 else RED}{len(mon.failsafe_events)} events{RESET}")
    if mon.failsafe_events:
        for ts, ev in mon.failsafe_events:
            print(f"   - [{ts}] {ev}")
    print("="*78)

if __name__ == "__main__":
    main()
