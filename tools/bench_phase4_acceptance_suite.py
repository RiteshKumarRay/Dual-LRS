#!/usr/bin/env python3
"""
Dual-LRS Phase 4 Clean Transfer Acceptance Suite
Validates:
  - 10 Consecutive Full Parameter Downloads over RF
  - 10 Consecutive Mission Upload & Download Cycles over RF
  - Continuous Concurrent FC Truth Monitoring (Radxa TCP) for:
      * Zero RC Failsafes
      * Zero MCU Reboots
      * Continuous RC Sensor Health
"""

import sys
import time
import threading
from pymavlink import mavutil

GREEN  = "\033[1;32m"
YELLOW = "\033[1;33m"
RED    = "\033[1;31m"
CYAN   = "\033[1;36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

GND_PORT = "/dev/ttyUSB0"
RADXA_TCP = "tcp:10.233.23.186:5760"

TOTAL_PARAM_CYCLES = 10
TOTAL_MISSION_CYCLES = 10

class ConcurrentTruthMonitor:
    def __init__(self):
        self.running = True
        self.failsafe_events = []
        self.reboot_events = []
        self.rc_packet_count = 0
        self.last_rc_time = time.time()
        self.max_rc_gap_ms = 0.0

    def start(self, fc_conn):
        self.thread = threading.Thread(target=self._monitor_loop, args=(fc_conn,), daemon=True)
        self.thread.start()

    def _monitor_loop(self, fc):
        rc_bit = 1 << 16
        while self.running:
            try:
                msg = fc.recv_match(blocking=True, timeout=0.05)
                if not msg:
                    continue
                mtype = msg.get_type()
                now = time.time()
                if mtype == "RC_CHANNELS":
                    self.rc_packet_count += 1
                    gap = (now - self.last_rc_time) * 1000.0
                    if self.rc_packet_count > 1 and gap > self.max_rc_gap_ms:
                        self.max_rc_gap_ms = gap
                    self.last_rc_time = now
                elif mtype == "SYS_STATUS":
                    healthy = bool(msg.onboard_control_sensors_health & rc_bit)
                    if not healthy:
                        self.failsafe_events.append((time.time(), "RC Unhealthy in SYS_STATUS"))
                elif mtype == "STATUSTEXT":
                    text = msg.text
                    if isinstance(text, bytes): text = text.decode(errors="ignore")
                    if "failsafe" in text.lower():
                        self.failsafe_events.append((time.time(), f"Failsafe message: {text}"))
                    if "boot" in text.lower() or "init" in text.lower():
                        self.reboot_events.append((time.time(), f"Reboot indicator: {text}"))
            except Exception:
                pass

    def stop(self):
        self.running = False

def run_acceptance_suite():
    print(BOLD + "="*78 + RESET)
    print(BOLD + "DUAL-LRS PHASE 4 CLEAN TRANSFER ACCEPTANCE BENCH SUITE" + RESET)
    print(BOLD + "="*78 + RESET)
    print(f"Ground RF Link:   {GND_PORT} (115200 baud)")
    print(f"FC Truth Monitor: {RADXA_TCP}")
    print(f"Required Target:  10x Full Parameter Downloads + 10x Full Mission Transfers")
    print(f"Safety Gate:      ZERO Failsafes, ZERO Reboots, 100% Protocol Fidelity\n")

    # Connect to Ground GCS
    print(CYAN + "[1/3] Connecting to Ground Radio GCS port..." + RESET)
    try:
        gnd = mavutil.mavlink_connection(GND_PORT, baud=115200, source_system=255, source_component=190)
    except Exception as e:
        print(RED + f"FAILED to open {GND_PORT}: {e}" + RESET)
        sys.exit(1)

    # Connect to Radxa Truth
    print(CYAN + "[2/3] Connecting to FC Truth port via Radxa TCP..." + RESET)
    try:
        fc = mavutil.mavlink_connection(RADXA_TCP, source_system=254, source_component=189)
    except Exception as e:
        print(RED + f"FAILED to open {RADXA_TCP}: {e}" + RESET)
        sys.exit(1)

    truth_monitor = ConcurrentTruthMonitor()
    truth_monitor.start(fc)

    # Await Heartbeats
    print(CYAN + "[3/3] Waiting for Heartbeat from Flight Controller..." + RESET)
    hb = gnd.recv_match(type='HEARTBEAT', blocking=True, timeout=8.0)
    if not hb:
        print(RED + "ERROR: No Heartbeat received over RF within 8s!" + RESET)
        sys.exit(1)
    sysid = hb.get_srcSystem()
    compid = hb.get_srcComponent()
    print(GREEN + f"  -> FC Online over RF: SysID={sysid}, CompID={compid}" + RESET)

    # Clear backlog
    while gnd.recv_msg(): pass

    # =========================================================================
    # PART A: 10 CONSECUTIVE FULL PARAMETER DOWNLOADS
    # =========================================================================
    print("\n" + BOLD + "="*78 + RESET)
    print(BOLD + "PART A: 10 CONSECUTIVE FULL PARAMETER DOWNLOADS" + RESET)
    print(BOLD + "="*78 + RESET)

    param_results = []
    for cycle in range(1, TOTAL_PARAM_CYCLES + 1):
        print(f"\n>>> Parameter Download Cycle {cycle}/{TOTAL_PARAM_CYCLES} <<<")
        t_start = time.time()
        gnd.mav.param_request_list_send(sysid, compid)

        params_received = {}
        seen_indices = set()
        total_expected = None
        t_deadline = time.time() + 180.0  # 180s timeout per full parameter table (~1004 params @ ~7.5 params/s takes ~135s)

        while time.time() < t_deadline:
            msg = gnd.recv_match(type='PARAM_VALUE', blocking=True, timeout=3.0)
            if msg:
                param_id = msg.param_id
                if isinstance(param_id, bytes): param_id = param_id.decode(errors='ignore').rstrip('\x00')
                params_received[param_id] = msg.param_value
                seen_indices.add(msg.param_index)
                if total_expected is None:
                    total_expected = msg.param_count
                if total_expected and len(seen_indices) >= total_expected:
                    break
            else:
                # Stream paused or completed
                if total_expected and len(seen_indices) > 0:
                    break

        # Standard MAVLink recovery for any missed indices in bulk stream
        if total_expected and len(seen_indices) < total_expected:
            missing = [i for i in range(total_expected) if i not in seen_indices]
            print(YELLOW + f"    [INFO] Re-requesting {len(missing)} missed indices: {missing[:5]}..." + RESET)
            for idx in missing:
                gnd.mav.param_request_read_send(sysid, compid, b"", idx)
                retry_msg = gnd.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
                if retry_msg and retry_msg.param_index == idx:
                    p_id = retry_msg.param_id
                    if isinstance(p_id, bytes): p_id = p_id.decode(errors='ignore').rstrip('\x00')
                    params_received[p_id] = retry_msg.param_value
                    seen_indices.add(idx)

        elapsed = time.time() - t_start
        success = (total_expected is not None and len(seen_indices) >= total_expected)
        param_results.append({
            "cycle": cycle,
            "success": success,
            "elapsed": elapsed,
            "count": len(seen_indices),
            "expected": total_expected
        })

        if success:
            print(GREEN + f"  [PASS] Cycle {cycle}: Received all {len(seen_indices)}/{total_expected} params in {elapsed:.2f}s ({len(seen_indices)/elapsed:.1f} params/sec)" + RESET)
        else:
            print(RED + f"  [FAIL] Cycle {cycle}: Received {len(seen_indices)}/{total_expected} params in {elapsed:.2f}s (TIMED OUT)" + RESET)

        time.sleep(1.0)  # Inter-cycle settling delay

    # =========================================================================
    # PART B: 10 CONSECUTIVE MISSION UPLOAD & DOWNLOAD CYCLES
    # =========================================================================
    print("\n" + BOLD + "="*78 + RESET)
    print(BOLD + "PART B: 10 CONSECUTIVE MISSION UPLOAD & DOWNLOAD CYCLES" + RESET)
    print(BOLD + "="*78 + RESET)

    WP_COUNT = 4
    test_wps = [
        (int(19.0760 * 1e7), int(72.8777 * 1e7), 0.0),   # WP0
        (int(19.0765 * 1e7), int(72.8780 * 1e7), 20.0),  # WP1
        (int(19.0770 * 1e7), int(72.8785 * 1e7), 30.0),  # WP2
        (int(19.0775 * 1e7), int(72.8780 * 1e7), 25.0),  # WP3
    ]

    mission_results = []
    for cycle in range(1, TOTAL_MISSION_CYCLES + 1):
        print(f"\n>>> Mission Cycle {cycle}/{TOTAL_MISSION_CYCLES} <<<")
        t_cycle_start = time.time()

        # 1. Clear
        gnd.mav.mission_clear_all_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        t_clear_start = time.time()
        clear_ok = False
        while time.time() - t_clear_start < 3.0:
            msg = gnd.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
            if msg:
                clear_ok = True
                break

        # 2. Upload
        t_up_start = time.time()
        gnd.mav.mission_count_send(sysid, compid, WP_COUNT, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        upload_ok = False
        t_deadline = time.time() + 8.0
        while time.time() < t_deadline:
            msg = gnd.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=0.3)
            if msg:
                mtype = msg.get_type()
                if mtype in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                    seq = msg.seq
                    if seq < WP_COUNT:
                        lat, lon, alt = test_wps[seq]
                        gnd.mav.mission_item_int_send(
                            sysid, compid, seq,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                            0, 1, 0, 0, 0, 0,
                            lat, lon, alt,
                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                        )
                elif mtype == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        upload_ok = True
                        break

        up_time = time.time() - t_up_start

        # 3. Download
        t_down_start = time.time()
        gnd.mav.mission_request_list_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        downloaded = {}
        expected_count = 0
        t_deadline = time.time() + 8.0
        while time.time() < t_deadline:
            msg = gnd.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=0.3)
            if msg:
                mtype = msg.get_type()
                if mtype == 'MISSION_COUNT':
                    expected_count = msg.count
                    if expected_count > 0:
                        gnd.mav.mission_request_int_send(sysid, compid, 0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                elif mtype in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
                    downloaded[msg.seq] = msg
                    next_seq = msg.seq + 1
                    if next_seq < expected_count:
                        gnd.mav.mission_request_int_send(sysid, compid, next_seq, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                    else:
                        gnd.mav.mission_ack_send(sysid, compid, mavutil.mavlink.MAV_MISSION_ACCEPTED, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                        break

        down_time = time.time() - t_down_start
        download_ok = (len(downloaded) == WP_COUNT)

        # Integrity Check
        integrity_ok = True
        if download_ok:
            for seq, item in sorted(downloaded.items()):
                exp_lat, exp_lon, exp_alt = test_wps[seq]
                if seq > 0:
                    if item.x != exp_lat or item.y != exp_lon or abs(item.z - exp_alt) > 0.1:
                        integrity_ok = False

        cycle_ok = (upload_ok and download_ok and integrity_ok)
        mission_results.append({
            "cycle": cycle,
            "success": cycle_ok,
            "upload_time": up_time,
            "download_time": down_time,
            "items": len(downloaded)
        })

        if cycle_ok:
            print(GREEN + f"  [PASS] Cycle {cycle}: Upload {up_time:.2f}s | Download {down_time:.2f}s | 100% Integrity" + RESET)
        else:
            print(RED + f"  [FAIL] Cycle {cycle}: Upload={upload_ok} ({up_time:.2f}s), Download={download_ok} ({down_time:.2f}s), Integrity={integrity_ok}" + RESET)

        time.sleep(0.5)

    truth_monitor.stop()

    # =========================================================================
    # FINAL STATISTICAL SUMMARY & ACCEPTANCE VERIFICATION
    # =========================================================================
    print("\n" + BOLD + "="*78 + RESET)
    print(BOLD + "PHASE 4 BENCH ACCEPTANCE SCOREBOARD" + RESET)
    print(BOLD + "="*78 + RESET)

    param_success_count = sum(1 for r in param_results if r["success"])
    param_avg_time = sum(r["elapsed"] for r in param_results if r["success"]) / max(1, param_success_count)
    mission_success_count = sum(1 for r in mission_results if r["success"])
    mission_avg_up = sum(r["upload_time"] for r in mission_results if r["success"]) / max(1, mission_success_count)
    mission_avg_down = sum(r["download_time"] for r in mission_results if r["success"]) / max(1, mission_success_count)

    print(f"1. Parameter Downloads:   {param_success_count}/{TOTAL_PARAM_CYCLES} Passed (Success Rate: {param_success_count/TOTAL_PARAM_CYCLES*100:.1f}%)")
    print(f"   - Average Duration:    {param_avg_time:.2f} seconds per 1037-parameter download")
    print(f"2. Mission Operations:    {mission_success_count}/{TOTAL_MISSION_CYCLES} Passed (Success Rate: {mission_success_count/TOTAL_MISSION_CYCLES*100:.1f}%)")
    print(f"   - Average Upload:      {mission_avg_up:.2f} seconds")
    print(f"   - Average Download:    {mission_avg_down:.2f} seconds")
    print(f"3. Concurrent RC Health:  {truth_monitor.rc_packet_count} RC packets received on FC Truth")
    print(f"   - Max RC Inter-Packet: {truth_monitor.max_rc_gap_ms:.1f} ms")
    print(f"   - Failsafe Events:     {len(truth_monitor.failsafe_events)}")
    print(f"   - MCU Reboot Events:   {len(truth_monitor.reboot_events)}")

    passed_all = (param_success_count == TOTAL_PARAM_CYCLES and
                  mission_success_count == TOTAL_MISSION_CYCLES and
                  len(truth_monitor.failsafe_events) == 0 and
                  len(truth_monitor.reboot_events) == 0)

    print("\n" + BOLD + "-"*78 + RESET)
    if passed_all:
        print(GREEN + BOLD + "PHASE 4 CLEAN TRANSFER ACCEPTANCE GATE: 100% PASSED!" + RESET)
    else:
        print(RED + BOLD + f"PHASE 4 GATE FAILED: Parameters={param_success_count}/{TOTAL_PARAM_CYCLES}, Missions={mission_success_count}/{TOTAL_MISSION_CYCLES}, Failsafes={len(truth_monitor.failsafe_events)}" + RESET)
    print(BOLD + "="*78 + RESET)

    return 0 if passed_all else 1

if __name__ == '__main__':
    sys.exit(run_acceptance_suite())
