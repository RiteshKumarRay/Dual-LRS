#!/usr/bin/env python3
"""
Dual-LRS Phase 5 Powered-Node RF-Loss Acceptance Benchmark

Test Procedure:
  1. Props REMOVED (mandatory bench safety).
  2. Ground ESP32 and Air BlackPill remain continuously powered.
  3. Continuous truth monitoring on Radxa port 5760 (RC health, failsafes, reboots).
  4. Start mission transfer over Ground RF link (/dev/ttyUSB0).
  5. Introduce RF path interruption (shielding, attenuation, or RF path disconnect).
  6. Observe transport retry exhaustion / pause and ArduPilot radio failsafe (~500-800ms).
  7. Confirm zero MCU reboots on Air or Ground.
  8. Restore RF path.
  9. Confirm automatic link reacquisition, failsafe clearing, and fresh stick authority.
 10. Execute post-restoration mission cycle to verify transport pipeline remains clean and unblocked.
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

class FCTruthAuditor:
    def __init__(self):
        self.running = True
        self.failsafe_events = []
        self.failsafe_cleared_events = []
        self.reboot_events = []
        self.rc_healthy_history = []
        self.last_healthy_state = None
        self.rc_packet_count = 0
        self.latest_rc = None

    def start(self, fc_conn):
        self.thread = threading.Thread(target=self._run, args=(fc_conn,), daemon=True)
        self.thread.start()

    def _run(self, fc):
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
                    self.latest_rc = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw)
                elif mtype == "SYS_STATUS":
                    healthy = bool(msg.onboard_control_sensors_health & rc_bit)
                    if healthy != self.last_healthy_state:
                        self.last_healthy_state = healthy
                        self.rc_healthy_history.append((now, healthy))
                        if not healthy:
                            self.failsafe_events.append((now, "RC sensor became unhealthy in SYS_STATUS"))
                        else:
                            self.failsafe_cleared_events.append((now, "RC sensor restored healthy in SYS_STATUS"))
                elif mtype == "STATUSTEXT":
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode(errors="ignore")
                    if "radio failsafe" in text.lower():
                        if "cleared" in text.lower():
                            self.failsafe_cleared_events.append((now, f"STATUSTEXT: {text}"))
                        else:
                            self.failsafe_events.append((now, f"STATUSTEXT: {text}"))
                    if "init" in text.lower() or "boot" in text.lower() or "ready" in text.lower():
                        self.reboot_events.append((now, f"Potential reboot text: {text}"))
            except Exception:
                pass

    def stop(self):
        self.running = False

    def failsafe_observed_since(self, timestamp):
        """Only count a failsafe event after the current loss marker."""
        return any(event_time >= timestamp for event_time, _ in self.failsafe_events)

    def recovery_observed_since(self, timestamp):
        """Only count a healthy/clear event after the current restore marker."""
        return any(event_time >= timestamp for event_time, _ in self.failsafe_cleared_events)


def execute_mission_cycle(gnd, sysid, compid, cycle_label):
    WP_COUNT = 4
    test_wps = [
        (int(19.0760 * 1e7), int(72.8777 * 1e7), 0.0),   # WP0
        (int(19.0765 * 1e7), int(72.8780 * 1e7), 20.0),  # WP1
        (int(19.0770 * 1e7), int(72.8785 * 1e7), 30.0),  # WP2
        (int(19.0775 * 1e7), int(72.8780 * 1e7), 25.0),  # WP3
    ]

    print(f"\n{BOLD}--- [{cycle_label}] Executing Full Mission Cycle ---{RESET}")
    # 1. Clear
    gnd.mav.mission_clear_all_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    t0 = time.time()
    clear_ok = False
    while time.time() - t0 < 3.0:
        msg = gnd.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
        if msg:
            clear_ok = True
            break
    print(f"  [1/3] Mission Clear: {'OK (' + f'{time.time()-t0:.2f}s)' if clear_ok else 'TIMEOUT'}")

    # 2. Upload
    t_up = time.time()
    gnd.mav.mission_count_send(sysid, compid, WP_COUNT, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent_items = set()
    upload_ok = False
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = gnd.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=0.2)
        if msg:
            mtype = msg.get_type()
            if mtype in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                seq = msg.seq
                if seq < WP_COUNT:
                    lat, lon, alt = test_wps[seq]
                    gnd.mav.mission_item_int_send(
                        sysid, compid, seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1, 0, 0, 0, 0,
                        lat, lon, alt,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    sent_items.add(seq)
            elif mtype == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    upload_ok = True
                    break

    up_duration = time.time() - t_up
    print(f"  [2/3] Mission Upload (4 WPs): {'OK (' + f'{up_duration:.2f}s)' if upload_ok else 'FAILED'}")

    # 3. Download
    t_down = time.time()
    gnd.mav.mission_request_list_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    downloaded = {}
    expected_count = 0
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = gnd.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=0.2)
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

    down_duration = time.time() - t_down
    download_ok = (len(downloaded) == WP_COUNT)
    print(f"  [3/3] Mission Download (4 WPs): {'OK (' + f'{down_duration:.2f}s)' if download_ok else 'FAILED'}")

    # Coordinate integrity check
    integrity_ok = True
    if download_ok:
        for seq, item in sorted(downloaded.items()):
            exp_lat, exp_lon, exp_alt = test_wps[seq]
            if seq > 0:
                if item.x != exp_lat or item.y != exp_lon or abs(item.z - exp_alt) > 0.1:
                    integrity_ok = False

    passed = (clear_ok and upload_ok and download_ok and integrity_ok)
    print(f"  Result: {GREEN + 'PASSED (100% Integrity)' if passed else RED + 'FAILED'}{RESET}")
    return passed


def main():
    print(BOLD + "="*78 + RESET)
    print(BOLD + "DUAL-LRS PHASE 5: POWERED-NODE RF-LOSS BENCH GATE" + RESET)
    print(BOLD + "="*78 + RESET)
    print(YELLOW + "SAFETY NOTICE: PROPELLERS MUST BE REMOVED FOR THIS BENCH TEST." + RESET)
    print(f"Ground Link: {GND_PORT} | FC Truth: {RADXA_TCP}\n")

    # Connect to Ground Radio
    print(CYAN + "[1/4] Connecting to Ground Radio..." + RESET)
    try:
        gnd = mavutil.mavlink_connection(GND_PORT, baud=115200, source_system=255, source_component=190)
    except Exception as e:
        print(RED + f"Error opening {GND_PORT}: {e}" + RESET)
        sys.exit(1)

    # Connect to FC Truth
    print(CYAN + "[2/4] Connecting to Flight Controller Truth..." + RESET)
    try:
        fc = mavutil.mavlink_connection(RADXA_TCP, source_system=254, source_component=189)
    except Exception as e:
        print(RED + f"Error opening {RADXA_TCP}: {e}" + RESET)
        sys.exit(1)

    auditor = FCTruthAuditor()
    auditor.start(fc)

    # Await initial heartbeats
    print(CYAN + "[3/4] Awaiting Heartbeat over RF Link..." + RESET)
    hb = gnd.recv_match(type='HEARTBEAT', blocking=True, timeout=8.0)
    if not hb:
        print(RED + "ERROR: No heartbeat received from FC over RF!" + RESET)
        sys.exit(1)

    sysid = hb.get_srcSystem()
    compid = hb.get_srcComponent()
    print(GREEN + f"  -> FC Online: SysID={sysid}, CompID={compid}" + RESET)

    # Phase 5A: Clean baseline verification before impairment
    print(CYAN + "\n[Phase 5A] Baseline Pre-Impairment Verification..." + RESET)
    baseline_ok = execute_mission_cycle(gnd, sysid, compid, "BASELINE")
    if not baseline_ok:
        print(RED + "Baseline test failed! Aborting Phase 5." + RESET)
        sys.exit(1)

    print(GREEN + "Baseline verified. RC Truth Healthy: " + str(auditor.last_healthy_state) + RESET)

    # Phase 5B: Controlled RF path interruption
    print(BOLD + "\n" + "="*78 + RESET)
    print(BOLD + "[Phase 5B] CONTROLLED RF-LOSS IMPAIRMENT" + RESET)
    print(BOLD + "="*78 + RESET)
    print(YELLOW + "INSTRUCTIONS FOR BENCH TESTER:" + RESET)
    print("1. Ground ESP32 and Air BlackPill MUST REMAIN POWERED.")
    print("2. Interrupt ONLY the RF path using approved shielding or a rated attenuator.")
    print("   NEVER disconnect an antenna or leave a powered RF output unterminated.")
    print("3. Maintain RF loss for 5 to 10 seconds.")
    print("4. Press ENTER when RF path has been interrupted...")
    input(">>> Press ENTER once RF loss is applied: ")

    t_loss_start = time.time()
    print(YELLOW + f"[{t_loss_start:.2f}] RF Loss active. Monitoring failsafe engagement..." + RESET)

    # Monitor for failsafe engagement on truth port
    failsafe_detected = False
    for _ in range(50): # up to 5 seconds
        time.sleep(0.1)
        if auditor.failsafe_observed_since(t_loss_start):
            failsafe_detected = True
            t_fs = time.time()
            elapsed_fs = (t_fs - t_loss_start) * 1000.0
            print(GREEN + f"  >>> [FAILSAFE DETECTED] ArduPilot entered failsafe in ~{elapsed_fs:.1f}ms! <<<" + RESET)
            break

    if not failsafe_detected:
        print(YELLOW + "  Warning: Failsafe event not yet detected by truth monitor. Continuing..." + RESET)

    # Attempt a mission command during RF loss to verify graceful non-blocking behavior
    print(CYAN + "Attempting mission upload during RF loss (expecting timeout or graceful abort)..." + RESET)
    t_attempt = time.time()
    gnd.mav.mission_count_send(sysid, compid, 4, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    # Drain or wait briefly
    time.sleep(2.0)

    # Phase 5C: Restore RF Path
    print(BOLD + "\n" + "="*78 + RESET)
    print(BOLD + "[Phase 5C] RESTORE RF PATH" + RESET)
    print(BOLD + "="*78 + RESET)
    print(YELLOW + "INSTRUCTIONS FOR BENCH TESTER:" + RESET)
    print("1. Remove the shield or restore the approved attenuator setting.")
    print("2. Both MCUs should still be powered.")
    print("3. Press ENTER when RF path is restored...")
    input(">>> Press ENTER once RF path is restored: ")

    t_restore = time.time()
    print(YELLOW + f"[{t_restore:.2f}] RF Path restored. Monitoring automatic link reacquisition..." + RESET)

    # Wait for failsafe clearance
    cleared = False
    for _ in range(80): # up to 8 seconds
        time.sleep(0.1)
        if auditor.recovery_observed_since(t_restore):
            cleared = True
            t_clr = time.time()
            elapsed_clr = (t_clr - t_restore) * 1000.0
            print(GREEN + f"  >>> [LINK REACQUIRED] ArduPilot cleared failsafe in ~{elapsed_clr:.1f}ms! <<<" + RESET)
            break

    if not cleared:
        print(RED + "  ERROR: Link did not clear failsafe within 8s!" + RESET)

    # Phase 5D: Post-Restoration Mission Verification
    print(CYAN + "\n[Phase 5D] Executing Post-Restoration Mission Verification..." + RESET)
    post_ok = execute_mission_cycle(gnd, sysid, compid, "POST-RESTORE")

    auditor.stop()

    # Final Evaluation
    print("\n" + BOLD + "="*78 + RESET)
    print(BOLD + "PHASE 5 POWERED-NODE RF-LOSS BENCH AUDIT REPORT" + RESET)
    print(BOLD + "="*78 + RESET)
    print(f"1. Baseline Mission Cycle:     {'PASSED' if baseline_ok else 'FAILED'}")
    print(f"2. Failsafe Engagement:        {'CONFIRMED' if failsafe_detected else 'UNCONFIRMED'}")
    print(f"3. Automatic Recovery:         {'CONFIRMED' if cleared else 'FAILED'}")
    print(f"4. Post-Recovery Mission Cycle: {'PASSED (100% Integrity)' if post_ok else 'FAILED'}")
    print(f"5. MCU Reboot Events:          {len(auditor.reboot_events)} (Expected 0)")
    print(f"6. Latest RC Channels:         {auditor.latest_rc}")

    success = baseline_ok and failsafe_detected and cleared and post_ok and (len(auditor.reboot_events) == 0)
    print("="*78)
    if success:
        print(GREEN + BOLD + ">>> PHASE 5 POWERED-NODE RF-LOSS BENCH GATE: PASSED (100% SUCCESS) <<<" + RESET)
        sys.exit(0)
    else:
        print(RED + BOLD + ">>> PHASE 5 POWERED-NODE RF-LOSS BENCH GATE: FAILED <<<" + RESET)
        sys.exit(1)


if __name__ == '__main__':
    main()
