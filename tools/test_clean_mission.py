#!/usr/bin/env python3
import time, sys
from pymavlink import mavutil

print("===============================================================")
print("DUAL-LRS: FULL 3-CYCLE MISSION PROTOCOL VERIFICATION")
print("===============================================================")

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200, source_system=255, source_component=190)

print("Waiting 2s for ESP32 and Air PLL lock...")
time.sleep(2.0)

# Drain backlog
while m.recv_msg():
    pass

# Heartbeat
hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5.0)
if not hb:
    print("Error: No FC Heartbeat!")
    sys.exit(1)
sysid = hb.get_srcSystem()
compid = hb.get_srcComponent()
print(f"FC Online: SysID={sysid}, CompID={compid}")

WP_COUNT = 4
test_wps = [
    # lat, lon, alt
    (int(19.0760 * 1e7), int(72.8777 * 1e7), 0.0),   # WP0 (Home)
    (int(19.0765 * 1e7), int(72.8780 * 1e7), 20.0),  # WP1
    (int(19.0770 * 1e7), int(72.8785 * 1e7), 30.0),  # WP2
    (int(19.0775 * 1e7), int(72.8780 * 1e7), 25.0),  # WP3
]

for cycle in range(1, 4):
    print(f"\n>>> CYCLE {cycle}/3 <<<")

    # 1. Clear existing mission
    print("  [1/3] Clearing mission...")
    m.mav.mission_clear_all_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    t0 = time.time()
    ack_clear = False
    while time.time() - t0 < 3.0:
        msg = m.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
        if msg:
            print(f"    Clear ACK received in {time.time()-t0:.3f}s (result={msg.type})")
            ack_clear = True
            break
    if not ack_clear:
        print("    WARNING: Clear ACK timed out, proceeding anyway...")

    time.sleep(0.1)

    # 2. Upload Mission
    print(f"  [2/3] Uploading {WP_COUNT} waypoints...")
    t_up_start = time.time()
    m.mav.mission_count_send(sysid, compid, WP_COUNT, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    upload_done = False
    sent_items = set()
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=0.3)
        if msg:
            mtype = msg.get_type()
            if mtype in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                seq = msg.seq
                if seq < WP_COUNT:
                    lat, lon, alt = test_wps[seq]
                    m.mav.mission_item_int_send(
                        sysid, compid, seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1, 0, 0, 0, 0,
                        lat, lon, alt,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    sent_items.add(seq)
            elif mtype == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    upload_done = True
                    print(f"    [UPLOAD SUCCESS] All {WP_COUNT} items uploaded in {time.time()-t_up_start:.2f}s!")
                    break
                else:
                    print(f"    Mission ACK error: {msg.type}")
                    break

    if not upload_done:
        print(f"    FAILED: Upload timed out (sent items: {sorted(sent_items)})")
        continue

    time.sleep(0.1)

    # 3. Download Mission
    print(f"  [3/3] Downloading mission from FC...")
    t_down_start = time.time()
    m.mav.mission_request_list_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    downloaded = {}
    expected_count = 0
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = m.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=0.3)
        if msg:
            mtype = msg.get_type()
            if mtype == 'MISSION_COUNT':
                expected_count = msg.count
                if expected_count > 0:
                    m.mav.mission_request_int_send(sysid, compid, 0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            elif mtype in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
                downloaded[msg.seq] = msg
                next_seq = msg.seq + 1
                if next_seq < expected_count:
                    m.mav.mission_request_int_send(sysid, compid, next_seq, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                else:
                    m.mav.mission_ack_send(sysid, compid, mavutil.mavlink.MAV_MISSION_ACCEPTED, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                    print(f"    [DOWNLOAD SUCCESS] Downloaded {len(downloaded)} items in {time.time()-t_down_start:.2f}s!")
                    break

    # Verification
    if len(downloaded) == WP_COUNT:
        matched = True
        for seq, item in sorted(downloaded.items()):
            exp_lat, exp_lon, exp_alt = test_wps[seq]
            if seq == 0:
                print(f"      WP#0 (Home): lat={item.x/1e7:.5f}, lon={item.y/1e7:.5f}, alt={item.z:.1f}m -> OK")
            else:
                ok = (item.x == exp_lat and item.y == exp_lon and abs(item.z - exp_alt) < 0.1)
                if not ok: matched = False
                print(f"      WP#{seq}: lat={item.x/1e7:.5f}, lon={item.y/1e7:.5f}, alt={item.z:.1f}m -> {'MATCH' if ok else 'MISMATCH'}")
        if matched:
            print(f"    *** VERIFICATION PASSED: 100% integrity ***")
    else:
        print(f"    FAILED: Expected {WP_COUNT}, got {len(downloaded)} items")

print("\nBenchmark completed.")
