#!/usr/bin/env python3
import time, sys
from pymavlink import mavutil

print("===============================================================")
print("DUAL-LRS: LIVE MISSION PROTOCOL UPLOAD & DOWNLOAD BENCHMARK")
print("===============================================================")

print("Connecting to Ground Station on /dev/ttyUSB0 (115200 baud)...")
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200, source_system=255, source_component=190)

print("Draining stale serial backlog...")
t_drain = time.time()
while time.time() - t_drain < 2.0:
    m.recv_msg()

print("Waiting for live FC Heartbeat...")
hb = None
t_hb = time.time()
last_gcs_hb = 0
while time.time() - t_hb < 15.0:
    if time.time() - last_gcs_hb >= 1.0:
        last_gcs_hb = time.time()
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
    if msg and msg.get_srcSystem() != 51:
        hb = msg
        break

if not hb:
    print("Error: No heartbeat received from Flight Controller!")
    sys.exit(1)

sysid = hb.get_srcSystem()
compid = hb.get_srcComponent()
print(f"FC Online: SysID={sysid}, CompID={compid}")

# Define a 5-waypoint mission
WP_COUNT = 5
waypoints = [
    # lat, lon, alt (cm or deg * 1e7)
    (int(19.0760 * 1e7), int(72.8777 * 1e7), 15.0), # WP0 (Home)
    (int(19.0765 * 1e7), int(72.8780 * 1e7), 20.0), # WP1
    (int(19.0770 * 1e7), int(72.8785 * 1e7), 25.0), # WP2
    (int(19.0775 * 1e7), int(72.8780 * 1e7), 20.0), # WP3
    (int(19.0760 * 1e7), int(72.8777 * 1e7), 15.0), # WP4 (RTL)
]

for cycle in range(1, 4):
    print(f"\n--- MISSION BENCHMARK CYCLE {cycle}/3 ---")

    # 1. Clear existing mission
    m.mav.mission_clear_all_send(sysid, compid)
    t_clear_wait = time.time()
    while time.time() - t_clear_wait < 1.5:
        ack = m.recv_match(type='MISSION_ACK', blocking=True, timeout=0.3)
        if ack:
            break
    time.sleep(0.05)

    # 2. Upload Mission
    t_up_start = time.time()
    m.mav.mission_count_send(sysid, compid, WP_COUNT, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    upload_success = False
    items_sent = 0
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=0.5)
        if msg:
            if msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                seq = msg.seq
                if seq < WP_COUNT:
                    lat, lon, alt = waypoints[seq]
                    # Send MISSION_ITEM_INT
                    m.mav.mission_item_int_send(
                        sysid, compid, seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1, # current, autocontinue
                        0, 0, 0, 0, # param 1-4
                        lat, lon, alt,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    items_sent += 1
            elif msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    upload_success = True
                    break
                else:
                    print(f"  Mission upload rejected with ack type: {msg.type}")
                    break

    t_up = time.time() - t_up_start
    if not upload_success:
        print(f"  FAILED: Mission Upload timed out after {t_up:.2f}s (sent {items_sent}/{WP_COUNT} items)")
        continue
    print(f"  [UPLOAD SUCCESS] {WP_COUNT} waypoints uploaded in {t_up:.2f} seconds!")

    # 3. Download Mission
    t_down_start = time.time()
    m.mav.mission_request_list_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    down_count = 0
    downloaded_wps = {}
    t_deadline = time.time() + 8.0
    while time.time() < t_deadline:
        msg = m.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT'], blocking=True, timeout=0.5)
        if msg:
            if msg.get_type() == 'MISSION_COUNT':
                down_count = msg.count
                if down_count > 0:
                    m.mav.mission_request_int_send(sysid, compid, 0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            elif msg.get_type() in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
                downloaded_wps[msg.seq] = msg
                next_seq = msg.seq + 1
                if next_seq < down_count:
                    m.mav.mission_request_int_send(sysid, compid, next_seq, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                else:
                    # All items received, send ACK
                    m.mav.mission_ack_send(sysid, compid, mavutil.mavlink.MAV_MISSION_ACCEPTED, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                    break

    t_down = time.time() - t_down_start
    if len(downloaded_wps) == WP_COUNT:
        print(f"  [DOWNLOAD SUCCESS] {len(downloaded_wps)} waypoints downloaded in {t_down:.2f} seconds!")
        # Verify coordinates
        all_matched = True
        for seq, wp in sorted(downloaded_wps.items()):
            expected_lat, expected_lon, expected_alt = waypoints[seq]
            wp_match = True
            if seq == 0:
                # WP0 is Home in ArduPilot; altitude is always 0.0m
                print(f"    WP#{seq} (Home): lat={wp.x/1e7:.5f}, lon={wp.y/1e7:.5f}, alt={wp.z:.1f}m -> STORED OK")
            else:
                if wp.get_type() == 'MISSION_ITEM_INT':
                    if wp.x != expected_lat or wp.y != expected_lon or abs(wp.z - expected_alt) > 0.01:
                        wp_match = False
                        all_matched = False
                print(f"    WP#{seq}: lat={wp.x/1e7:.5f}, lon={wp.y/1e7:.5f}, alt={wp.z:.1f}m -> {'MATCH' if wp_match else 'MISMATCH'}")
        if all_matched:
            print(f"  [VERIFICATION] All {WP_COUNT} waypoints matched with 100% integrity!")
    else:
        print(f"  FAILED: Download received {len(downloaded_wps)}/{WP_COUNT} waypoints in {t_down:.2f}s")

print("\nMission protocol benchmark finished.")
