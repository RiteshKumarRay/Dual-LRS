#!/usr/bin/env python3
import time, sys
from pymavlink import mavutil

print("Connecting to /dev/ttyUSB0...")
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)

print("Draining backlog...")
t0 = time.time()
while time.time() - t0 < 2.0:
    m.recv_msg()

print("Waiting for Heartbeat...")
hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5.0)
if not hb:
    print("No heartbeat!")
    sys.exit(1)
print(f"FC Online: sysid={hb.get_srcSystem()}, compid={hb.get_srcComponent()}")
sysid = hb.get_srcSystem()
compid = hb.get_srcComponent()

# Listen for 3 seconds of telemetry to see stream rates and RADIO_STATUS
print("\n--- 3-SECOND TELEMETRY AUDIT ---")
t_end = time.time() + 3.0
counts = {}
while time.time() < t_end:
    msg = m.recv_msg()
    if msg:
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1
        if t == 'RADIO_STATUS':
            print(f"  [RADIO_STATUS] rssi={msg.rssi}, remrssi={msg.remrssi}, rxerrors={msg.rxerrors}, fixed={msg.fixed}, txbuf={msg.txbuf}")

for k, v in sorted(counts.items()):
    print(f"  {k}: {v} pkts ({v/3.0:.1f} Hz)")

print("\n--- TEST 1: MISSION CLEAR ALL ---")
m.mav.mission_clear_all_send(sysid, compid)
t_start = time.time()
ack = None
while time.time() - t_start < 3.0:
    msg = m.recv_match(type='MISSION_ACK', blocking=True, timeout=0.1)
    if msg:
        ack = msg
        break
if ack:
    print(f"  MISSION_CLEAR_ALL ACK in {time.time()-t_start:.3f}s: result={ack.type}")
else:
    print("  MISSION_CLEAR_ALL TIMED OUT!")

print("\n--- TEST 2: SINGLE WAYPOINT UPLOAD ---")
m.mav.mission_count_send(sysid, compid, 2, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
t_start = time.time()
while time.time() - t_start < 5.0:
    msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=0.2)
    if msg:
        now = time.time() - t_start
        print(f"  [{now:.3f}s] Received: {msg.get_type()} seq={getattr(msg, 'seq', None)} type={getattr(msg, 'type', None)}")
        if msg.get_type() in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
            seq = msg.seq
            print(f"  [{now:.3f}s] Sending WP#{seq}...")
            m.mav.mission_item_int_send(
                sysid, compid, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(19.0760 * 1e7), int(72.8777 * 1e7), 20.0,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
        elif msg.get_type() == 'MISSION_ACK':
            print(f"  [{now:.3f}s] UPLOAD COMPLETE! ACK result={msg.type}")
            break

print("\n--- TEST 3: MISSION DOWNLOAD ---")
m.mav.mission_request_list_send(sysid, compid, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
t_start = time.time()
while time.time() - t_start < 5.0:
    msg = m.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT', 'MISSION_ACK'], blocking=True, timeout=0.2)
    if msg:
        now = time.time() - t_start
        print(f"  [{now:.3f}s] Received: {msg.get_type()} count={getattr(msg, 'count', None)} seq={getattr(msg, 'seq', None)}")
        if msg.get_type() == 'MISSION_COUNT':
            print(f"  [{now:.3f}s] Requesting WP#0...")
            m.mav.mission_request_int_send(sysid, compid, 0, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        elif msg.get_type() in ['MISSION_ITEM', 'MISSION_ITEM_INT']:
            seq = msg.seq
            if seq == 0:
                print(f"  [{now:.3f}s] Received WP#0! Requesting WP#1...")
                m.mav.mission_request_int_send(sysid, compid, 1, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            elif seq == 1:
                print(f"  [{now:.3f}s] Received WP#1! Sending ACK...")
                m.mav.mission_ack_send(sysid, compid, mavutil.mavlink.MAV_MISSION_ACCEPTED, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                print(f"  [{now:.3f}s] DOWNLOAD COMPLETE!")
                break
