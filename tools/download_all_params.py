#!/usr/bin/env python3
import time, json, sys, serial
from pymavlink import mavutil

sys.stdout.reconfigure(line_buffering=True)

print("===============================================================")
print("DUAL-LRS: FULL 0-1037 PARAMETER DOWNLOAD HARDWARE VERIFICATION")
print("===============================================================")

print("Connecting to Ground Station on /dev/ttyUSB0 (115200 baud)...")
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200, source_system=255, source_component=190)

print("Draining stale serial backlog...")
t_drain = time.time()
while time.time() - t_drain < 1.2:
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
print(f"FC Online: SysID={sysid}, CompID={compid}, Autopilot={hb.autopilot}, Type={hb.type}")

# Send PARAM_REQUEST_LIST
print("\n[UPLINK] Requesting full parameter table (PARAM_REQUEST_LIST)...")
m.mav.param_request_list_send(sysid, compid)

params = {}
total_params = 1037
t_start = time.time()
last_rx_time = t_start
first_rx_time = None

print("[DOWNLINK] Streaming parameters over Dual-LRS LoRa link...")
# Stream download loop: keep reading as long as parameters arrive (3.5s inactivity timeout)
while True:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)
    if msg:
        now = time.time()
        if first_rx_time is None:
            first_rx_time = now
            print(f"  First parameter arrived in {first_rx_time - t_start:.2f}s!")
        last_rx_time = now
        idx = msg.param_index
        total_params = msg.param_count
        name = msg.param_id.strip('\x00')
        val = msg.param_value
        params[idx] = (name, val)

        if len(params) % 50 == 0 or len(params) == total_params:
            elapsed = now - t_start
            pct = (len(params) / total_params * 100) if total_params else 0
            rate = len(params) / elapsed
            print(f"  [DOWNLINK PROGRESS] {len(params)}/{total_params} ({pct:.1f}%) in {elapsed:.1f}s | {rate:.1f} params/sec | Latest: #{idx} {name}={val}")

        if total_params and len(params) >= total_params:
            break
    else:
        # Inactivity check: if we've received at least 1 param, wait up to 4.5s for more
        if first_rx_time is not None:
            if time.time() - last_rx_time > 4.5:
                break
        else:
            # If no param received yet, wait up to 5s after request
            if time.time() - t_start > 5.0:
                print("  Initial request timeout, re-sending PARAM_REQUEST_LIST...")
                m.mav.param_request_list_send(sysid, compid)
                t_start = time.time()

initial_elapsed = time.time() - t_start
print(f"\n[STREAM COMPLETED] Initial stream downlinked {len(params)}/{total_params} parameters in {initial_elapsed:.1f}s ({len(params)/initial_elapsed:.1f} params/sec)")

# Step 2: Fill any missing indices with targeted individual requests
missing = [i for i in range(total_params) if i not in params]
if missing:
    print(f"\n[HOLE RECOVERY] Recovering {len(missing)} missing parameters via targeted uplink...")
    for m_idx in missing:
        if m_idx in params:
            continue
        for attempt in range(4):
            m.mav.param_request_read_send(sysid, compid, b'', m_idx)
            t_req = time.time()
            found = False
            while time.time() - t_req < 1.2:
                msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                if msg:
                    idx = msg.param_index
                    p_name = msg.param_id.strip('\x00')
                    params[idx] = (p_name, msg.param_value)
                    if idx == m_idx:
                        found = True
                        break
            if m_idx in params:
                print(f"  Retrieved missing #{m_idx}: {params[m_idx][0]} = {params[m_idx][1]} (attempt {attempt+1})")
                break
        else:
            print(f"  WARNING: Failed to retrieve #{m_idx} after 4 attempts")

total_time = time.time() - t_start
completion_pct = (len(params) / total_params * 100) if total_params else 0

print("\n===============================================================")
print(f"PARAMETER DOWNLOAD SUMMARY:")
print(f"Total Parameters Downloaded: {len(params)}/{total_params} ({completion_pct:.1f}%)")
print(f"Total Elapsed Time: {total_time:.1f} seconds")
print(f"Effective Download Speed: {len(params)/total_time:.1f} params/second")
print("===============================================================")

# Save all parameters to json
with open('bench_params.json', 'w') as f:
    json.dump({str(k): {'name': v[0], 'value': v[1]} for k, v in sorted(params.items())}, f, indent=2)
print(f"Saved all {len(params)} parameters to bench_params.json")
