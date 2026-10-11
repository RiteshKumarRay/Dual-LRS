#!/usr/bin/env python3
import time, sys
from pymavlink import mavutil

print("Testing full parameter download over RF link (/dev/ttyUSB0)...")
m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)

hb = m.wait_heartbeat(timeout=5)
if not hb:
    print("No heartbeat!")
    sys.exit(1)

print(f"FC Online: sys={m.target_system}, comp={m.target_component}")

print("Settling link...")
time.sleep(2.0)
while m.recv_msg(): pass

print("Requesting full parameter list (PARAM_REQUEST_LIST)...")
t_start = time.time()
m.mav.param_request_list_send(m.target_system, m.target_component)

params = {}
seen_indices = set()
total = None
last_print = time.time()

while True:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=8.0)
    if not msg:
        break
    p_id = msg.param_id
    if isinstance(p_id, bytes): p_id = p_id.decode(errors='ignore').rstrip('\x00')
    params[p_id] = msg.param_value
    seen_indices.add(msg.param_index)
    if total is None:
        total = msg.param_count
        print(f"Total parameters expected: {total}")
    
    if time.time() - last_print >= 5.0:
        elapsed = time.time() - t_start
        print(f"[{elapsed:.1f}s] Received {len(seen_indices)}/{total} parameters ({len(seen_indices)/elapsed:.1f} params/s)...")
        last_print = time.time()

    if total and len(seen_indices) >= total:
        break

# If any indices were missed during bulk stream, request them via PARAM_REQUEST_READ
if total and len(seen_indices) < total:
    missing = [i for i in range(total) if i not in seen_indices]
    print(f"Missing {len(missing)} parameter indices: {missing[:10]}... Requesting via PARAM_REQUEST_READ...")
    for idx in missing:
        m.mav.param_request_read_send(m.target_system, m.target_component, b"", idx)
        retry_msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if retry_msg and retry_msg.param_index == idx:
            p_id = retry_msg.param_id
            if isinstance(p_id, bytes): p_id = p_id.decode(errors='ignore').rstrip('\x00')
            params[p_id] = retry_msg.param_value
            seen_indices.add(idx)
            print(f"  Recovered param index {idx} ({p_id})!")

elapsed = time.time() - t_start
print(f"\nFinal: Downloaded {len(seen_indices)}/{total} unique parameters in {elapsed:.2f} seconds ({len(seen_indices)/elapsed:.1f} params/sec)!")
if total and len(seen_indices) == total:
    print("SUCCESS: 100% parameters downloaded with zero loss!")
    sys.exit(0)
else:
    print(f"FAILED: Still missed {total - len(seen_indices)} parameters")
    sys.exit(1)
