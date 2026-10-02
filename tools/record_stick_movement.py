#!/usr/bin/env python3
"""
Records stick movement and logs exact raw hex and channels.
"""
import serial, time, json

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
print("Move ROLL stick all the way LEFT, then all the way RIGHT, then release.")
print("Recording live data for 10 seconds...\n")

t0 = time.time()
while time.time() - t0 < 10.0:
    l = s.readline().decode('utf-8', errors='replace').strip()
    if 'RC_GROUND_IN' in l:
        try:
            d = json.loads(l)
            ch = d['ch']
            roll = round((ch[0] - 992) * 5/8 + 1500)
            pitch = round((ch[1] - 992) * 5/8 + 1500)
            thr = round((ch[2] - 992) * 5/8 + 1500)
            yaw = round((ch[3] - 992) * 5/8 + 1500)
            # Print if roll is not ~1370 or pitch is not 1500
            if abs(roll - 1370) > 40 or abs(pitch - 1500) > 30:
                print(f"Roll={roll:4d} Pitch={pitch:4d} Thr={thr:4d} Yaw={yaw:4d} | Hex[0..5]={d['hex'][:10]}")
        except Exception:
            pass
s.close()
print("Done recording.")
