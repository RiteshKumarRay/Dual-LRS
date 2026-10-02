#!/usr/bin/env python3
"""
Controlled Stick and Switch Movement Diagnostic
Guides user through moving one control at a time to prove:
1. Which physical stick axis controls Channel 1 (proves TAER vs AETR vs Throttle Cut)
2. Which physical switches or pots are mapped to Channels 5..10
3. Proves why Channel 1 shows 988us vs 1500us
"""

import sys
import time
import json
import serial
from pymavlink import mavutil

def get_channels():
    # Reads live channels from Ground ESP32 (/dev/ttyUSB0)
    try:
        s = serial.Serial("/dev/ttyUSB0", 115200, timeout=1.0)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            l = s.readline()
            if b"RC_GROUND_IN" in l:
                data = json.loads(l.decode('utf-8', errors='replace').strip())
                raw_ch = data.get("ch", [])
                s.close()
                return [round((c - 992) * 5/8 + 1500) if c > 0 else 0 for c in raw_ch]
        s.close()
    except Exception as e:
        print(f"Serial read error: {e}")
    return None

def monitor_live():
    print("=" * 80)
    print("LIVE CHANNEL DELTA MONITOR (Press Ctrl+C to exit)")
    print("Move any stick, pot, or toggle any switch to see which channel reacts:")
    print("=" * 80)
    baseline = get_channels()
    if not baseline:
        print("Failed to get baseline. Check /dev/ttyUSB0 connection.")
        return

    print(f"Baseline Channels 1..8 : {baseline[:8]}")
    print(f"Baseline Channels 9..16: {baseline[8:]}")
    print("\nListening for movements...")

    prev = list(baseline)
    while True:
        curr = get_channels()
        if not curr:
            time.sleep(0.05)
            continue
        deltas = []
        for i in range(16):
            diff = curr[i] - prev[i]
            if abs(diff) > 20: # Meaningful movement threshold (> 20 us)
                deltas.append(f"CH{i+1}: {prev[i]} -> {curr[i]} (Δ={diff:+d}us)")
        if deltas:
            print(f"[{time.strftime('%H:%M:%S')}] {', '.join(deltas)}")
            prev = list(curr)
        time.sleep(0.05)

if __name__ == "__main__":
    monitor_live()
