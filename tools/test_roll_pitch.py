#!/usr/bin/env python3
"""
Diagnostic tool to observe Roll and Pitch interaction in real time.
Displays raw bytes, binary bit layout, and decoded values.
"""
import serial, time, json

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
print("=== ROLL / PITCH REAL-TIME DIAGNOSTIC ===")
print("Move your ROLL stick slowly from left to right.")
print("Watch how Byte 0, Byte 1, Byte 2, and the decoded values behave.\n")

last_roll = None
last_pitch = None

t_start = time.time()
try:
    while time.time() - t_start < 20.0:
        line = s.readline().decode('utf-8', errors='replace').strip()
        if 'RC_GROUND_IN' in line:
            try:
                d = json.loads(line)
                ch = d['ch']
                roll_raw = ch[0]
                pitch_raw = ch[1]

                # Check for movement
                if last_roll is None or abs(roll_raw - last_roll) > 5 or abs(pitch_raw - last_pitch) > 5:
                    last_roll = roll_raw
                    last_pitch = pitch_raw

                    roll_us = round((roll_raw - 992) * 5/8 + 1500)
                    pitch_us = round((pitch_raw - 992) * 5/8 + 1500)

                    hex_str = d.get('hex', '')
                    if len(hex_str) >= 6:
                        b = bytes.fromhex(hex_str[:6])
                        b0, b1, b2 = b[0], b[1], b[2]
                        print(f"ROLL: {roll_us:4d} µs (raw {roll_raw:4d}) | PITCH: {pitch_us:4d} µs (raw {pitch_raw:4d}) | B0={b0:02X} B1={b1:02X} B2={b2:02X} [B1_bin={b1:08b}]")
            except Exception:
                pass
except KeyboardInterrupt:
    pass
finally:
    s.close()
