#!/usr/bin/env python3
"""
Full RC Chain Tracer:
Compares FS-i6X Wire Bytes (via ESP32 console) directly with ArduPilot MAVLink RC_CHANNELS.
"""
import sys
import time
import json
import serial
from pymavlink import mavutil

def unpack_11bit(p):
    ch = []
    buf, bits, idx = 0, 0, 0
    for _ in range(16):
        while bits < 11 and idx < len(p):
            buf |= p[idx] << bits
            idx += 1
            bits += 8
        if bits < 11: break
        ch.append(buf & 0x7FF)
        buf >>= 11
        bits -= 11
    return ch

def crsf_to_us(crsf):
    return round((crsf - 992) * 5 / 8 + 1500)

def main():
    print("Connecting to Ground ESP32 on /dev/ttyUSB0...")
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.02)

    print("Connecting to ArduPilot MAVLink on tcp:10.94.163.186:5760...")
    mav = mavutil.mavlink_connection('tcp:10.94.163.186:5760')
    mav.wait_heartbeat(timeout=3)
    print("Connected to both!")

    print("\nMove sticks. Showing (Raw Wire CH1..4) vs (Ground Decoded CH1..4) vs (ArduPilot CH1..4):")
    print("-" * 85)

    last_ap = None
    last_print = 0

    while True:
        # Check MAVLink
        msg = mav.recv_match(type='RC_CHANNELS', blocking=False)
        if msg:
            ap_ch = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
            if last_ap is None or any(abs(a - b) > 8 for a, b in zip(ap_ch, last_ap)):
                last_ap = ap_ch
                # Read latest from serial
                wire_str = ""
                raw_ch = []
                gnd_ch = []
                while ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if 'RC_GROUND_IN' in line:
                        try:
                            d = json.loads(line)
                            w = d.get('wire', '')
                            if len(w) >= 52:
                                wb = bytes.fromhex(w)
                                for i in range(len(wb) - 25):
                                    if wb[i] == 0x0F and wb[i+25] == 0x00:
                                        raw_ch = unpack_11bit(wb[i+2:i+24])
                                        wire_str = ' '.join(f'{b:02X}' for b in wb[i:i+26])
                                        break
                            gnd_ch = d.get('ch', [])
                        except Exception:
                            pass

                now = time.time()
                if now - last_print > 0.1:
                    last_print = now
                    wire_us = [crsf_to_us(c) for c in raw_ch[:4]] if raw_ch else []
                    gnd_us = [crsf_to_us(c) for c in gnd_ch[:4]] if gnd_ch else []
                    print(f"AP:  R={ap_ch[0]:4d} P={ap_ch[1]:4d} T={ap_ch[2]:4d} Y={ap_ch[3]:4d} | "
                          f"GND: {gnd_us} | WIRE_RAW: {raw_ch[:4]}")
        time.sleep(0.01)

if __name__ == '__main__':
    main()
