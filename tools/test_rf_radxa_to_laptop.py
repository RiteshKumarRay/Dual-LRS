#!/usr/bin/env python3
import os
import serial
import time
import subprocess
import threading
import sys

GND_PORT = os.getenv("DUAL_LRS_GND_PORT", "/dev/ttyUSB0")
BAUD = int(os.getenv("DUAL_LRS_BAUD", "115200"))

# Companion computer configuration via environment variables
RADXA_HOST = os.getenv("RADXA_HOST")
RADXA_USER = os.getenv("RADXA_USER", "radxa")
RADXA_PASSWORD = os.getenv("RADXA_PASSWORD")
RADXA_SERIAL_PORT = os.getenv("RADXA_SERIAL_PORT", "/dev/ttyAS4")

if not RADXA_HOST:
    print("Error: Missing required environment variable RADXA_HOST.")
    print("Usage: RADXA_HOST=<ip-or-host> RADXA_PASSWORD=<password> python3 tools/test_rf_radxa_to_laptop.py")
    sys.exit(1)

if not RADXA_PASSWORD:
    print("Error: Missing required environment variable RADXA_PASSWORD.")
    print("Do not run with an empty or undefined password fallback.")
    sys.exit(1)

print(f"Connecting to Ground on Laptop ({GND_PORT})...")
try:
    s_gnd = serial.Serial(GND_PORT, BAUD, timeout=0.2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Failed to open {GND_PORT}: {e}")
    sys.exit(1)

# Drain Ground
s_gnd.reset_input_buffer()

# 1. Test Air (Radxa) -> Ground (Laptop)
print(f"\n--- TEST 1: AIR (Radxa {RADXA_SERIAL_PORT}) -> GROUND (Laptop {GND_PORT}) ---")
test_air_msg = b"HELLO_FROM_AIR_STM32_12345"

# SSH to Radxa to write test_air_msg to RADXA_SERIAL_PORT
print(f"Sending '{test_air_msg.decode()}' from Radxa {RADXA_SERIAL_PORT}...")
cmd = (
    f"sshpass -p {subprocess.list2cmdline([RADXA_PASSWORD])} "
    f"ssh -o StrictHostKeyChecking=no {RADXA_USER}@{RADXA_HOST} "
    f"\"python3 -c \\\"import serial, time; s = serial.Serial('{RADXA_SERIAL_PORT}', {BAUD}); s.write(b'{test_air_msg.decode()}'); s.flush(); s.close()\\\"\""
)
subprocess.run(cmd, shell=True, check=True)

# Listen on Ground
t0 = time.time()
rec = b""
success_air_to_gnd = False
while time.time() - t0 < 2.0:
    c = s_gnd.read(50)
    if c:
        rec += c
        if test_air_msg in rec:
            success_air_to_gnd = True
            break

if success_air_to_gnd:
    print(f"SUCCESS! Ground received: {test_air_msg.decode()}")
else:
    print(f"Air->Ground payload not found in {len(rec)} bytes received on Ground.")

# 2. Test Ground (Laptop) -> Air (Radxa)
print(f"\n--- TEST 2: GROUND (Laptop {GND_PORT}) -> AIR (Radxa {RADXA_SERIAL_PORT}) ---")
test_gnd_msg = b"HELLO_FROM_GROUND_ESP32_67890"

# Start reader on Radxa
radxa_out = []
def radxa_read():
    read_cmd = (
        f"sshpass -p {subprocess.list2cmdline([RADXA_PASSWORD])} "
        f"ssh -o StrictHostKeyChecking=no {RADXA_USER}@{RADXA_HOST} "
        f"\"python3 -c \\\"import serial, time; s = serial.Serial('{RADXA_SERIAL_PORT}', {BAUD}, timeout=0.1); t=time.time(); b=b''; \\nwhile time.time()-t < 2.5: \\n  c=s.read(50); \\n  if c: b+=c; \\n  if b'{test_gnd_msg.decode()}' in b: break; \\ns.close(); \\nprint('RADXA_FOUND' if b'{test_gnd_msg.decode()}' in b else f'RADXA_RAW_LEN={{len(b)}}')\\\"\""
    )
    p = subprocess.run(read_cmd, shell=True, capture_output=True, text=True)
    radxa_out.append(p.stdout)

t = threading.Thread(target=radxa_read)
t.start()
time.sleep(0.5)

print(f"Sending '{test_gnd_msg.decode()}' from Ground {GND_PORT}...")
s_gnd.write(test_gnd_msg)
s_gnd.flush()

t.join()
radxa_res = "".join(radxa_out).strip()
print(f"Radxa result: {radxa_res}")

s_gnd.close()
