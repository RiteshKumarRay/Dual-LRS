#!/usr/bin/env python3
import serial
import time
import subprocess
import threading
import sys

GND_PORT = "/dev/ttyUSB0"
BAUD = 115200

print(f"Connecting to Ground on Laptop ({GND_PORT})...")
try:
    s_gnd = serial.Serial(GND_PORT, BAUD, timeout=0.2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Failed to open {GND_PORT}: {e}")
    sys.exit(1)

# Drain Ground
s_gnd.reset_input_buffer()

# 1. Test Air (Radxa) -> Ground (Laptop)
print("\n--- TEST 1: AIR (Radxa /dev/ttyAS4) -> GROUND (Laptop /dev/ttyUSB0) ---")
test_air_msg = b"HELLO_FROM_AIR_STM32_12345"

# SSH to Radxa to write test_air_msg to /dev/ttyAS4
print(f"Sending '{test_air_msg.decode()}' from Radxa /dev/ttyAS4...")
cmd = f"sshpass -p 'radxa' ssh -o StrictHostKeyChecking=no radxa@10.122.52.186 \"python3 -c \\\"import serial, time; s = serial.Serial('/dev/ttyAS4', 115200); s.write(b'{test_air_msg.decode()}'); s.flush(); s.close()\\\"\""
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
print("\n--- TEST 2: GROUND (Laptop /dev/ttyUSB0) -> AIR (Radxa /dev/ttyAS4) ---")
test_gnd_msg = b"HELLO_FROM_GROUND_ESP32_67890"

# Start reader on Radxa
radxa_out = []
def radxa_read():
    read_cmd = f"sshpass -p 'radxa' ssh -o StrictHostKeyChecking=no radxa@10.122.52.186 \"python3 -c \\\"import serial, time; s = serial.Serial('/dev/ttyAS4', 115200, timeout=0.1); t=time.time(); b=b''; \\nwhile time.time()-t < 2.5: \\n  c=s.read(50); \\n  if c: b+=c; \\n  if b'{test_gnd_msg.decode()}' in b: break; \\ns.close(); \\nprint('RADXA_FOUND' if b'{test_gnd_msg.decode()}' in b else f'RADXA_RAW_LEN={{len(b)}}')\\\"\""
    p = subprocess.run(read_cmd, shell=True, capture_output=True, text=True)
    radxa_out.append(p.stdout)

t = threading.Thread(target=radxa_read)
t.start()
time.sleep(0.5)

print(f"Sending '{test_gnd_msg.decode()}' from Ground /dev/ttyUSB0...")
s_gnd.write(test_gnd_msg)
s_gnd.flush()

t.join()
radxa_res = "".join(radxa_out).strip()
print(f"Radxa result: {radxa_res}")

s_gnd.close()
