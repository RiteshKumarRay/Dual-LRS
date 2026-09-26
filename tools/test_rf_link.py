#!/usr/bin/env python3
import serial
import time
import sys

AIR_PORT = "/dev/ttyACM0"
GND_PORT = "/dev/ttyUSB0"
BAUD = 115200

print(f"Connecting to Air ({AIR_PORT}) and Ground ({GND_PORT}) @ {BAUD} baud...")

try:
    s_air = serial.Serial(AIR_PORT, BAUD, timeout=0.2)
except Exception as e:
    print(f"Could not open Air port {AIR_PORT}: {e}")
    sys.exit(1)

try:
    # Use dtr=False to avoid resetting ESP32
    s_gnd = serial.Serial(GND_PORT, BAUD, timeout=0.2, dsrdtr=False, rtscts=False)
except Exception as e:
    print(f"Could not open Ground port {GND_PORT}: {e}")
    s_air.close()
    sys.exit(1)

time.sleep(0.5)
s_air.reset_input_buffer()
s_air.reset_output_buffer()
s_gnd.reset_input_buffer()

print("Both serial ports ready.")
print("Monitoring ESP32 Ground Unit while transmitting test packets from STM32 Air Unit...\n")

test_message = b"TEST_PAYLOAD_LORA_PONG"
success = False

for attempt in range(1, 6):
    print(f"--- Attempt {attempt}/5 ---")
    print(f"Sending '{test_message.decode()}' into STM32 ({AIR_PORT})...")
    s_air.write(test_message)
    s_air.flush()

    # Listen on ESP32 Ground port for up to 1.5 seconds
    t0 = time.time()
    received_bytes = b""
    while time.time() - t0 < 1.5:
        chunk = s_gnd.read(100)
        if chunk:
            received_bytes += chunk
            if test_message in received_bytes:
                print(f" SUCCESS! Received test message on ESP32 Ground: {test_message.decode()}")
                success = True
                break

    if not success:
        # Check what was received
        radio_status_count = received_bytes.count(b'\xfe\t')
        other_bytes = len(received_bytes) - (radio_status_count * 17)
        print(f"  ESP32 received {len(received_bytes)} total bytes ({radio_status_count} RADIO_STATUS packets, {other_bytes} other bytes)")
        if other_bytes > 0:
            print(f"  Non-status hex: {received_bytes.hex()}")

    if success:
        break
    time.sleep(0.3)

s_air.close()
s_gnd.close()

if success:
    print("\n RESULT: RF LINK IS WORKING! Data received over the air!")
else:
    print("\n RESULT: No payload received over the air yet.")
