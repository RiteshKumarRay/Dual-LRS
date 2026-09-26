#!/usr/bin/env python3
"""
Dual-LRS Mock Drone Telemetry Generator
=======================================
Simulates an ArduPilot Quadrotor running on the STM32 Air Unit.
Sends live MAVLink telemetry packets into /dev/ttyACM0 (STM32 Air Unit USB):
- HEARTBEAT (1 Hz) -> Autopilot ArduCopter, Armable/Standby
- SYS_STATUS (1 Hz) -> 4S Battery 15.2V, 85%, All sensors healthy (Pre-Arm Good)
- GPS_RAW_INT & GLOBAL_POSITION_INT (2 Hz) -> 3D Fix, 14 sats, live coords
- ATTITUDE (5 Hz) -> Live roll/pitch/yaw oscillations
- RAW_IMU (2 Hz) -> Gyro, Accelerometer, Magnetometer
- SCALED_PRESSURE (1 Hz) -> Barometer pressure and temperature
- VFR_HUD (2 Hz) -> Airspeed, Groundspeed, Heading, Throttle
- STATUSTEXT -> Info notifications
"""

import sys
import time
import math
import random

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink not found. Run with /home/ritesh/venv-ardupilot/bin/python")
    sys.exit(1)

AIR_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Base GPS Coordinates (Chandigarh / Punjab area)
BASE_LAT = 30.733315
BASE_LON = 76.779418
BASE_ALT = 12.5 # meters AGL

def main():
    print(f"==================================================")
    print(f" Dual-LRS Drone Telemetry Simulator")
    print(f" Connecting to Air Unit on {AIR_PORT} @ {BAUD_RATE}...")
    print(f"==================================================")

    try:
        master = mavutil.mavlink_connection(AIR_PORT, baud=BAUD_RATE, source_system=1, source_component=1)
    except Exception as e:
        print(f"Failed to open {AIR_PORT}: {e}")
        sys.exit(1)

    print("Connected to Air Unit! Streaming live MAVLink telemetry...")
    print("Look at QGroundControl now - it should show Vehicle 1 Connected!")

    start_time = time.time()
    last_1hz = 0
    last_2hz = 0
    last_5hz = 0
    statustext_sent = False

    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            boot_ms = int(elapsed * 1000) & 0xFFFFFFFF

            # Gentle simulated flight movement
            roll_rad = 0.08 * math.sin(elapsed * 1.5)
            pitch_rad = 0.05 * math.cos(elapsed * 1.2)
            yaw_rad = (elapsed * 0.1) % (2 * math.pi)
            heading_deg = int(math.degrees(yaw_rad)) % 360

            # Small simulated GPS drift
            lat = BASE_LAT + (0.00003 * math.sin(elapsed * 0.2))
            lon = BASE_LON + (0.00003 * math.cos(elapsed * 0.2))
            alt = BASE_ALT + (0.5 * math.sin(elapsed * 0.5))

            # --- 5 Hz: High-Rate Flight Dynamics (ATTITUDE) ---
            if now - last_5hz >= 0.20:
                last_5hz = now
                master.mav.attitude_send(
                    boot_ms,
                    roll_rad,
                    pitch_rad,
                    yaw_rad,
                    0.02 * math.cos(elapsed),
                    0.02 * math.sin(elapsed),
                    0.05
                )

            # --- 2 Hz: Position & Navigation (GPS, VFR_HUD, IMU) ---
            if now - last_2hz >= 0.50:
                last_2hz = now
                # 1. Global Position
                master.mav.global_position_int_send(
                    boot_ms,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    int((alt + 310) * 1000), # MSL altitude mm
                    int(alt * 1000),         # Relative altitude mm
                    int(120 * math.sin(yaw_rad)), # vx cm/s
                    int(120 * math.cos(yaw_rad)), # vy cm/s
                    int(-10 * math.sin(elapsed)), # vz cm/s
                    heading_deg * 100             # cdeg
                )

                # 2. VFR HUD
                master.mav.vfr_hud_send(
                    2.8, # airspeed m/s
                    2.5, # groundspeed m/s
                    heading_deg,
                    28,  # throttle %
                    alt, # altitude m
                    0.1  # climb m/s
                )

                # 3. Raw IMU (Gyro, Accelerometer, Magnetometer)
                master.mav.raw_imu_send(
                    int(now * 1e6),
                    int(roll_rad * 100),
                    int(pitch_rad * 100),
                    -981 + random.randint(-5, 5), # 1G vertical accel
                    int(math.degrees(roll_rad) * 10),
                    int(math.degrees(pitch_rad) * 10),
                    5,
                    int(180 * math.cos(yaw_rad)), # Mag X
                    int(180 * math.sin(yaw_rad)), # Mag Y
                    -320                          # Mag Z
                )

            # --- 1 Hz: Systems, Sensors & Heartbeat ---
            if now - last_1hz >= 1.00:
                last_1hz = now

                # 1. Autopilot HEARTBEAT (ArduCopter in LOITER mode, Ready to Fly)
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    5, # LOITER Mode in ArduCopter
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )

                # 2. Battery & Pre-arm Health (SYS_STATUS)
                # All vital sensors marked healthy & enabled
                sensors_healthy = (
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS |
                    mavutil.mavlink.MAV_SYS_STATUS_SENSOR_BATTERY
                )
                master.mav.sys_status_send(
                    sensors_healthy, sensors_healthy, sensors_healthy,
                    350,   # 35% MCU Load
                    15400, # 15.4V (4S LiPo fully healthy)
                    1800,  # 1.8A Current
                    88,    # 88% Battery Remaining
                    0, 0, 0, 0, 0, 0
                )

                # 3. GPS Raw Details (3D Lock, 16 Satellites, HDOP 0.8)
                master.mav.gps_raw_int_send(
                    int(now * 1e6),
                    3, # 3D Fix
                    int(lat * 1e7),
                    int(lon * 1e7),
                    int((alt + 310) * 1000),
                    80,  # HDOP 0.8m
                    95,  # VDOP 0.95m
                    250, # 2.5 m/s
                    heading_deg * 100,
                    16   # 16 satellites
                )

                # 4. Barometer (SCALED_PRESSURE)
                master.mav.scaled_pressure_send(
                    boot_ms,
                    1013.25 + (alt * -0.12), # Barometric pressure (hPa)
                    0.0,
                    2650 # 26.50 deg C
                )

                # 5. Status Text announcement on startup
                if not statustext_sent and elapsed > 2.0:
                    master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_NOTICE,
                        b"Dual-LRS: Pre-Arm Checks PASSED. Safe to Fly."
                    )
                    statustext_sent = True
                    print("-> Pre-arm checks passed message sent!")

                print(f"[{elapsed:5.1f}s] Telemetry Live: Alt={alt:.1f}m | Bat=15.4V (88%) | Sats=16 | Mode=LOITER", end="\r")

            time.sleep(0.02) # 50 Hz loop

    except KeyboardInterrupt:
        print("\nStopping telemetry generator.")
        master.close()

if __name__ == "__main__":
    main()
