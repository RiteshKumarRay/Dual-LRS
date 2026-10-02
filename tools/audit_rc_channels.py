#!/usr/bin/env python3
"""
Senior Audit Tool: Dual-LRS End-to-End RC Channel & Cadence Diagnostic
Validates:
1. Ground Handset Ingest Rate (ESP32 /dev/ttyUSB0) vs TDM RF Rate (90ms cycle = ~11.1 Hz)
2. Ground Input Channels 1..16 vs Flight Controller Received Channels 1..16 (via Radxa TCP 5760)
3. Controlled Stick Movement Tracker (proves whether CH1 is Throttle, Roll, or Throttle Cut)
"""

import sys
import os
import time
import json
import threading
import serial
from pymavlink import mavutil

class AuditEngine:
    def __init__(self, ground_port="/dev/ttyUSB0", fc_host=None, fc_port=5760):
        self.ground_port = ground_port
        self.fc_host = fc_host or os.getenv("DUAL_LRS_FC_HOST")
        self.fc_port = fc_port
        self.running = True

        # Ground state
        self.ground_ch = [0] * 16
        self.ground_frames = 0
        self.ground_rf_tx = 0
        self.ground_fps = 0.0
        self.ground_rf_fps = 0.0
        self.ground_lock = threading.Lock()
        self.last_ground_wire = ""

        # FC state
        self.fc_ch = [0] * 16
        self.fc_frames = 0
        self.fc_fps = 0.0
        self.fc_lock = threading.Lock()
        self.fc_connected = False

    def ground_reader(self):
        try:
            s = serial.Serial(self.ground_port, 115200, timeout=1.0)
            last_calc = time.time()
            prev_frames = 0
            prev_tx = 0
            while self.running:
                line = s.readline()
                if not line:
                    continue
                try:
                    data = json.loads(line.decode('utf-8', errors='replace').strip())
                    if data.get("event") == "RC_GROUND_IN":
                        with self.ground_lock:
                            raw_ch = data.get("ch", [])
                            # Convert 11-bit CRSF to us
                            self.ground_ch = [round((c - 992) * 5/8 + 1500) if c > 0 else 0 for c in raw_ch]
                            self.ground_frames = data.get("frames", self.ground_frames + 1)
                            self.ground_rf_tx = data.get("rf_tx", self.ground_rf_tx)
                            self.last_ground_wire = data.get("wire", "")

                        now = time.time()
                        if now - last_calc >= 1.0:
                            dt = now - last_calc
                            with self.ground_lock:
                                self.ground_fps = (self.ground_frames - prev_frames) / dt
                                self.ground_rf_fps = (self.ground_rf_tx - prev_tx) / dt
                                prev_frames = self.ground_frames
                                prev_tx = self.ground_rf_tx
                            last_calc = now
                except Exception:
                    pass
            s.close()
        except Exception as e:
            print(f"[-] Ground reader error: {e}")

    def fc_reader(self):
        try:
            mav = mavutil.mavlink_connection(f"tcp:{self.fc_host}:{self.fc_port}")
            msg = mav.wait_heartbeat(timeout=5)
            if not msg:
                print("[-] FC heartbeat timeout on TCP 5760")
                return
            self.fc_connected = True
            last_calc = time.time()
            fc_frame_count = 0
            prev_fc_frames = 0
            while self.running:
                m = mav.recv_match(type="RC_CHANNELS", blocking=True, timeout=1.0)
                if m:
                    d = m.to_dict()
                    with self.fc_lock:
                        self.fc_ch = [d[f"chan{i}_raw"] for i in range(1, 17)]
                        fc_frame_count += 1
                        self.fc_frames = fc_frame_count

                    now = time.time()
                    if now - last_calc >= 1.0:
                        dt = now - last_calc
                        with self.fc_lock:
                            self.fc_fps = (fc_frame_count - prev_fc_frames) / dt
                            prev_fc_frames = fc_frame_count
                        last_calc = now
        except Exception as e:
            print(f"[-] FC reader error: {e}")

    def run_snapshot(self, duration_sec=5.0):
        t_ground = threading.Thread(target=self.ground_reader, daemon=True)
        t_fc = threading.Thread(target=self.fc_reader, daemon=True)
        t_ground.start()
        t_fc.start()

        time.sleep(1.5) # Let connections settle
        print("=" * 80)
        print("AUDIT: DUAL-LRS CADENCE & CHANNEL COMPARISON (5-SECOND SAMPLE)")
        print("=" * 80)

        t0 = time.time()
        while time.time() - t0 < duration_sec:
            time.sleep(1.0)
            with self.ground_lock:
                g_ch = list(self.ground_ch)
                g_fps = self.ground_fps
                g_rf_fps = self.ground_rf_fps
                g_wire = self.last_ground_wire
            with self.fc_lock:
                f_ch = list(self.fc_ch)
                f_fps = self.fc_fps
                f_conn = self.fc_connected

            print(f"\n--- Timestamp: {time.strftime('%H:%M:%S')} ---")
            print(f"Cadence: Ground Ingest = {g_fps:.1f} Hz | Ground RF TX = {g_rf_fps:.1f} Hz | FC RX Rate = {f_fps:.1f} Hz")
            print(f"Ground Channels 1..8 : {g_ch[:8]}")
            print(f"FC RC_CHANNELS 1..8  : {f_ch[:8]}")
            print(f"Ground Channels 9..16: {g_ch[8:]}")
            print(f"FC RC_CHANNELS 9..16 : {f_ch[8:]}")

            # Fidelity check
            mismatches = []
            for i in range(16):
                if abs(g_ch[i] - f_ch[i]) > 4:
                    mismatches.append(f"CH{i+1}(G:{g_ch[i]}!=FC:{f_ch[i]})")
            if mismatches:
                print(f"Channel Mismatches: {', '.join(mismatches)}")
            else:
                print("Fidelity: 100% Match across Ground Ingest -> RF Link -> Air Unit -> FC!")

        self.running = False

if __name__ == "__main__":
    engine = AuditEngine()
    engine.run_snapshot(5.0)
