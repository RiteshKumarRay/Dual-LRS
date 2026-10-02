#!/usr/bin/env python3
"""
tools/phase1_raw_benchmark.py
Deterministic host-side benchmark harness for Phase 1 Raw E22 Link Testing.

Measures:
  1. Register Read Check (non-destructive 0xC1 0x00 0x07 with restore validation)
  2. Mode A: Air -> Ground Simplex across payload sizes [0, 10, 30, 53, 54, 55]
  3. Mode B: Ground -> Air Simplex across payload sizes [0, 10, 30, 53, 54, 55]
  4. Mode C: Ground-master sequential Ping-Pong RTT across payload sizes [0, 10, 30, 53, 54, 55]

Streams all raw events to audit/phase1_raw_log.jsonl and outputs a structured
markdown summary report.
"""

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import serial
except ImportError:
    serial = None


class DiagBenchRunner:
    def __init__(
        self,
        air_port: str,
        ground_port: str,
        baud: int = 115200,
        output_file: str = "audit/phase1_raw_log.jsonl",
        summary_file: str = "audit/phase1_raw_summary.md",
        pacing_ms: int = 100,
        count: int = 1000,
        payloads: Optional[List[int]] = None,
        dry_run: bool = False,
    ):
        self.air_port_name = air_port
        self.ground_port_name = ground_port
        self.baud = baud
        self.output_file = output_file
        self.summary_file = summary_file
        self.pacing_ms = pacing_ms
        self.count = count
        self.payloads = payloads or [0, 10, 30, 53, 54, 55]
        self.dry_run = dry_run

        self.ser_air = None
        self.ser_ground = None
        self.log_fd = None
        self.records: List[Dict] = []
        self.modem_failed = False
        self.air_reg_status = "UNKNOWN"
        self.ground_reg_status = "UNKNOWN"
        self.log_overflow_events: List[Dict] = []
        self.total_dropped_events = 0
        self.malformed_json_lines: List[Tuple[str, str, str]] = []
        self.non_json_text_lines: List[Tuple[str, str]] = []
        self.fatal_serial_error = False
        self.serial_error_msg = ""

    def open_ports(self):
        if self.dry_run:
            print("[DRY-RUN] Skipping serial port opening.")
            return

        if serial is None:
            print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
            sys.exit(1)

        try:
            print(f"Connecting to Air unit on {self.air_port_name} at {self.baud} baud...")
            self.ser_air = serial.Serial(self.air_port_name, self.baud, timeout=0.1)

            print(f"Connecting to Ground unit on {self.ground_port_name} at {self.baud} baud...")
            self.ser_ground = serial.Serial(self.ground_port_name, self.baud, timeout=0.1)

            time.sleep(1.0)  # Allow USB serial to settle
            self.flush_inputs()
        except (serial.SerialException, OSError) as exc:
            print(f"[FATAL ERROR] Failed to open serial port: {exc}", file=sys.stderr)
            self.fatal_serial_error = True
            self.serial_error_msg = str(exc)

    def flush_inputs(self):
        try:
            if self.ser_air:
                self.ser_air.reset_input_buffer()
            if self.ser_ground:
                self.ser_ground.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            print(f"[FATAL ERROR] Serial buffer flush failed: {exc}", file=sys.stderr)
            self.fatal_serial_error = True
            self.serial_error_msg = str(exc)

    def send_cmd(self, port_type: str, cmd: str):
        if self.dry_run:
            print(f"[DRY-RUN] Send to {port_type}: {cmd}")
            return

        ser = self.ser_air if port_type == "AIR" else self.ser_ground
        if not ser:
            return
        try:
            ser.write((cmd + "\n").encode("utf-8"))
            ser.flush()
        except (serial.SerialException, OSError) as exc:
            print(f"[FATAL ERROR] Serial write failed on {port_type}: {exc}", file=sys.stderr)
            self.fatal_serial_error = True
            self.serial_error_msg = f"{port_type}: {exc}"
            self.records.append({
                "event": "HOST_SERIAL_ERROR",
                "port": port_type,
                "error": str(exc),
                "_host_timestamp": time.time()
            })

    def read_incoming_lines(self, timeout_sec: float = 0.5) -> List[Tuple[str, str]]:
        """Reads available lines from both ports within timeout_sec."""
        lines = []
        if self.dry_run or self.fatal_serial_error:
            return lines

        t_end = time.time() + timeout_sec
        while time.time() < t_end:
            had_data = False
            for port_type, ser in [("AIR", self.ser_air), ("GROUND", self.ser_ground)]:
                if ser:
                    try:
                        if ser.in_waiting > 0:
                            raw = ser.readline().decode("utf-8", errors="replace").strip()
                            if raw:
                                had_data = True
                                lines.append((port_type, raw))
                                self.record_raw_line(port_type, raw)
                    except (serial.SerialException, OSError) as exc:
                        print(f"[FATAL ERROR] Serial read failure on {port_type}: {exc}", file=sys.stderr)
                        self.fatal_serial_error = True
                        self.serial_error_msg = f"{port_type}: {exc}"
                        self.records.append({
                            "event": "HOST_SERIAL_ERROR",
                            "port": port_type,
                            "error": str(exc),
                            "_host_timestamp": time.time()
                        })
                        return lines
                    except Exception as exc:
                        print(f"[ERROR] Unexpected error reading {port_type}: {exc}", file=sys.stderr)
                        self.records.append({
                            "event": "HOST_UNEXPECTED_ERROR",
                            "port": port_type,
                            "error": str(exc),
                            "_host_timestamp": time.time()
                        })
            if not had_data:
                time.sleep(0.01)
        return lines

    def record_raw_line(self, port_type: str, raw: str):
        if not (raw.startswith("{") and raw.endswith("}")):
            # Normal non-JSON serial output (e.g. boot messages, status banners)
            self.non_json_text_lines.append((port_type, raw))
            return

        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                self.malformed_json_lines.append((port_type, raw, "JSON root is not an object"))
                return
            data["_source_port"] = port_type
            data["_host_timestamp"] = time.time()
            self.records.append(data)
            if data.get("event") == "LOG_BUFFER_OVERFLOW":
                self.log_overflow_events.append(data)
                self.total_dropped_events += int(data.get("dropped_events", 0))
            if self.log_fd:
                self.log_fd.write(json.dumps(data) + "\n")
                self.log_fd.flush()
        except json.JSONDecodeError as exc:
            self.malformed_json_lines.append((port_type, raw, str(exc)))
            diag = {
                "event": "HOST_MALFORMED_JSON",
                "port": port_type,
                "raw": raw,
                "error": str(exc),
                "_host_timestamp": time.time()
            }
            self.records.append(diag)
            if self.log_fd:
                self.log_fd.write(json.dumps(diag) + "\n")
                self.log_fd.flush()
            print(f"[{port_type}] [MALFORMED JSON] {raw} (error: {exc})", file=sys.stderr)

    def run_handshake(self) -> bool:
        print("\n--- Handshake & Role Verification ---")
        self.send_cmd("AIR", "CMD:PING")
        self.send_cmd("GROUND", "CMD:PING")
        lines = self.read_incoming_lines(1.5)

        air_ok = False
        ground_ok = False
        for port, line in lines:
            if not (line.startswith("{") and line.endswith("}")):
                continue
            try:
                msg = json.loads(line)
                if isinstance(msg, dict) and msg.get("event") == "PONG":
                    if msg.get("role") == "AIR":
                        air_ok = True
                    elif msg.get("role") == "GROUND":
                        ground_ok = True
            except json.JSONDecodeError as exc:
                print(f"[{port}] Handshake line decode error: {exc}", file=sys.stderr)

        if self.dry_run:
            print("[DRY-RUN] Handshake simulated OK.")
            return True

        print(f"Air Role Verified:    {'[OK]' if air_ok else '[FAILED]'}")
        print(f"Ground Role Verified: {'[OK]' if ground_ok else '[FAILED]'}")
        return air_ok and ground_ok

    def run_register_read(self) -> bool:
        print("\n--- Non-Destructive E22 Register Interrogation ---")
        all_ok = True
        for port_type in ["AIR", "GROUND"]:
            self.send_cmd(port_type, "CMD:REG_READ")
            lines = self.read_incoming_lines(1.0)
            reg_found = False
            for p, line in lines:
                try:
                    msg = json.loads(line)
                    if msg.get("event") == "REG_READ":
                        reg_found = True
                        status = msg.get("status", "UNKNOWN")
                        ok = msg.get("ok", False)
                        if port_type == "AIR":
                            self.air_reg_status = status
                        else:
                            self.ground_reg_status = status
                        print(f"[{port_type}] Register Read Status: {status} (ok={ok})")

                        if status in ["MODE_RESTORE_FAILED", "UART_RESTORE_FAILED"]:
                            print(f"[CRITICAL ERROR] [{port_type}] Modem restoration failed! Hardware unusable.")
                            self.modem_failed = True
                            all_ok = False
                        elif status in ["REGISTERS_UNVERIFIED", "READ_FAILED"]:
                            print(f"[{port_type}] Warning: Registers unverified / read failed ({status}): {msg.get('bytes_hex')}")
                            all_ok = False
                        elif ok:
                            print(f"       REG0={msg.get('reg0')} (UART/AirDataRate), REG1={msg.get('reg1')} (SubPacket/Power), "
                                  f"REG2={msg.get('reg2')} (Channel), REG3={msg.get('reg3')}")
                except json.JSONDecodeError as exc:
                    print(f"[{p}] Register read line decode error: {exc}", file=sys.stderr)
            if not reg_found:
                if not self.dry_run:
                    print(f"[{port_type}] No register response received.")
                    if port_type == "AIR":
                        self.air_reg_status = "NO_RESPONSE"
                    else:
                        self.ground_reg_status = "NO_RESPONSE"
                    all_ok = False
                else:
                    if port_type == "AIR":
                        self.air_reg_status = "SIMULATED_OK"
                    else:
                        self.ground_reg_status = "SIMULATED_OK"
        return all_ok

    def run_mode_a_batch(self, payload_len: int, count: int, pacing_ms: int):
        """Mode A: Air -> Ground Simplex"""
        print(f"\n[Mode A: Air -> Ground] Payload: {payload_len}B (Frame: {payload_len + 11}B) | Count: {count} | Pacing: {pacing_ms}ms")
        self.send_cmd("GROUND", "CMD:MODE_A_RX")
        time.sleep(0.01 if self.dry_run else 0.1)
        self.send_cmd("AIR", f"CMD:MODE_A_TX {payload_len} {count} {pacing_ms}")

        if self.dry_run:
            sim_count = min(count, 10)
            is_split = (payload_len >= 54)
            for s in range(sim_count):
                self.record_raw_line("AIR", json.dumps({
                    "event": "TX_FRAME", "role": "AIR", "mode": "A", "test_id": 1, "seq": s,
                    "payload_len": payload_len, "frame_len": payload_len + 11,
                    "t_uart_start_us": 100000 + s * 100000, "t_uart_end_us": 100500 + s * 100000,
                    "uart_duration_us": 500, "aux_pre_busy": False,
                    "aux_edge_observed": True, "aux_wait_timeout": False,
                    "aux_busy_start_us": 100600 + s * 100000, "aux_ready_us": 113000 + s * 100000,
                    "aux_busy_duration_us": 12400, "overrun": False
                }))
                self.record_raw_line("GROUND", json.dumps({
                    "event": "RX_FRAME", "role": "GROUND", "test_id": 1, "dir": 1, "seq": s,
                    "payload_len": payload_len, "frame_len": payload_len + 11, "crc_ok": True,
                    "pattern_ok": True, "seq_status": "IN_ORDER", "rx_duration_us": 4000,
                    "max_inter_byte_gap_us": 2500 if is_split else 800,
                    "chunks": 2 if is_split else 1
                }))
            return

        expected_sec = (count * pacing_ms) / 1000.0 + 5.0
        t_start = time.time()
        tx_complete = False

        while time.time() - t_start < expected_sec:
            lines = self.read_incoming_lines(0.2)
            if self.fatal_serial_error:
                break
            for port, line in lines:
                if not (line.startswith("{") and line.endswith("}")):
                    continue
                try:
                    msg = json.loads(line)
                    if isinstance(msg, dict) and msg.get("event") == "TX_BATCH_COMPLETE":
                        tx_complete = True
                        break
                except json.JSONDecodeError as exc:
                    print(f"[{port}] TX batch line decode error: {exc}", file=sys.stderr)
            if tx_complete:
                break

        self.send_cmd("AIR", "CMD:STOP")
        self.send_cmd("GROUND", "CMD:STOP")
        time.sleep(0.2)
        self.flush_inputs()

    def run_mode_b_batch(self, payload_len: int, count: int, pacing_ms: int):
        """Mode B: Ground -> Air Simplex"""
        print(f"\n[Mode B: Ground -> Air] Payload: {payload_len}B (Frame: {payload_len + 11}B) | Count: {count} | Pacing: {pacing_ms}ms")
        self.send_cmd("AIR", "CMD:MODE_B_RX")
        time.sleep(0.01 if self.dry_run else 0.1)
        self.send_cmd("GROUND", f"CMD:MODE_B_TX {payload_len} {count} {pacing_ms}")

        if self.dry_run:
            sim_count = min(count, 10)
            is_split = (payload_len >= 54)
            for s in range(sim_count):
                self.record_raw_line("GROUND", json.dumps({
                    "event": "TX_FRAME", "role": "GROUND", "mode": "B", "test_id": 1, "seq": s,
                    "payload_len": payload_len, "frame_len": payload_len + 11,
                    "t_uart_start_us": 100000 + s * 100000, "t_uart_end_us": 100500 + s * 100000,
                    "uart_duration_us": 500, "aux_pre_busy": False,
                    "aux_edge_observed": True, "aux_wait_timeout": False,
                    "aux_busy_start_us": 100600 + s * 100000, "aux_ready_us": 113000 + s * 100000,
                    "aux_busy_duration_us": 12400, "overrun": False
                }))
                self.record_raw_line("AIR", json.dumps({
                    "event": "RX_FRAME", "role": "AIR", "test_id": 1, "dir": 2, "seq": s,
                    "payload_len": payload_len, "frame_len": payload_len + 11, "crc_ok": True,
                    "pattern_ok": True, "seq_status": "IN_ORDER", "rx_duration_us": 4000,
                    "max_inter_byte_gap_us": 2500 if is_split else 800,
                    "chunks": 2 if is_split else 1
                }))
            return

        expected_sec = (count * pacing_ms) / 1000.0 + 5.0
        t_start = time.time()
        tx_complete = False

        while time.time() - t_start < expected_sec:
            lines = self.read_incoming_lines(0.2)
            if self.fatal_serial_error:
                break
            for port, line in lines:
                if not (line.startswith("{") and line.endswith("}")):
                    continue
                try:
                    msg = json.loads(line)
                    if isinstance(msg, dict) and msg.get("event") == "TX_BATCH_COMPLETE":
                        tx_complete = True
                        break
                except json.JSONDecodeError as exc:
                    print(f"[{port}] TX batch line decode error: {exc}", file=sys.stderr)
            if tx_complete:
                break

        self.send_cmd("AIR", "CMD:STOP")
        self.send_cmd("GROUND", "CMD:STOP")
        time.sleep(0.2)
        self.flush_inputs()

    def run_mode_c_batch(self, payload_len: int, count: int, pacing_ms: int):
        """Mode C: Ground Master Ping-Pong RTT"""
        print(f"\n[Mode C: Ping-Pong RTT] Payload: {payload_len}B (Frame: {payload_len + 11}B) | Count: {count} | Pacing: {pacing_ms}ms")
        self.send_cmd("AIR", "CMD:MODE_C_SLAVE")
        time.sleep(0.01 if self.dry_run else 0.1)
        self.send_cmd("GROUND", f"CMD:MODE_C_MASTER {payload_len} {count} {pacing_ms}")

        if self.dry_run:
            sim_count = min(count, 10)
            for s in range(sim_count):
                self.record_raw_line("GROUND", json.dumps({
                    "event": "MODE_C_RTT", "role": "GROUND", "seq": s, "payload_len": payload_len,
                    "crc_ok": True, "pattern_ok": True,
                    "t1_us": 100000 + s * 100000, "t4_us": 135000 + s * 100000,
                    "turnaround_us": 3500, "rtt_us": 31500, "approx_one_way_us": 15750
                }))
            return

        expected_sec = (count * pacing_ms) / 1000.0 + 6.0
        t_start = time.time()
        complete = False

        while time.time() - t_start < expected_sec:
            lines = self.read_incoming_lines(0.2)
            if self.fatal_serial_error:
                break
            for port, line in lines:
                if not (line.startswith("{") and line.endswith("}")):
                    continue
                try:
                    msg = json.loads(line)
                    if isinstance(msg, dict) and msg.get("event") == "MODE_C_COMPLETE":
                        complete = True
                        break
                except json.JSONDecodeError as exc:
                    print(f"[{port}] Mode C complete line decode error: {exc}", file=sys.stderr)
            if complete:
                break

        self.send_cmd("AIR", "CMD:STOP")
        self.send_cmd("GROUND", "CMD:STOP")
        time.sleep(0.2)
        self.flush_inputs()

    def analyze_results(self, mode: str = "all") -> str:
        """Analyzes recorded JSONL events and generates a Markdown summary report."""
        md = []
        md.append("# Phase 1 Raw E22 Link Measurement Report")
        md.append(f"**Date:** {time.strftime('%Y-%m-%d %H:%M:%S')}")
        md.append(f"**Log File:** `{self.output_file}`")
        md.append(f"**Target Pacing (Mode A/B):** {self.pacing_ms} ms ({1000.0/self.pacing_ms:.1f} Hz)")
        md.append(f"**Target Pacing (Mode C):** {max(self.pacing_ms * 2, 100)} ms ({1000.0/max(self.pacing_ms * 2, 100):.1f} Hz)")
        md.append(f"**Requested Count per Payload:** {self.count} frames")
        if self.dry_run:
            md.append("\n> [!WARNING]")
            md.append("> **SIMULATED DRY-RUN DATA ONLY:** The data below was generated by host dry-run simulation and does NOT constitute physical link validation or acceptance evidence.\n")

        all_valid_counts = []

        # Table: Mode A (Air -> Ground)
        if mode in ["A", "all"]:
            md.append("\n## 1. Mode A: Air -> Ground Simplex Measurements")
            md.append("| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |")
            md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")

            for plen in self.payloads:
                tx_events = [r for r in self.records if r.get("event") == "TX_FRAME" and r.get("mode") == "A" and r.get("payload_len") == plen]
                rx_events = [r for r in self.records if r.get("event") == "RX_FRAME" and r.get("dir") == 1 and r.get("payload_len") == plen]
                malformed_events = [r for r in self.records if r.get("event") == "RX_MALFORMED" and (r.get("payload_len") == plen or r.get("payload_len") == 0)]

                req_count = self.count if not self.dry_run else len(tx_events)
                sent = len(tx_events)
                valid_rx = len([r for r in rx_events if r.get("crc_ok") and r.get("pattern_ok")])
                all_valid_counts.append(valid_rx)
                pdr = (valid_rx / sent * 100.0) if sent > 0 else 0.0
                crc_err = len([r for r in rx_events if not r.get("crc_ok")])
                pat_err = len([r for r in rx_events if r.get("crc_ok") and not r.get("pattern_ok")])
                malformed = len(malformed_events)

                avg_uart = (sum(r.get("uart_duration_us", 0) for r in tx_events) / sent) if sent > 0 else 0
                avg_aux = (sum(r.get("aux_busy_duration_us", 0) for r in tx_events) / sent) if sent > 0 else 0
                aux_edge_count = len([r for r in tx_events if r.get("aux_edge_observed")])
                aux_edge_str = f"{aux_edge_count}/{sent}"

                max_ib_gap = max([r.get("max_inter_byte_gap_us", 0) for r in rx_events], default=0)
                avg_chunks = (sum(r.get("chunks", 1) for r in rx_events) / len(rx_events)) if len(rx_events) > 0 else 1
                overruns = len([r for r in tx_events if r.get("overrun")])

                # Evidence-based split classification
                if plen >= 54:
                    if valid_rx > 0 and avg_chunks >= 1.5 and max_ib_gap >= 2000:
                        split_diag = f"MODEM_SPLIT_DETECTED ({avg_chunks:.1f} chunks)"
                    elif valid_rx > 0:
                        split_diag = f"CONTIGUOUS_BURST ({avg_chunks:.1f} chunks)"
                    else:
                        split_diag = "NO_VALID_DATA"
                else:
                    split_diag = "SINGLE_PACKET"

                if self.dry_run:
                    row_status = "SIMULATED"
                elif self.total_dropped_events > 0:
                    row_status = "INVALID (LOG_OVERFLOW)"
                elif self.air_reg_status not in ["OK", "SIMULATED_OK"] or self.ground_reg_status not in ["OK", "SIMULATED_OK"]:
                    row_status = "CONFIGURATION_UNVERIFIED"
                elif valid_rx < 1000:
                    row_status = "INSUFFICIENT_SAMPLES"
                elif pdr >= 98.0 and pat_err == 0 and malformed == 0:
                    row_status = "PASS"
                else:
                    row_status = "FAIL"

                md.append(f"| {plen} | {plen + 11} | {req_count} | {sent} | {valid_rx} | {pdr:.1f}% | {crc_err} | {pat_err} | {malformed} | {avg_uart:.0f} | {avg_aux:.0f} | {aux_edge_str} | {max_ib_gap} | {split_diag} | {overruns} | {row_status} |")

        # Table: Mode B (Ground -> Air)
        if mode in ["B", "all"]:
            md.append("\n## 2. Mode B: Ground -> Air Simplex Measurements")
            md.append("| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |")
            md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")

            for plen in self.payloads:
                tx_events = [r for r in self.records if r.get("event") == "TX_FRAME" and r.get("mode") == "B" and r.get("payload_len") == plen]
                rx_events = [r for r in self.records if r.get("event") == "RX_FRAME" and r.get("dir") == 2 and r.get("payload_len") == plen]
                malformed_events = [r for r in self.records if r.get("event") == "RX_MALFORMED" and (r.get("payload_len") == plen or r.get("payload_len") == 0)]

                req_count = self.count if not self.dry_run else len(tx_events)
                sent = len(tx_events)
                valid_rx = len([r for r in rx_events if r.get("crc_ok") and r.get("pattern_ok")])
                all_valid_counts.append(valid_rx)
                pdr = (valid_rx / sent * 100.0) if sent > 0 else 0.0
                crc_err = len([r for r in rx_events if not r.get("crc_ok")])
                pat_err = len([r for r in rx_events if r.get("crc_ok") and not r.get("pattern_ok")])
                malformed = len(malformed_events)

                avg_uart = (sum(r.get("uart_duration_us", 0) for r in tx_events) / sent) if sent > 0 else 0
                avg_aux = (sum(r.get("aux_busy_duration_us", 0) for r in tx_events) / sent) if sent > 0 else 0
                aux_edge_count = len([r for r in tx_events if r.get("aux_edge_observed")])
                aux_edge_str = f"{aux_edge_count}/{sent}"

                max_ib_gap = max([r.get("max_inter_byte_gap_us", 0) for r in rx_events], default=0)
                avg_chunks = (sum(r.get("chunks", 1) for r in rx_events) / len(rx_events)) if len(rx_events) > 0 else 1
                overruns = len([r for r in tx_events if r.get("overrun")])

                if plen >= 54:
                    if valid_rx > 0 and avg_chunks >= 1.5 and max_ib_gap >= 2000:
                        split_diag = f"MODEM_SPLIT_DETECTED ({avg_chunks:.1f} chunks)"
                    elif valid_rx > 0:
                        split_diag = f"CONTIGUOUS_BURST ({avg_chunks:.1f} chunks)"
                    else:
                        split_diag = "NO_VALID_DATA"
                else:
                    split_diag = "SINGLE_PACKET"

                if self.dry_run:
                    row_status = "SIMULATED"
                elif self.total_dropped_events > 0:
                    row_status = "INVALID (LOG_OVERFLOW)"
                elif self.air_reg_status not in ["OK", "SIMULATED_OK"] or self.ground_reg_status not in ["OK", "SIMULATED_OK"]:
                    row_status = "CONFIGURATION_UNVERIFIED"
                elif valid_rx < 1000:
                    row_status = "INSUFFICIENT_SAMPLES"
                elif pdr >= 98.0 and pat_err == 0 and malformed == 0:
                    row_status = "PASS"
                else:
                    row_status = "FAIL"

                md.append(f"| {plen} | {plen + 11} | {req_count} | {sent} | {valid_rx} | {pdr:.1f}% | {crc_err} | {pat_err} | {malformed} | {avg_uart:.0f} | {avg_aux:.0f} | {aux_edge_str} | {max_ib_gap} | {split_diag} | {overruns} | {row_status} |")

        # Table: Mode C (Ping-Pong RTT)
        if mode in ["C", "all"]:
            md.append("\n## 3. Mode C: Sequential Ping-Pong RTT Measurements")
            md.append("| Payload (B) | Frame (B) | Req Pings | Sent Pings | Timeouts (Lost) | Valid Pongs | PDR (%) | Pattern Err | CRC Err | Min RTT (ms) | Avg RTT (ms) | Max RTT (ms) | Avg Turnaround (µs) | Approx 1-Way (ms) | Status |")
            md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")

            for plen in self.payloads:
                rtt_events = [r for r in self.records if r.get("event") == "MODE_C_RTT" and r.get("payload_len") == plen]
                timeout_events = [r for r in self.records if r.get("event") == "MODE_C_TIMEOUT" and (r.get("payload_len") == plen or r.get("payload_len") is None)]
                pat_err_events = [r for r in self.records if r.get("event") == "MODE_C_INVALID_PATTERN" and r.get("payload_len") == plen]

                req_pings = self.count if not self.dry_run else 10
                valid_pongs = len([r for r in rtt_events if r.get("crc_ok") and r.get("pattern_ok")])
                all_valid_counts.append(valid_pongs)
                timeouts = len(timeout_events)
                pattern_err = len(pat_err_events)
                crc_err = len([r for r in rtt_events if not r.get("crc_ok")])
                pings_sent = valid_pongs + timeouts + pattern_err + crc_err
                if pings_sent == 0 and not self.dry_run:
                    pings_sent = req_pings
                pdr = (valid_pongs / pings_sent * 100.0) if pings_sent > 0 else 0.0

                if self.dry_run:
                    row_status = "SIMULATED"
                elif self.total_dropped_events > 0:
                    row_status = "INVALID (LOG_OVERFLOW)"
                elif self.air_reg_status not in ["OK", "SIMULATED_OK"] or self.ground_reg_status not in ["OK", "SIMULATED_OK"]:
                    row_status = "CONFIGURATION_UNVERIFIED"
                elif valid_pongs < 1000:
                    row_status = "INSUFFICIENT_SAMPLES"
                elif pdr >= 98.0 and pattern_err == 0:
                    row_status = "PASS"
                else:
                    row_status = "FAIL"

                if valid_pongs > 0:
                    rtts = [r.get("rtt_us", 0) / 1000.0 for r in rtt_events]
                    min_rtt = min(rtts)
                    avg_rtt = sum(rtts) / valid_pongs
                    max_rtt = max(rtts)
                    avg_turn = sum(r.get("turnaround_us", 0) for r in rtt_events) / valid_pongs
                    approx_1way = avg_rtt / 2.0
                    md.append(f"| {plen} | {plen + 11} | {req_pings} | {pings_sent} | {timeouts} | {valid_pongs} | {pdr:.1f}% | {pattern_err} | {crc_err} | {min_rtt:.2f} | {avg_rtt:.2f} | {max_rtt:.2f} | {avg_turn:.0f} | {approx_1way:.2f} | {row_status} |")
                else:
                    md.append(f"| {plen} | {plen + 11} | {req_pings} | {pings_sent} | {timeouts} | 0 | 0.0% | {pattern_err} | {crc_err} | N/A | N/A | N/A | N/A | N/A | {row_status} |")

        # Hardware Configuration & System Health
        md.append("\n## 4. Hardware Configuration & System Health")
        md.append(f"- **Air Unit Register Read Status:** `{self.air_reg_status}`")
        md.append(f"- **Ground Unit Register Read Status:** `{self.ground_reg_status}`")
        if self.air_reg_status not in ["OK", "SIMULATED_OK"] or self.ground_reg_status not in ["OK", "SIMULATED_OK"]:
            md.append("  > [!WARNING]\n  > **CONFIGURATION_UNVERIFIED:** E22 modem registers could not be verified in non-destructive read-only mode.\n")

        md.append(f"- **Logging Queue Overflows:** {len(self.log_overflow_events)} (Total dropped events: {self.total_dropped_events})")
        if self.total_dropped_events > 0:
            md.append("  > [!CAUTION]\n  > **RUN INVALIDATED:** Log buffer overflow was detected during execution. Benchmark results cannot be used for formal Phase 1 acceptance because events were dropped.\n")

        total_malformed = len([r for r in self.records if r.get("event") == "RX_MALFORMED"])
        md.append(f"- **Malformed Frames Detected:** {total_malformed}")
        if total_malformed > 0:
            md.append("  > [!WARNING]\n  > Malformed frame headers or unsupported payload lengths were safely rejected by the diagnostic parser.\n")

        md.append(f"- **Malformed Host JSON Lines:** {len(self.malformed_json_lines)}")
        if len(self.malformed_json_lines) > 0:
            md.append("  > [!WARNING]\n  > Malformed JSON was received from firmware and logged as diagnostic events.\n")

        if self.fatal_serial_error:
            md.append(f"- **Host Serial Status:** `FATAL_ERROR` ({self.serial_error_msg})")
            md.append("  > [!CAUTION]\n  > **RUN ABORTED:** Serial communication failure occurred.\n")
        else:
            md.append("- **Host Serial Status:** `OK`")

        # Phase 1 Acceptance Gate Evaluation
        md.append("\n## 5. Phase 1 Formal Acceptance Assessment")
        min_samples = min(all_valid_counts) if all_valid_counts else 0
        if self.dry_run:
            md.append("**Gate Status:** `PENDING PHYSICAL BENCHMARK (DRY-RUN ONLY)`")
            md.append("> [!NOTE]\n> Simulated dry-run output is never accepted as evidence of physical link validation.")
        elif self.fatal_serial_error:
            md.append(f"**Gate Status:** `INVALID — SERIAL I/O FAILURE ({self.serial_error_msg})`")
        elif len(self.malformed_json_lines) > 0:
            md.append("**Gate Status:** `INVALID — MALFORMED JSON DETECTED`")
        elif self.total_dropped_events > 0:
            md.append("**Gate Status:** `INVALID — LOG BUFFER OVERFLOW DETECTED`")
            md.append(f"> [!CAUTION]\n> The benchmark dropped {self.total_dropped_events} log events. The run is invalid for formal acceptance.")
        elif self.air_reg_status != "OK" or self.ground_reg_status != "OK":
            md.append("**Gate Status:** `CONFIGURATION_UNVERIFIED`")
            md.append("> [!WARNING]\n> Modem registers were unverified or read failed. Link parameters (frequency, air data rate, channel) are not verified.")
        elif min_samples < 1000:
            md.append(f"**Gate Status:** `INSUFFICIENT SAMPLES` (Minimum observed: {min_samples}/1000 frames per payload size)")
            md.append("> [!IMPORTANT]\n> Physical acceptance requires at least 1,000 valid frames per payload size.")
        else:
            md.append("**Gate Status:** `EVALUATED — READY FOR ACCEPTANCE REVIEW`")

        md.append("\n> [!NOTE]")
        md.append(r"> **Acceptance Gate Criteria:** Physical link validation requires $\ge 1,000$ frames per payload size collected on physical hardware.")
        md.append("> **Latency Disclaimer:** Mode C RTT = $(t_4 - t_1) - (t_3 - t_2)$. Approximate one-way latency ($\text{RTT}/2$) assumes symmetric propagation and includes UART buffering and RF transmission time.")
        md.append("> **Timing Disclaimer:** AUX timing measures observed pin transitions, not guaranteed RF airtime.")

        report_content = "\n".join(md)
        return report_content

    def execute(self, mode: str = "all"):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self.log_fd = open(self.output_file, "a", encoding="utf-8")

        self.open_ports()

        if self.fatal_serial_error:
            print(f"[FATAL ERROR] Serial communication failed: {self.serial_error_msg}", file=sys.stderr)
            report = self.analyze_results()
            print("\n" + report)
            with open(self.summary_file, "w", encoding="utf-8") as f:
                f.write(report)
            if self.log_fd:
                self.log_fd.close()
            return

        if not self.run_handshake():
            print("WARNING: Hardware handshake did not confirm both roles. Proceeding anyway...")

        if self.fatal_serial_error:
            print(f"[FATAL ERROR] Serial communication failed: {self.serial_error_msg}", file=sys.stderr)
            report = self.analyze_results()
            print("\n" + report)
            with open(self.summary_file, "w", encoding="utf-8") as f:
                f.write(report)
            if self.log_fd:
                self.log_fd.close()
            return

        self.run_register_read()

        if self.modem_failed:
            print("[FATAL ERROR] Aborting benchmark due to modem restoration failure.")
            report = self.analyze_results()
            print("\n" + report)
            with open(self.summary_file, "w", encoding="utf-8") as f:
                f.write(report)
            if self.log_fd:
                self.log_fd.close()
            return

        if not self.dry_run and (self.air_reg_status != "OK" or self.ground_reg_status != "OK"):
            print("[FATAL ERROR] Register verification failed; aborting physical benchmark.")
            report = self.analyze_results()
            print("\n" + report)
            with open(self.summary_file, "w", encoding="utf-8") as f:
                f.write(report)
            print(f"\nSummary report saved to: {self.summary_file}")
            print(f"Full JSONL log saved to: {self.output_file}")
            if self.log_fd:
                self.log_fd.close()
            return

        if mode in ["A", "all"]:
            for plen in self.payloads:
                if self.fatal_serial_error:
                    break
                self.run_mode_a_batch(plen, self.count, self.pacing_ms)

        if mode in ["B", "all"]:
            for plen in self.payloads:
                if self.fatal_serial_error:
                    break
                self.run_mode_b_batch(plen, self.count, self.pacing_ms)

        if mode in ["C", "all"]:
            for plen in self.payloads:
                if self.fatal_serial_error:
                    break
                self.run_mode_c_batch(plen, self.count, max(self.pacing_ms * 2, 100))

        report = self.analyze_results(mode=mode)
        print("\n" + report)

        with open(self.summary_file, "w", encoding="utf-8") as f:
            f.write(report)
        print(f"\nSummary report saved to: {self.summary_file}")
        print(f"Full JSONL log saved to: {self.output_file}")

        if self.log_fd:
            self.log_fd.close()


def main():
    parser = argparse.ArgumentParser(description="Dual-LRS Phase 1 Raw E22 Link Benchmark")
    parser.add_argument("--air-port", default="/dev/ttyACM0", help="Serial port for Air unit (BlackPill USB CDC)")
    parser.add_argument("--ground-port", default="/dev/ttyUSB0", help="Serial port for Ground unit (ESP32 USB UART)")
    parser.add_argument("--baud", type=int, default=115200, help="USB serial baud rate (default: 115200)")
    parser.add_argument("--mode", choices=["A", "B", "C", "all"], default="all", help="Test mode to execute")
    parser.add_argument("--counts", type=int, default=1000, help="Frames per payload size (default: 1000 for statistical validity)")
    parser.add_argument("--pacing", type=int, default=100, help="Pacing interval in ms (default: 100 ms / 10 Hz conservative; pass 50 for 20 Hz)")
    parser.add_argument("--output", default="audit/phase1_raw_log.jsonl", help="Output path for JSONL event log")
    parser.add_argument("--summary", default="audit/phase1_raw_summary.md", help="Output path for markdown summary")
    parser.add_argument("--dry-run", action="store_true", help="Simulate execution without opening serial ports")

    args = parser.parse_args()

    runner = DiagBenchRunner(
        air_port=args.air_port,
        ground_port=args.ground_port,
        baud=args.baud,
        output_file=args.output,
        summary_file=args.summary,
        pacing_ms=args.pacing,
        count=args.counts,
        dry_run=args.dry_run,
    )
    runner.execute(mode=args.mode)


if __name__ == "__main__":
    main()
