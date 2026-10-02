# Phase 2: Transport Protocol, Engine & RC Pipeline — Verification Report (Corrected Baseline)

**Date:** 2026-10-01
**Project:** Dual-LRS (Air: STM32F411 BlackPill, Ground: ESP32-WROOM-32)
**Status:** `PHASE 2 ISOLATED IMPLEMENTATION & HOST VERIFICATION PASSED — LIVE SMOKE TEST RECORDED (AUDIT BASELINE)`
**Target Hardware:**
- Ground Unit: ESP32 Dev Module (`/dev/ttyUSB0`)
- Air Unit: STM32F411CEU6 BlackPill (`/dev/ttyACM0`)
- Transmitter: FlySky FS-i6X running OpenI6X (Trainer Port connected to ESP32 GPIO 13)
- Modems: Ebyte E22-900T30D (868/915 MHz, UART 115200, Air Data Rate 62.5 kbps)

---

## 1. Executive Summary & Verification Boundaries

Phase 2 replaces legacy ad-hoc packet handling with an isolated, bounded, multi-channel transport protocol and engine, establishes a multi-protocol handset RC ingest adapter, and validates the implementation across host test suites and embedded compilation targets.

### Implementation Scope & Boundaries:
1. **Isolated Protocol Specification & Implementation:**
   - Implemented in [`include/transport_protocol.h`](../include/transport_protocol.h), [`include/transport_engine.h`](../include/transport_engine.h), and [`src/transport_engine.cpp`](../src/transport_engine.cpp).
   - Wire format strictly enforces single-burst framing: **Maximum frame size = 64 bytes**, **Maximum single-burst payload = 49 bytes**, **Header = 13 bytes**, **CRC = 2 bytes**.
   - Specific packed RC frame size: **39 bytes** (13B Header + 24B `TransportPackedRc` + 2B CRC).
   - Sync magic word: **`0x44 0x4C`** (`'D'`, `'L'`), Protocol version: **`0x01`**.
2. **Production Code Isolation & Worktree Preservation:**
   - **No transport-engine calls or integration logic were added to the production runtime during Phase 2.**
   - Pre-existing uncommitted changes in production runtime files ([`src/main.cpp`](../src/main.cpp), [`src/mavlink_handler.cpp`](../src/mavlink_handler.cpp), [`src/tdm_engine.cpp`](../src/tdm_engine.cpp)) were strictly preserved.
   - Production targets (`dual_lrs_air`, `dual_lrs_ground_esp32`) compile cleanly, proving absence of symbol collisions, but runtime transport integration is explicitly deferred to Phase 3.
3. **Handset RC Ingestion & Adapter:**
   - Implemented in [`include/crsf_protocol.h`](../include/crsf_protocol.h) and [`include/rc_adapter.h`](../include/rc_adapter.h).
   - Ingests standard CRSF (420,000 baud) and FlySky OpenI6X inverted serial frames (400,000 baud) on ESP32 GPIO 13.
   - Packs 16 channels (11-bit resolution) into a 24-byte payload encapsulated within the 39-byte RF transport frame.
4. **Physical Verification Classification:**
   - Physical testing on hardware is classified as **Operator-Reported Physical Smoke-Test Evidence** conducted via the diagnostic harness ([`src/diag_main.cpp`](../src/diag_main.cpp)) and live monitor ([`tools/monitor_rc_live.py`](../tools/monitor_rc_live.py)).

---

## 2. Wire Protocol Specification & Memory Safety

### 2.1 Frame Layout & Wire Format
The transport frame is bounded to the E22 modem hardware sub-packet boundary (64 bytes, `REG1=0x83`):

```text
+-------------------+-----------------------------+-----------------------+
|  TransportHeader  |  Payload (0..49 Bytes Max)  |  CRC-16-CCITT (2B)   |
|     13 Bytes      |     e.g. RC Payload (24B)   |  Big-Endian (0x1021)  |
+-------------------+-----------------------------+-----------------------+
| <----------------------- Total Frame: 15..64 Bytes -------------------> |
```

#### Exact Header Structure (`sizeof(TransportHeader) == 13`):
```text
Byte 0:       magic0          ('D' = 0x44)
Byte 1:       magic1          ('L' = 0x4C)
Byte 2:       version         (0x01)
Byte 3:       channel         (0x01=RC, 0x02=LINK, 0x03=MAV_UP, 0x04=MAV_DOWN)
Byte 4:       flags           (Bitmask: RELIABLE, IS_ACK, IS_NACK, FIRST_FRAG, LAST_FRAG)
Bytes 5-6:    sequence        (uint16_t, Big-Endian)
Bytes 7-8:    transfer_id     (uint16_t, Big-Endian)
Bytes 9-10:   fragment_offset (uint16_t, Big-Endian)
Bytes 11-12:  payload_length  (uint16_t, Big-Endian, <= 49)
Trailing CRC (computed over Bytes 0 .. 12+payload_length):
Bytes 13+payload_length:     CRC16 (MSB, Big-Endian)
Bytes 13+payload_length + 1: CRC16 (LSB, Big-Endian)
```

#### Size Distinctions:
- **Maximum Transport Frame (`TRANSPORT_MAX_FRAME_SIZE`):** **64 bytes**.
- **Maximum Single-Burst Payload (`TRANSPORT_MAX_SINGLE_BURST_PAYLOAD`):** **49 bytes** ($64 - 13 - 2 = 49$).
- **Transport Overhead (`TRANSPORT_OVERHEAD_SIZE`):** **15 bytes** (13B Header + 2B CRC).
- **Packed RC Transport Frame (Calculated):** **39 bytes** ($13\text{B Header} + \text{sizeof(TransportPackedRc)} [24\text{B}] + 2\text{B CRC} = 39\text{ bytes}$).

### 2.2 Wire Helpers & Memory Safety
- **Endianness Enforcement:** All multi-byte header fields are encoded and decoded using explicit shift operations (`transport_read_u16_be`, `transport_write_u16_be`). 32-bit arithmetic inside the engine is native C++ range checking, avoiding unaligned memory access and integer overflow. No `reinterpret_cast` of unaligned memory is permitted.
- **Checksum:** CRC-16-CCITT (polynomial `0x1021`, initial `0xFFFF`) is computed over bytes `0 .. (12 + payload_length)`. Wire CRC is appended and validated in big-endian order.
- **Channel Bit-Packing:** 16 channels $\times$ 11 bits = 176 bits = 22 bytes. Verified with exact boundary test vectors (all zeros = 22x `0x00`, all 2047 = 22x `0xFF`).

---

## 3. Transport Engine Architecture & State Machines

### 3.1 Streaming Parser (`TransportParser`)
- Maintains a streaming state machine (`SEEK_MAGIC0`, `SEEK_MAGIC1`, `READ_HEADER`, `READ_PAYLOAD`, `READ_CRC`).
- Sync detection matches `0x44` followed by `0x4C`.
- Enforces strict frame bounds: Rejects any frame with `payload_length > 49` or `payload_length > (64 - 15)`.
- Validates node direction: Drops frames where source node role mismatches channel direction permissions.

### 3.2 Bounded Reassembler (`TransportReassembler`)
The reassembly engine manages reassembly of multi-fragment datagrams up to 512 bytes (`TRANSPORT_MAX_TRANSFER_SIZE`):
- **Defensive Boundary Validation:**
  - Rejects null or zero-length payloads with `TransportNackReason::BAD_CRC` (`!payload || payload_length == 0`).
  - Guards against integer overflow: Rejects when `(uint32_t)fragment_offset + payload_length > TRANSPORT_MAX_TRANSFER_SIZE`.
- **Bounded Range Tracking (Out-of-Order Acceptance):**
  - Maintains a table of up to 11 received fragment intervals (`ReceivedRange ranges_[11]`).
  - Accepts non-contiguous out-of-order fragments within the active transfer.
  - Re-acknowledges exact duplicates without buffer corruption.
  - Detects and rejects partial-overlap conflicts (`OVERLAP_CONFLICT`) without overwriting previously validated bytes.
  - `check_coverage()` verifies contiguous byte coverage from offset 0 to `total_size_` before marking `is_complete_ = true`.
- **Transfer Lifecycle & Atomic Takeover:**
  - Incomplete transfers are strictly protected: Competing `FIRST_FRAG` packets with differing `transfer_id` are rejected as `BUFFER_FULL`.
  - Once complete, datagram is retained for consumer read access.
  - If a new `FIRST_FRAG` arrives after completion, it atomically clears the completed transfer and begins the new transfer without requiring manual intervention.
  - Inactivity timeout: Aborts uncompleted transfers after 1000 ms (`TRANSPORT_TRANSFER_TIMEOUT_MS`).

### 3.3 Reliability & Retry Manager (`TransportRetryManager`)
- Implements stop-and-wait ARQ for reliable channels (`LINK_CONTROL`, `MAVLINK_UPLINK`, `MAVLINK_DOWNLINK`).
- Retransmission Timeout ($T_{\text{RTO}}$): **200 ms** (`TRANSPORT_RETRY_TIMEOUT_MS = 200`), derived as 2 $\times$ 90 ms TDM cycle + 20 ms scheduling margin.
- Retransmission Limit: Strictly **5 retry attempts** (`TRANSPORT_MAX_RETRIES = 5`) before transfer abort.

### 3.4 Atomic RC Mailbox & Failsafe Monitor
- **`TransportRcMailbox`:** Overwrite mailbox holding latest 16 channels with monotonic sequence tracking. Zero dynamic allocation.
- **`TransportRcFailsafe`:**
  - Failsafe timeout: **500 ms** (`TRANSPORT_RC_FAILSAFE_TIMEOUT_MS = 500`) of channel silence asserts protocol failsafe flag.
  - Glitch-free restoration: Requires **3 consecutive valid frames** (`TRANSPORT_RC_RESTORE_FRAME_COUNT = 3`) within timeout to disengage failsafe.

---

## 4. TDM Schedule & Timing Budget — Approved Bench Baseline

The timing baseline is established in [`docs/PHASE_2_TRANSPORT_SPEC.md`](../docs/PHASE_2_TRANSPORT_SPEC.md) based on Phase 1 physical measurements:

```text
0.0 ms                 32.0 ms 37.0 ms                             82.0 ms 90.0 ms
|-----------------------|-------|-----------------------------------|-------|
| Slot 1: Ground Uplink | Guard | Slot 2: Air Downlink              | Guard |
| (RC + Link Control)   | Gap 1 | (MAVLink Telemetry & Commands)    | Gap 2 |
| Duration: 32.0 ms     | 5.0 ms| Duration: 45.0 ms                 | 8.0 ms |
| Max Frame: <= 39 B    |       | Max Frame: <= 64 B (Single-Burst) |       |
```

### Slot Arithmetic & Headroom Summary:
1. **Slot 1: Ground Uplink ($32.0\text{ ms}$):**
   - Transmits 1 RC frame (39 bytes total).
   - Phase 1 empirical transfer duration: $\approx 28.5\text{ ms}$.
   - **Headroom:** $32.0\text{ ms} - 28.5\text{ ms} = \mathbf{+3.5\text{ ms}}$.
2. **Guard Gap 1 ($5.0\text{ ms}$):**
   - Accommodates Ground E22 AUX fall time, RF transmission decay, and Air UART FIFO drain.
3. **Slot 2: Air Downlink ($45.0\text{ ms}$):**
   - Transmits 1 full single-burst frame (up to 64 bytes total, 49B payload).
   - Phase 1 empirical transfer duration: $\mathbf{38.68\text{ ms}}$ (Mode C 53B payload).
   - **Headroom:** $45.0\text{ ms} - 38.68\text{ ms} = \mathbf{+6.32\text{ ms}}$.
4. **Guard Gap 2 ($8.0\text{ ms}$):**
   - Accommodates Air E22 AUX return to IDLE, Ground UART FIFO drain, and PLL clock phase drift.
5. **Cycle Period & Frequency:**
   $$T_{\text{cycle}} = 32.0 + 5.0 + 45.0 + 8.0 = \mathbf{90.0\text{ ms}}$$
   $$f_{\text{cycle}} = \frac{1000}{90.0} \approx \mathbf{11.11\text{ Hz}\ (\sim 11.1\text{ Hz})}$$
6. **Retransmission Timeout ($T_{\text{RTO}}$):**
   $$T_{\text{RTO}} = 2 \times 90.0\text{ ms} + 20.0\text{ ms} = \mathbf{200\text{ ms}}$$

---

## 5. Handset RC Ingestion & Adapter Architecture

### 5.1 Multi-Protocol Handset Support ([`include/rc_adapter.h`](../include/rc_adapter.h))
The `RcGroundAdapter` provides a streaming parser supporting two handset serial inputs:
1. **Standard CRSF:** Ingests `0xC8` (or `0xEE`/`0xEA`/`0xEC`) sync, validates CRC-8 (DVB-S2, poly `0xD5`), extracts 22 packed channel bytes.
2. **Inverted S.BUS (OpenI6X):** Ingests 25-byte standard and 26-byte OpenI6X extended frames starting with `0x0F` and ending with `0x00`.

### 5.2 Physical Layer Conditioning (FS-i6X Trainer Port)
- **Signal Pin Identification:** ESP32 edge counters (`CMD:MEASURE_PULSES`) verified active transitions on GPIO 13 (>30,000 edges/sec) while GPIO 14 remained idle (0 edges/sec). Handset signal wire confirmed on GPIO 13.
- **Transistor Inversion:** The FS-i6X trainer port routes through an internal NPN inverter transistor on the radio motherboard, pulling idle UART voltage LOW. Resolved in silicon via `invert = true` on ESP32 `HardwareSerial(1)`.
- **Baud Rate & Cadence:** Serial stream operates at **400,000 baud** arriving at **~94 Hz** (every 10.6 ms).

### 5.3 Degradation Assessment (CRSF vs Inverted S.BUS)
Zero degradation occurs between the handset and Ground ESP32:
- Both protocols encode 16 channels with **11-bit resolution** (2048 discrete steps, ~0.5 µs precision).
- Wire transfer time: 26 bytes at 400,000 baud takes **0.65 ms** (versus 0.62 ms for 420,000 baud CRSF).
- Over-the-air RF transport is identical: The Ground unit encapsulates decoded channels into the Dual-LRS 39-byte RF frame. The Air unit reconstructs standard 420,000 baud CRSF frames to the flight controller.

---

## 6. Physical Smoke-Test Evidence (Operator-Reported)

The physical link was verified on live hardware using the diagnostic harness. Because formal automated logging artifacts were not captured to disk, these results are classified as **Operator-Reported Physical Smoke-Test Evidence**:

### Smoke-Test Observations:
- **Active Diagnostic Session:** Verified with transmitter active, Ground ESP32 (`/dev/ttyUSB0`), and Air BlackPill (`/dev/ttyACM0`).
- **Telemetry Output Sample:**
  ```text
  Handset Ingest:  [ ACTIVE ] Ingesting OpenI6X on ESP32 GPIO 13
    CRSF Frames In :    142  | Rate:  93.7 Hz | CRC Errors: 0
    Ground RF TX   :     14 pkts | Active CRSF Pin: GPIO 13

  Air Receiver:    [ RF LINK ACTIVE ]
    Air RF RX      : 26572 pkts | Rate: diagnostic-session measurement; not the
                       production 90 ms TDM RC rate (~11.1 Hz) | Failsafe Events: 0
  -------------------------------------------------------------------------
  PRIMARY CHANNELS (Sticks):
    Ch 1 (Roll ): [              |##           ] 1581 us (CRSF 1124) [MATCH]
    Ch 2 (Pitch): [              |##           ] 1593 us (CRSF 1144) [MATCH]
    Ch 3 (Thr  ): [##############|             ] 1000 us (CRSF  124) [MATCH]
    Ch 4 (Yaw  ): [##############|             ] 1017 us (CRSF  200) [MATCH]
  ```
- **Stick Tracking:** Real-time deflection of sticks and switches produced verified `[MATCH]` status between Ground input and Air output with zero failsafe events during active transmission.

### Potentiometer Crosstalk & Calibration Root Cause:
- **Phenomenon:** Rotating knobs (`VrA`, `VrB`) caused minor shifts (10–30 raw counts) on adjacent analog channels.
- **Root Cause:** FS-i6X hardware design routes all 6 analog potentiometers across a shared 3.3V analog rail and internal ADC multiplexer. Fresh OpenI6X installations use default uncalibrated EEPROM endpoints (Min=0, Mid=2048, Max=4095), leaving stick centers at raw count 1124 (~1580 µs) and internal ADC filtering inactive.
- **Resolution:**
  1. Onboard 30-second OpenI6X calibration (`Radio Setup` $\rightarrow$ `CALIBRATION`) normalizes hardware endpoints in EEPROM.
  2. Delivered [`tools/calibrate_rc.py`](../tools/calibrate_rc.py): Provides 3-point piecewise linear scaling, a 4-count center deadband filter to eliminate potentiometer crosstalk, and persists profiles to [`tools/rc_calibration.json`](../tools/rc_calibration.json).
  3. Delivered [`tools/monitor_rc_live.py`](../tools/monitor_rc_live.py): Full 10-channel live visualizer with individual ASCII meters and calibrated microsecond display.

---

## 7. Verification Test Matrix

| Verification Target | Command Executed | Result | Details |
| :--- | :--- | :--- | :--- |
| **Python Protocol Suite** | `python3 tools/test_transport_protocol.py -v` | **25/25 PASS** | Wire serialization, big-endian encodings, reassembly lifecycle, duplicate ACK, overlap conflict. |
| **C++ Protocol Suite** | `g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_protocol.cpp -o /tmp/test_tp && /tmp/test_tp` | **5/5 PASS** | Header, link, ACK/NACK, and RC exact wire-byte verification. |
| **C++ Engine Suite** | `g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_engine.cpp src/transport_engine.cpp -o /tmp/test_te && /tmp/test_te` | **9/9 PASS** | Parser, fragmenter, retry buffer, reassembly bounds, atomic takeover, mailbox, failsafe. |
| **C++ CRSF Suite** | `g++ -std=c++17 -Wall -Wextra -I include tools/test_crsf_protocol.cpp src/transport_engine.cpp -o /tmp/test_cp && /tmp/test_cp` | **4/4 PASS** | CRC-8, channel roundtrips, streaming parser, transport integration. |
| **C++ RC Adapter Suite** | `g++ -std=c++17 -Wall -Wextra -I include tools/test_rc_adapter.cpp src/transport_engine.cpp -o /tmp/test_ra && /tmp/test_ra` | **2/2 PASS** | Ground ingest (CRSF & S.BUS) $\rightarrow$ RF transport $\rightarrow$ Air decoder. |
| **Embedded Build (Air)** | `pio run -e diag_air` | **SUCCESS** | STM32F411 BlackPill diagnostic build (RAM: 7.5%, Flash: 8.2%). |
| **Embedded Build (Ground ESP32)**| `pio run -e diag_ground_esp32` | **SUCCESS** | ESP32 Dev Module diagnostic build (RAM: 7.8%, Flash: 24.2%). |
| **Production Build (Air STM32)** | `pio run -e dual_lrs_air` | **SUCCESS** | STM32F411 air production target compiles cleanly. |
| **Production Build (Ground STM32)** | `pio run -e dual_lrs_ground` | **SUCCESS** | STM32F411 ground production target compiles cleanly. |
| **Production Build (Ground ESP32)** | `pio run -e dual_lrs_ground_esp32` | **SUCCESS** | ESP32 ground production target compiles cleanly. |

---

## 8. Compliance & Phase 3 Transition Criteria

### Compliance Checklist:
- [x] **Framing Constants Verified:** Magic bytes `0x44 0x4C`, Version `0x01`, Header `13B`, Max Frame `64B`, Max Payload `49B`, RC Frame `39B`.
- [x] **Timing Arithmetic Grounded:** Ground slot $= 32.0\text{ ms}$, Guard 1 $= 5.0\text{ ms}$, Air slot $= 45.0\text{ ms}$, Guard 2 $= 8.0\text{ ms}$, Cycle $= 90.0\text{ ms}$ (~11.1 Hz), $T_{\text{RTO}} = 200\text{ ms}$.
- [x] **Engine Accuracy Verified:** Reassembler implements bounded range tracking (`ReceivedRange ranges_[11]`), out-of-order acceptance, partial-overlap rejection, duplicate re-ACK, and contiguous coverage checking.
- [x] **Worktree & Production Isolation Preserved:** No transport-engine calls were added to production runtime files; uncommitted modifications in production files were preserved.
- [x] **Physical Evidence Classified:** Physical test data accurately labeled as operator-reported smoke-test evidence.

### Conclusion & Recommendation:
The isolated transport protocol, engine, and RC adapter implementations are fully verified across unit tests and embedded builds. **This corrected report serves as the official Phase 2 engineering baseline.** Phase 3 (Production TDM & MAVLink Integration) may proceed following formal review of this document.
