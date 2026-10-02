# Dual-LRS Phase 0 Baseline Audit & Hygiene Report (v2 — Senior Review Revisions Applied)

**Date:** October 1, 2026
**Scope:** Phase 0 Baseline Audit and Hygiene as mandated by `approach.md`
**Working Directory:** `/home/ritesh/Desktop/Dual-LRS`
**Reviewer:** Antigravity (Advanced Agentic Pair Programmer)

---

## 1. Executive Summary

This report establishes the baseline for the Dual-LRS codebase in accordance with [approach.md](file:///home/ritesh/Desktop/Dual-LRS/approach.md).

Dual-LRS is designed to provide a dedicated, open-source, bidirectional MAVLink telemetry and RC control link using the **STM32F411 BlackPill** (Air), **ESP32-WROOM-32** (Ground), and **Ebyte E22-900T30D** (1W UART LoRa transceivers).

### 1.1 Observed Baseline Capabilities (Reported Observations)
In live bench observations with an attached ArduPilot flight controller:
- **Telemetry Downlink:** Observed steady streaming to Ground USB `/dev/ttyUSB0` (ATTITUDE @ ~4.3 Hz, GPS @ ~2.0 Hz, VFR_HUD @ ~2.0 Hz, SYS_STATUS @ ~1.0 Hz, Heartbeat @ ~1.0 Hz).
- **Parameter Downlink:** In benchmark script runs, 1004 parameters were received in 56.9–57.2 seconds (~17.6 params/sec) with targeted single-hole recovery operational.
- **Urgent Traffic Bypass:** Flight mode switch acknowledgements and pre-arm warnings were observed downlinking in 650–950 ms via the priority cache.

### 1.2 Identified Architectural & Operational Limitations
1. **Transport-Level Framing & Fragment Handling Gap:** The current firmware has partial MAVLink stream parsing and heuristic priority caches, but **lacks robust transport-level fragment metadata (`transfer_id`, `fragment_offset`), packet-level retransmission, duplicate rejection, and fragment timeout recovery**. The observed mission upload stalls are currently treated as a **leading hypothesis** stemming from this transport gap combined with multi-slot delivery and home waypoint coordinate validation; final confirmation requires Phase 1 & 2 transport verification.
2. **Experimental TDM Configuration:** Slot payloads (`MAX_PAYLOAD_AIR_SLOT = 55`, `MAX_PAYLOAD_GROUND_SLOT = 10`) are **experimental/provisional** values pending rigorous Phase 1 timing measurements across payload boundaries.
3. **Security & Credential Remediation Completed:** Real credentials in headers, scripts, and lab notes have been redacted. Rotation of previously exposed credentials is required.
4. **RC Architecture Defined by User Decision:** User has confirmed:
   - Ground RC Input: **CRSF** protocol from transmitter (OpenI6X / EdgeTX).
   - Air RC Output: **CRSF** protocol to ArduPilot RC input UART.

---

## 2. Current Hardware Contract

### 2.1 Air Unit
- **Microcontroller:** STMicroelectronics STM32F411CEU6 "BlackPill" (ARM Cortex-M4 @ 100 MHz, 128 KB RAM, 512 KB Flash).
- **Radio Interface (`SerialRadio`):** Hardware USART1
  - Pin PA9: MCU TX $\rightarrow$ E22 RXD
  - Pin PA10: MCU RX $\leftarrow$ E22 TXD
- **Radio Control:**
  - Pin PB0: M0 (Operating Mode)
  - Pin PB1: M1 (Operating Mode)
  - Pin PB10: AUX (Busy input with internal pullup; LOW = Busy, HIGH = Ready)
- **Flight Controller Interface (`SerialTELEM`):** Hardware USART2
  - Pin PA2: MCU TX $\rightarrow$ ArduPilot TELEM RX
  - Pin PA3: MCU RX $\leftarrow$ ArduPilot TELEM TX
  - Default Baud: 115200 (Auto-baud permanently latches upon first valid heartbeat)
- **RC Output Interface (Confirmed Decision):** CRSF output to ArduPilot RC UART (pin allocation pending Phase 4).
- **Diagnostic / Maintenance:** Native USB CDC (`Serial` via `/dev/ttyACM0`). Emits diagnostic logs only. Does not carry FC MAVLink traffic.
- **Sync Indicator:** Pin PC13 (Onboard Blue LED, Active LOW).

### 2.2 Ground Unit
- **Microcontroller:** Espressif ESP32-WROOM-32 (Xtensa Dual-Core @ 240 MHz, 320 KB RAM, 4 MB Flash).
- **Radio Interface (`Serial2`):** Hardware UART2
  - GPIO17: MCU TX $\rightarrow$ E22 RXD
  - GPIO16: MCU RX $\leftarrow$ E22 TXD
- **Radio Control:**
  - GPIO21: M0
  - GPIO22: M1
  - GPIO19: AUX (Busy input with internal pullup; LOW = Busy, HIGH = Ready)
- **RC Input Interface (Confirmed Decision):** CRSF input from transmitter (e.g. OpenI6X rear bay / EdgeTX).
- **GCS Host Interface:** Native CP2102/CH340 USB-UART bridge on `Serial` (`/dev/ttyUSB0` @ 115200 baud). Carries binary MAVLink to/from GCS.
- **Sync Indicator:** GPIO2 (Onboard Blue LED, Active HIGH).

### 2.3 Ebyte E22-900T30D RF Modem Baseline
- **Hardware Engine:** Semtech SX1262 1000 mW (30 dBm) module controlled via proprietary Ebyte UART modem firmware.
- **Baud Rate:** 115200 baud, 8N1.
- **Frequency Register:** `REG2 = 0x17`; exact RF operating frequency is pending independent verification for this specific hardware module variant (nominally channel 23 / 915 MHz band).
- **Air Data Rate:** 62.5 kbps LoRa (REG0 = `0xE7`).
- **Sub-packet Size:** 64 bytes (REG1 bit 7 = `1`).
- **Active Power:** `E22_TX_POWER_BENCH = 3` (21 dBm / 125 mW; REG1 bits [1:0] = `11`) to prevent receiver saturation during bench tests.
- **Mode:** Transparent transmission (REG3 = `0x00`).

---

## 3. Current Software Architecture

The software is structured across the following modules:

```text
                  AIR UNIT (STM32F411)                    GROUND UNIT (ESP32)

         ArduPilot FC ◄──► USART2 (PA2/PA3)             GCS (QGC/MP) ◄──► USB CDC/UART
                               │                                           │
                               ▼                                           ▼
                     MavlinkHandler (48KB FIFO)                 MavlinkHandler (1KB FIFO)
                     - Stream byte parser                       - Stream byte parser
                     - Telemetry rate limiter                   - QGC TIMESYNC dropping
                     - Priority caches (_hbCache,               - Priority caches (_hbCache,
                       _missionCache, _urgentCache)               _missionCache, _urgentCache)
                     - RADIO_STATUS injector                    - RADIO_STATUS injector
                               │                                           │
                               ▼                                           ▼
                          TdmEngine                                   TdmEngine
                     - 50ms slot scheduler                      - 50ms time master
                     - 0B beacon PLL discipline                 - Sends 0B beacons
                     - LrsFrameHeader (5B)                      - LrsFrameHeader (5B)
                     - CRC-16-CCITT                             - CRC-16-CCITT
                               │                                           │
                               ▼                                           ▼
                           E22Driver                                   E22Driver
                     - Hardware USART1                          - Hardware Serial2
                     - AUX busy polling                         - AUX busy polling
                               │                                           │
                               └─────────── LoRa RF Channel ───────────────┘
```

### Architectural Deficiencies Identified:
1. **Coupled MAVLink & Transport Logic:** `src/mavlink_handler.cpp` mixes application-level MAVLink message filtering, queue management, cache prioritization, CRC validation, and radio slot packet slicing into a single 1,000+ line class.
2. **Absence of Transport Layer (`link_transport`):** The TDM frame header contains only sequence number, packet type, and payload length. When a 49-byte MAVLink packet is fragmented into 10-byte slices over 5 TDM slots, there is no transport header identifying `transfer_id`, `fragment_offset`, or total length. Missing fragments cannot be detected or requested at the transport level.
3. **Absence of RC Subsystem (`rc_endpoint`):** Firmware currently has no CRSF frame parser, channel unpacker, failsafe timer, or RC output generator.

---

## 4. Existing Uncommitted Changes & Worktree Status

The Git worktree is dirty. The complete status is tracked and preserved as follows:

### Modified Tracked Files
- `M .gitignore`: Added `audit.md` to ignore list.
- `M include/config.h`:
  - `MAX_PAYLOAD_AIR_SLOT = 55` (experimental).
  - `MAX_PAYLOAD_GROUND_SLOT = 10` (experimental).
  - Sanitized Wi-Fi credentials to non-secret placeholders.
- `M include/mavlink_handler.h`: Added `_missionCache[64]` variables and `AirUplinkState` enum.
- `M src/main.cpp`: Fixed `requestStream()` IDs (10 for ATTITUDE, 11 for VFR_HUD); permanently latched FC baud.
- `M src/mavlink_handler.cpp`: Added MAVLink CRC lookup table, `isMissionMavlinkMessage()`, `validateMavlinkCrc()`, dedicated mission cache, and ground frame reassembly.
- `M src/tdm_engine.cpp`: Restricted Air PLL sync to 0-byte `HEARTBEAT_SYNC` beacons; refined graduated slew steps; added rolling `link_quality`.
- `M tools/download_all_params.py`: Updated parameter count check.
- `M tools/test_rf_radxa_to_laptop.py`: Refactored to require environment variables (`RADXA_HOST`, `RADXA_PASSWORD`) without cleartext credentials.
- `M README.md`: Synchronized slot definitions with current experimental source values and added required disclaimers.
- `M docs/TDM_PROTOCOL.md`: Synchronized slot definitions with current experimental source values and added required disclaimers.

### Untracked Files (Explicitly Preserved)
- `?? approach.md`: Core architectural contract.
- `?? audit/`: Dedicated audit directory containing this report (`audit/PHASE_0_REPORT.md`).
- `?? tools/diagnose_mission.py`: Experimental mission diagnostic probe.
- `?? tools/test_clean_mission.py`: Experimental clean mission benchmark.
- `?? tools/test_live_mission.py`: Experimental 3-cycle live mission test.

### Gitignored & Local Context Files
- `context.md`: Detailed engineering log (sanitized with `[REDACTED]` credentials).
- `audit.md`: Earlier senior review notes.
- `bench_params.json`: Output from parameter benchmark.

---

## 5. Documentation Inconsistencies & Reconciliations

| Item | `include/config.h` (Code) | `README.md` | `docs/TDM_PROTOCOL.md` | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Air Slot Payload** | 55 bytes | 55 bytes *(provisional)* | 55 bytes *(provisional)* | Reconciled with Phase 1 validation disclaimer |
| **Ground Slot Payload** | 10 bytes | 10 bytes *(provisional)* | 10 bytes *(provisional)* | Reconciled with Phase 1 validation disclaimer |
| **Air Max Frame Over-the-Air** | 62 bytes (5B+55B+2B) | 62 bytes *(provisional)* | 62 bytes *(provisional)* | Fits inside 64B E22 sub-packet |
| **RF Frequency Notation** | `REG2 = 0x17` | `REG2 = 0x17` | `REG2 = 0x17` | Standardized: exact RF frequency pending verification |
| **PLL Transit Formula** | Locked to 0B beacon (45.2 ms) | Noted as 0B beacon baseline | Noted as 0B beacon baseline | Leading approach documented |

---

## 6. Security and Credential Hygiene Findings

### Comprehensive Privacy Review Scope
The privacy audit covered:
- Source code (`src/`, `include/`)
- Test scripts and utilities (`tools/`)
- Documentation and lab notes (`docs/`, `context.md`, `audit.md`, `README.md`)
- Build configurations (`platformio.ini`)
- Local untracked files and generated artifacts (`bench_params.json`)

### Actions Taken:
1. **`include/config.h` (Lines 37–40):** Cleartext Wi-Fi credentials removed and replaced with explicit placeholders (`"DISABLED_PLACEHOLDER_SSID"`, `"DISABLED_PLACEHOLDER_PASSWORD"`).
2. **`tools/test_rf_radxa_to_laptop.py` (Lines 27, 54):** Hardcoded SSH password and private IP removed. The script now reads `RADXA_HOST` and `RADXA_PASSWORD` from environment variables, aborting with an explicit error if missing.
3. **`context.md` (Lines 16, 495, 552):** Companion computer private IPs and credentials replaced with `[REDACTED_IP]`, `[REDACTED_USER]`, and `[REDACTED]`.

> [!WARNING]
> **Credential Rotation Notice:** Any real passwords, Wi-Fi keys, or SSH credentials that were previously present in local or versioned files must be rotated on the affected devices and networks immediately.

---

## 7. Build and Test Reproducibility Baseline

### 7.1 Verified Tooling & Build Environment
- **PlatformIO Core:** `version 6.2.0`
- **ST STM32 Platform:** `ststm32 @ 20.0.0`
  - Framework: `framework-arduinoststm32 @ 4.30000.0 (3.0.0)`
  - Toolchain: `toolchain-gccarmnoneeabi @ 1.120301.0 (12.3.1)`
  - Uploader: `tool-dfuutil @ 1.11.0`
- **Espressif 32 Platform:** `espressif32 @ 7.1.3`
  - Framework: `framework-arduinoespressif32 @ 4.20017.260907+sha.dcc1105b`
  - Toolchain: `toolchain-xtensa-esp32 @ 8.4.0+2021r2-patch5`
  - Uploader: `tool-esptoolpy @ 2.41100.260830 (4.11.0)`
- **Host Python Environment:** Python 3.12 (with `pyserial`, `pymavlink`).

### 7.2 Exact Build Verification Command
```bash
~/.platformio/penv/bin/pio run -e dual_lrs_air -e dual_lrs_ground -e dual_lrs_ground_esp32
```
**Results (Exit Code: 0):**
- `dual_lrs_air`: **SUCCESS** (RAM: 7,908 B / 6.0%, Flash: 34,684 B / 6.6%, Took 1.49s).
- `dual_lrs_ground` (STM32 BlackPill Ground): **SUCCESS** (RAM: 7,876 B / 6.0%, Flash: 30,860 B / 5.9%, Took 1.58s).
- `dual_lrs_ground_esp32` (ESP32 Ground): **SUCCESS** (RAM: 22,736 B / 6.9%, Flash: 280,165 B / 21.4%, Took 2.50s).

---

## 8. Feature Readiness Matrix

| Feature | Current Status | Basis of Assessment | Next Phase Action |
| :--- | :---: | :--- | :--- |
| **Raw RF Link** | **FUNCTIONAL** | E22 TX/RX operational; AUX timing ~30 ms observed | Phase 1: formal test harness |
| **Bidirectional Heartbeat** | **FUNCTIONAL** | 1.0 Hz exchange verified on live bench | Retain in MAVLink layer |
| **Telemetry Downlink** | **FUNCTIONAL** | Attitude (4.3 Hz), GPS (2 Hz), HUD (2 Hz) verified | Retain in MAVLink layer |
| **Parameter Read** | **FUNCTIONAL** | 1004 params in 57s observed with hole recovery | Phase 3 acceptance test |
| **Commands & Modes** | **FUNCTIONAL** | Mode switches delivered in 650–950 ms observed | Phase 3 acceptance test |
| **Mission Download** | **PARTIAL** | `MISSION_COUNT` fetched in 0.19s; multi-item fragile | Requires Phase 2 transport framing |
| **Mission Upload** | **UNSTABLE** | Multi-slot fragment drops cause timeouts | Requires Phase 2 transport framing |
| **RC Input (Ground)** | **NOT IMPLEMENTED** | User confirmed: CRSF from transmitter | Implement in Phase 4 |
| **RC Output (Air)** | **NOT IMPLEMENTED** | User confirmed: CRSF to ArduPilot UART | Implement in Phase 4 |
| **RC Failsafe** | **NOT IMPLEMENTED** | No failsafe state machine in firmware | Implement in Phase 4 |
| **Link State Machine** | **PARTIAL** | Basic `Sync: 0/1`; lacks formal state transitions | Implement in Phase 5 |

---

## 9. Phase 0 Acceptance-Gate Table

| Criterion (from `approach.md` §4) | Status | Evidence & Resolution |
| :--- | :---: | :--- |
| **1. Identify exact E22 module variant & config** | **PASS** | Ebyte E22-900T30D, SX1262, 115200 8N1, 62.5 kbps LoRa, `REG2=0x17`, transparent mode. |
| **2. Confirm Air MCU and Ground MCU target** | **PASS** | Air: STM32F411CEU6 BlackPill. Ground: ESP32-WROOM-32 (with STM32F411 ground built & verified). |
| **3. Confirm RC input protocol from transmitter** | **PASS** | **Resolved by User:** CRSF out of transmitter (FlySky OpenI6X / EdgeTX). |
| **4. Confirm FC RC output protocol** | **PASS** | **Resolved by User:** CRSF out of Air BlackPill to ArduPilot RC UART. |
| **5. Confirm FC telemetry UART and baud** | **PASS** | BlackPill PA2/PA3 to FC TELEM @ 115200 baud, permanently latched. |
| **6. Capture E22 AUX timing baseline** | **PROVISIONAL** | Initial observations captured (~30 ms for 55B frame; ~10.2 ms for 0B beacon). Formal curve to be mapped in Phase 1. |
| **7. Remove credentials from source and notes** | **PASS** | Cleartext passwords sanitized in `config.h`, `test_rf_radxa_to_laptop.py`, and `context.md`. |
| **8. Reconcile README, context, TDM, & config** | **PASS** | Documentation aligned with source experimental values with clear provisional disclaimers. |
| **9. Record uncommitted state** | **PASS** | Complete Git status, diff stats, and untracked file inventory recorded. |

---

## 10. Summary of Resolved Blockers

1. **RC Input & Output Protocol Decision:** **RESOLVED.** The user has selected **CRSF** for both Ground input and Air output.
2. **Credential Hygiene:** **RESOLVED.** All real passwords and infrastructure IPs have been removed from source headers, test scripts, and lab notes.
3. **Build Reproducibility:** **RESOLVED.** All 3 environments (`dual_lrs_air`, `dual_lrs_ground`, `dual_lrs_ground_esp32`) build with 100% success and documented toolchain versions.
4. **Documentation Alignment:** **RESOLVED.** Documentation and configuration agree, with provisional disclaimers pending Phase 1 measurements.

---

## 11. Proposed Next Step

With all Phase 0 hygiene and baseline items completed and reviewed:
1. **User confirms acceptance of Phase 0 closure.**
2. **Authorize Phase 1 (Prove Raw E22 Link):**
   - Create a deterministic raw E22 test harness in `tools/` that streams sequence-numbered test payloads in both directions (no MAVLink, no RC).
   - Measure sequence loss, duplicate handling, CRC rejection, exact AUX busy durations across 0B, 10B, 30B, and 55B payloads, and reconnection times.
