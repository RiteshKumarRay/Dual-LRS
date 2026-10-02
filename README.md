# Dual-LRS — Open Source Bidirectional Long Range Telemetry System

> **High-performance, bidirectional 915 MHz LoRa telemetry system for ArduPilot drones, engineered on the STM32F411 BlackPill, ESP32-WROOM-32, and Ebyte E22-900T30D (1W LoRa).**

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Build%20Passing-brightgreen.svg)](https://platformio.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Hardware: Air](https://img.shields.io/badge/Air%20MCU-STM32F411CEU6-blue.svg)]()
[![Hardware: Ground](https://img.shields.io/badge/Ground%20MCU-ESP32--WROOM--32%20%7C%20STM32F411-brightgreen.svg)]()
[![Radio: UART LoRa modem](https://img.shields.io/badge/Radio-E22--900T30D%20UART-orange.svg)]()
[![MAVLink](https://img.shields.io/badge/Protocol-MAVLink%20v1%20%2F%20v2-purple.svg)]()

---

## 1. Overview & Problem Statement

Dual-LRS provides an open-source, ultra-low-cost, long-range bidirectional telemetry link between autonomous ArduPilot unmanned aerial vehicles (UAVs) and ground control stations (**QGroundControl**, **Mission Planner**).

### The Traditional Problem
Commercial UAV setups typically rely on expensive, bulky, or proprietary radio hardware:
* Standard SiK 3DR radios (915 MHz FSK) transmit at only 100 mW, suffering packet loss and dropouts under non-line-of-sight conditions.
* High-power telemetry systems (such as RFD900) cost $150–$250 per pair.
* Traditional setups require independent hardware for telemetry and RC links, increasing airframe weight, antenna count, and electromagnetic interference.

### The Dual-LRS Solution
Dual-LRS uses affordable Ebyte E22-900T30D UART LoRa modems paired with an
STM32F411 BlackPill Air unit and an ESP32 Ground unit. The modem hides the
underlying radio chip behind its own UART firmware, so this project is an
independent modem-based protocol implementation, not a directly flashable mLRS
build.

The current verified baseline provides:
* A bounded 64-byte modem-frame transport with CRC-16 and 90 ms TDM scheduling.
* RC transport from the Ground unit to hardware CRSF output on the Air unit.
* Development-mode MAVLink fragmentation, reassembly, and basic RC coexistence.
* Host tests and embedded build targets for the transport, RC, and TDM layers.

Full MAVLink reliability, mission transactions, parameter transactions, and
reconnection under packet loss remain development work; they are not claimed as
flight-ready features.

---

## 2. Key Architecture & Features

### A. Deterministic ~11.1 Hz TDM Scheduler (90.0 ms Period)
* **Ground Transmit Slot (32 ms / 0.0 – 32.0 ms):** Transmits unconditional 39B packed RC control frames (`RC_CONTROL`) from the handset, with interleaved GCS uplink commands (`MAVLINK_UPLINK`) and Link Control sync frames.
* **Turnaround Guard Gap 1 (5 ms / 32.0 – 37.0 ms):** Turnaround margin allowing receiver processing and AUX line settling before Air transmits.
* **Air Transmit Slot (45 ms / 37.0 – 82.0 ms):** Transmits high-throughput downlink telemetry (`ATTITUDE`, `GLOBAL_POSITION_INT`, `SYS_STATUS`, `VFR_HUD`, `PARAM_VALUE`). Payload capped at 49 bytes (`MAX_PAYLOAD_AIR_SLOT = 49`) for strict single-burst transmission ($\le 64\text{B}$ wire frame).
* **Turnaround Guard Gap 2 (8 ms / 82.0 – 90.0 ms):** Turnaround margin and PLL synchronization window before the 90.0 ms frame wraps.

```text
                              ← 90 ms TDM Frame (~11.1 Hz) →
┌───────────────────────────┬─────────┬──────────────────────────────────┬─────────┐
│     Slot 1: Ground        │  Gap 1  │       Slot 2: Air Transmit       │  Gap 2  │
│   RC / Commands (32 ms)   │  Guard  │     Telemetry Downlink (45 ms)   │  Guard  │
│    RC_CONTROL, GCS Cmds   │  5 ms   │    ATTITUDE, GPS, PARAMS, HUD    │  8 ms   │
└───────────────────────────┴─────────┴──────────────────────────────────┴─────────┘
      GROUND TRANSMITS                               AIR TRANSMITS
```

### B. Length-Compensated Graduated PLL Discipline
The Air unit acts as a time slave, locking its local microsecond clock to Ground frame arrivals (`LINK_CONTROL` and `RC_CONTROL`).
* **Transit Delay Compensation:** Calculates physical transit latency based on payload length:
  $$\text{expectedArrivalUs} = (\text{plen} == 0) \ ?\ 10200\ :\ (10200 + \text{plen} \times 500)\,\mu\text{s}$$
* **Graduated Phase Correction:** Uses a non-linear phase-locked loop (PLL) with micro-adjustments ($\pm 5\,\mu\text{s}$) and a $250\,\mu\text{s}$ jitter deadband to eliminate clock drift, maintaining **$\pm 4\,\mu\text{s}$ steady-state phase error** on live hardware.

### C. Traffic Prioritization & Segmentation Architecture
* **Phase 2 Single-Burst Framing:** All frames strictly bounded to $\le 64\text{ bytes}$ (13B Header + $\le 49\text{B}$ Payload + 2B CRC-16), completely eliminating modem buffer splitting.
* **Bounded Multi-Fragment Reassembly:** Telemetry and parameter packets up to 512 bytes are segmented by `TransportFragmenter` and reassembled by `TransportReassembler` with gap detection and duplicate rejection.
* **Integrated RC Control Pipeline:** Ingests OpenI6X/CRSF handset streams, intentionally maps handset CH6 to FC CH1, parks FC CH6 at neutral, and outputs CRSF frames to ArduPilot RC_IN on PA11 at 420,000 baud. The 90 ms TDM schedule limits the production RF RC update opportunity to approximately 11.1 Hz; it is not a transparent channel-number bridge.
* **Decoupled Safety Architecture:** Handset RC ingestion, RF dispatch, and failsafe watchdog remain active unconditionally, independent of MAVLink state.

---

## 3. Hardware Bill of Materials (BOM)

| Component | Role | Details | Approx. Cost |
| :--- | :--- | :--- | :---: |
| **STM32F411CEU6 "BlackPill"** | Air Unit MCU | ARM Cortex-M4 @ 100MHz, 512KB Flash, 128KB RAM | $3.50 |
| **ESP32-WROOM-32** | Ground Unit MCU | Xtensa Dual-Core @ 240MHz, USB-UART Bridge onboard | $3.50 |
| **Ebyte E22-900T30D** (×2) | RF Transceivers | UART LoRa modem; power and frequency must be verified per module | $10.00 ($5/ea) |
| **915 MHz Antennas** (×2) | RF Antennas | SMA dipole or tuned whip antenna | $2.00 |
| **Low-ESR Electrolytic Cap** (×2) | Power Decoupling | 470µF to 1000µF, 10V–16V (Absorbs 1.2A RF spikes) | $0.40 |
| **Total Hardware Cost** | | **Complete Air + Ground Telemetry Pair** | **~$19.40** |

---

## 4. Hardware Wiring & Pinout

### Air Unit: STM32F411 BlackPill + Ebyte E22-900T30D + FC

```text
STM32F411CE BlackPill                 Ebyte E22-900T30D
─────────────────────                 ──────────────────
      PA9  (USART1_TX) ──────────────► RXD (Pin 3)
      PA10 (USART1_RX) ◄────────────── TXD (Pin 4)
      PB0  (GPIO)      ──────────────► M0  (Pin 1)
      PB1  (GPIO)      ──────────────► M1  (Pin 2)
      PB10 (GPIO)      ◄────────────── AUX (Pin 5)
      GND              ──────────────  GND (Pin 7)
               5V BEC  ──────────────► VCC (Pin 6)
                                       │
                                    ┌──┴──┐
                                    │470µF│ (Low-ESR Electrolytic)
                                    └──┬──┘
                                       │
                                      GND

Flight Controller (TELEM1 / TELEM2)   STM32F411CE BlackPill
───────────────────────────────────   ─────────────────────
      FC TELEM TX      ──────────────► PA3 (USART2_RX)
      FC TELEM RX      ◄────────────── PA2 (USART2_TX)
      FC GND           ──────────────  GND
      FC 5V            ──────────────► 5V Pin (Powers BlackPill)
```

### Ground Unit: ESP32-WROOM-32 + Ebyte E22-900T30D + Computer USB

```text
ESP32 Dev Module                      Ebyte E22-900T30D
────────────────                      ──────────────────
      GPIO 17 (TX2)    ──────────────► RXD (Pin 3)
      GPIO 16 (RX2)    ◄────────────── TXD (Pin 4)
      GPIO 21 (GPIO)   ──────────────► M0  (Pin 1)
      GPIO 22 (GPIO)   ──────────────► M1  (Pin 2)
      GPIO 19 (GPIO)   ◄────────────── AUX (Pin 5)
      GND              ──────────────  GND (Pin 7)
      5V / VIN         ──────────────► VCC (Pin 6)
                                       │
                                    ┌──┴──┐
                                    │470µF│ (Low-ESR Electrolytic)
                                    └──┬──┘
                                       │
                                      GND

Computer USB Port                     ESP32 Dev Module
─────────────────                     ────────────────
      USB-A / USB-C    ──────────────► Micro-USB / USB-C Port
                                       (Onboard CP2102/CH340 to GCS)
```

> [!CAUTION]
> **1W RF Power Spikes:** The E22-900T30D draws up to **1.2A peak** during RF transmission bursts.
> * **DO NOT** power the radio from the MCU's onboard 3.3V LDO regulator.
> * Connect E22 VCC to a dedicated 5V BEC (Air) or the 5V USB rail (Ground).
> * Solder a **470µF to 1000µF Low-ESR capacitor** directly across E22 VCC and GND.

---

## 5. Building & Flashing (PlatformIO)

### Prerequisites
* Install [PlatformIO Core (CLI)](https://docs.platformio.org/en/latest/core/installation/index.html) or [PlatformIO IDE for VS Code](https://platformio.org/install/ide?install=vscode).
* Clone this repository:
  ```bash
  git clone https://github.com/RiteshKumarRay/Dual-LRS.git
  cd Dual-LRS
  ```

### Build All Firmware Targets
```bash
# Build Air (STM32 BlackPill), Ground (BlackPill), and Ground (ESP32)
pio run
```

### Flash Ground Unit (ESP32-WROOM-32)
Connect the ESP32 to your computer via USB and upload:
```bash
pio run -e dual_lrs_ground_esp32 --target upload
```

### Flash Air Unit (STM32F411 BlackPill via USB DFU)
1. Put the BlackPill into DFU mode: Hold down the **BOOT0** button, press and release the **NRST** (Reset) button, then release **BOOT0**.
   *(If already running Dual-LRS firmware, send character `0x1B` [ESC] over `/dev/ttyACM0` @ 115200 to trigger software DFU reboot automatically).*
2. Upload the firmware via DFU:
   ```bash
   pio run -e dual_lrs_air --target upload
   ```

---

## 6. ArduPilot & GCS Configuration

### ArduPilot Parameters
Connect your flight controller to QGroundControl or Mission Planner and set the following parameters for the serial port connected to the Dual-LRS Air Unit (e.g. `SERIAL1` for `TELEM1`):

| Parameter | Recommended Value | Description |
| :--- | :---: | :--- |
| **SERIAL1_PROTOCOL** | `2` | MAVLink2 |
| **SERIAL1_BAUD** | `115` | 115200 baud |
| **BRD_SER1_RTSCTS** | `0` | Disabled (No hardware handshake required) |
| **SR1_POSITION** | `3` to `5` | GPS location and altitude (3–5 Hz) |
| **SR1_EXTRA1** | `5` to `10` | Attitude horizon (Roll / Pitch / Yaw) (5–10 Hz) |
| **SR1_EXTRA2** | `3` to `5` | VFR HUD (Heading, airspeed, climb rate) (3–5 Hz) |
| **SR1_EXTRA3** | `2` | Battery voltage, current, smart battery (2 Hz) |
| **SR1_RAW_SENS** | `1` | Raw IMU sensors (1 Hz) |
| **SR1_RC_CHAN** | `2` | RC channel inputs (2 Hz) |

### Connecting Ground Station
1. Plug the **Ground Unit** into your computer via USB.
2. In **QGroundControl**:
   * Open **Application Settings -> Comm Links -> Add**.
   * Set Type to `Serial`, Port to your device (`/dev/ttyUSB0` on Linux, `COMx` on Windows), and Baud to `115200`.
   * Click **Connect**.
3. In **Mission Planner**:
   * Select your COM port in the top-right corner.
   * Set baud rate to `115200`.
   * Click **Connect**.

---

## 7. Historical Hardware Observations

The older parameter-download and latency figures below are retained as
operator-reported historical observations only. They are not current acceptance
criteria and do not prove reliable production MAVLink, mission, or parameter
transactions. See the current verification status below before using the
firmware.

All metrics were captured using live hardware: STM32F411 BlackPill (Air), ESP32-WROOM-32 (Ground), Ebyte E22-900T30D (1W LoRa @ 915 MHz), and ArduPilot Mega quadrotor flight controller:

```text
===============================================================
DUAL-LRS: FULL 0-1037 PARAMETER DOWNLOAD HARDWARE VERIFICATION
===============================================================
FC Online: SysID=1, CompID=1, Autopilot=3, Type=2
[UPLINK] Requesting full parameter table (PARAM_REQUEST_LIST)...
[DOWNLINK] Streaming parameters over Dual-LRS LoRa link...
  First parameter arrived in 0.96s!
  [DOWNLINK PROGRESS] 250/1037 (24.1%) in 15.4s | 16.2 params/sec
  [DOWNLINK PROGRESS] 500/1037 (48.2%) in 30.2s | 16.6 params/sec
  [DOWNLINK PROGRESS] 750/1037 (72.3%) in 44.7s | 16.8 params/sec
  [DOWNLINK PROGRESS] 1000/1037 (96.4%) in 59.5s | 16.8 params/sec
  [DOWNLINK PROGRESS] 1037/1037 (100.0%) in 61.5s | 16.9 params/sec

[STREAM COMPLETED] Initial stream downlinked 1037/1037 parameters in 61.5s (16.9 params/sec)
[HOLE RECOVERY] Recovering 1 missing parameters via targeted uplink...
  Retrieved missing #1036: MAV5_OPTIONS = 0.0 (attempt 1 in 0.3s)

Total Parameters Downloaded: 1038/1037 (100.1%)
Total Elapsed Time: 61.5 seconds
Effective Download Speed: 16.9 params/second
===============================================================
```

### Key Performance Numbers
* **Full Parameter Download:** **1037/1037 (100.0%) in 61.5 seconds** at sustained **16.9 parameters/sec**.
* **Flight Mode Switch Latency:** Mode switches confirmed in **650 ms to 950 ms** over RF (down from > 60 seconds).
* **Live `STATUSTEXT` Alerts:** Real-time delivery of ArduPilot pre-arm warnings (`PreArm: Battery 1 below minimum arming voltage`, `PreArm: Radio failsafe on`) to Ground USB.
* **TDM Phase Stability:** Air PLL disciplines to `Arr: 45204 µs, Err: 4 µs, Sync: 1, CRC: 0` with zero clock drift.

### Current Verification Status

The reproducible verification baseline is documented in
[`audit/STAGE_3_2_STATUS.md`](audit/STAGE_3_2_STATUS.md). Current checks include
transport serialization, CRC and bounds checks, fragmentation and reassembly,
CRSF conversion, production RC dispatch, and the complete PlatformIO build
matrix.

The native Stage 3.2 test uses simulated radio delivery. It does not prove
operation with real E22 AUX timing, UART queues, packet loss, or a flight
controller. ACK/NACK generation and retry scheduling are not yet integrated
into the production runtime. Mission upload/download, parameter read/write,
command acknowledgement, reconnection, and saturated-link RC latency therefore
remain unverified.

Keep the default `DUAL_LRS_STAGE31_RC_ONLY=1` configuration. Do not enable
Stage 3.2 for flight use until the reliability and physical bench gates are
completed; perform all bench tests with propellers removed.

---

## 8. Repository Layout

```text
Dual-LRS/
├── platformio.ini           # Multi-target build configurations (Air, Ground STM32, Ground ESP32)
├── README.md                # System documentation and architecture guide
├── .gitignore               # Clean repository filter for production
│
├── include/
│   ├── config.h             # TDM slot timings, buffer sizing, pinout definitions
│   ├── e22_driver.h         # Ebyte E22-900T30D UART driver header
│   ├── tdm_engine.h         # 90 ms TDM scheduler and transport framing
│   └── mavlink_handler.h    # RingBuffer, priority bypass caches, flow control
│
├── src/
│   ├── main.cpp             # Role setup, main execution loop, auto-baud scanner
│   ├── e22_driver.cpp       # Radio hardware configuration, AUX busy monitoring
│   ├── tdm_engine.cpp       # TDM state machine, CRC-16, slot synchronization
│   └── mavlink_handler.cpp  # MAVLink parser, load shedding, priority cache handlers
│
├── docs/
│   ├── TDM_PROTOCOL.md      # Mathematical derivation of TDM slots and PLL slew algorithm
│   ├── PINOUT_AND_WIRING.md # Pin assignments, wiring schematics, power safety guide
│   └── ARDUPILOT_SETUP.md   # FC parameter setup, stream rates, and GCS connection guide
│
└── tools/
    ├── test_transport_protocol.py       # Python transport protocol tests
    ├── test_production_mavlink_transport.cpp # Stage 3.2 host transport tests
    ├── test_production_rc_dispatch.cpp # Production RC path tests
    └── phase1_raw_benchmark.py          # Raw E22 diagnostic benchmark
```

---

## 9. Technical Specifications

| Parameter | Specification |
| :--- | :--- |
| **Air Microcontroller** | STM32F411CEU6 (ARM Cortex-M4 @ 100 MHz, 512KB Flash, 128KB RAM) |
| **Ground Microcontroller** | ESP32-WROOM-32 (Xtensa Dual-Core @ 240 MHz) or STM32F411CEU6 |
| **RF Transceiver** | Ebyte E22-900T30D (Semtech SX1262 LoRa Engine) |
| **Frequency Range** | 850 – 930 MHz (REG2 = 0x17; exact RF frequency pending verification for the exact E22 module variant) |
| **Transmit Power** | Module-dependent; verify the exact E22 configuration before range claims |
| **Air Data Rate** | 62.5 kbps LoRa Modulation |
| **TDM Frame Rate** | Approximately 11.1 Hz (strict 90 ms period) |
| **Ground Slot** | 32 ms |
| **Air Slot** | 45 ms |
| **Guard Intervals** | Gap 1: 5 ms; Gap 2: 8 ms |
| **Clock Synchronization** | Ground master / Air slave with bounded slot synchronization |
| **MAVLink Support** | Development-mode fragmented transport; production reliability pending |
| **GCS Compatibility** | QGroundControl, Mission Planner, MAVProxy, pymavlink |
| **Diagnostic Messages** | Automatic generation of MAVLink `RADIO_STATUS` (ID 109) |

---

## 10. License & Credits

* **License:** [MIT License](LICENSE)
* **Author:** [Ritesh Kumar Ray](https://github.com/RiteshKumarRay)
* **Inspirations & References:**
  * [ArduPilot](https://ardupilot.org/) — The industry-leading open-source autopilot.
  * [mLRS](https://github.com/olliw42/mLRS) by olliw42 — TDM architecture and MAVLink link management principles.
  * [Ebyte](https://www.ebyte.com/) — High-power LoRa hardware.
