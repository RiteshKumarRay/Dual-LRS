# Dual-LRS — Open Source Bidirectional Long Range System

> **DIY bidirectional MAVLink telemetry + RC link for ArduPilot drones, built on the STM32F411 BlackPill and Ebyte E22-900T30D (1W LoRa 900MHz).**

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Hardware](https://img.shields.io/badge/MCU-STM32F411CE-blue)]()
[![Radio](https://img.shields.io/badge/Radio-SX1262%20LoRa%201W-orange)]()

---

## What Is Dual-LRS?

Dual-LRS is an open-source firmware that creates a **unified, long-range, bidirectional data link** between your drone's flight controller and your ground control station (Mission Planner, QGroundControl). It replaces the traditional pair of separate radios (one for RC control, one for telemetry) with a **single LoRa link** that carries both.

### The Problem It Solves

A typical drone telemetry setup requires **two independent radio systems**:

| Traditional Setup | Radio 1 | Radio 2 |
| :--- | :--- | :--- |
| **Purpose** | RC stick inputs (pilot control) | Telemetry (flight data + GCS commands) |
| **Examples** | ExpressLRS, Crossfire, FrSky | SiK 3DR Radio, RFD900 |
| **Direction** | Ground → Air (mostly one-way) | Bidirectional |
| **Cost** | $20–$80 | $30–$150 |
| **Weight** | 2–10g per module | 15–50g per module |

**Dual-LRS merges both functions into one link**, reducing weight, cost, complexity, and the number of antennas on your airframe.

---

## How Does It Work?

### Half-Duplex LoRa + Time Division Multiplexing (TDM)

The Ebyte E22-900T30D uses a Semtech SX1262 LoRa transceiver. LoRa is inherently **half-duplex** — the radio can only transmit OR receive at any given moment, never both simultaneously (unlike WiFi or 4G).

Dual-LRS solves this with **Time Division Multiplexing (TDM)**: it slices time into fixed-duration slots and alternates between Air→Ground and Ground→Air transmissions, similar to how SiK radios and cellular TDMA systems work.

```
                          ← 50 ms TDM Frame (20 Hz) →
┌─────────────────────────────┬──────┬──────────────────┬──────┐
│     Air → Ground (30 ms)    │ Gap  │ Ground → Air     │ Gap  │
│   Telemetry Downlink (60%)  │ 4 ms │ Commands (12 ms) │ 4 ms │
│ ATTITUDE, GPS, BATTERY, HUD │      │ Params, WP, RC   │      │
└─────────────────────────────┴──────┴──────────────────┴──────┘
         DRONE TRANSMITS              LAPTOP TRANSMITS
```

**Why asymmetric slots?** ArduPilot generates far more downlink telemetry (attitude at 10Hz, GPS at 5Hz, battery at 2Hz, heartbeats at 1Hz) than Mission Planner sends uplink commands (occasional parameter requests, waypoint uploads, mode changes). The 60/24 split maximizes throughput where it matters most.

### Synchronization

The **Ground node** is the clock master — its timer runs freely. The **Air node** is the clock slave — it locks its frame timing to the arrival timestamp of each Ground packet. If the Air node doesn't hear from Ground for 1.5 seconds, it enters an unsynchronized listening mode and automatically re-locks when Ground transmits again.

---

## How Is It Different From Existing Solutions?

| Feature | **Dual-LRS** | **SiK 3DR Radio** | **mLRS** | **ExpressLRS** |
| :--- | :---: | :---: | :---: | :---: |
| **Modulation** | LoRa (SX1262) | FSK (Si1000) | LoRa (SX1262/SX1280) | LoRa (SX1280/SX1262) |
| **Max TX Power** | 1W (30 dBm) | 100mW (20 dBm) | 1W (30 dBm) | 1W (30 dBm) |
| **Frequency** | 868/915 MHz | 433/915 MHz | 433/868/915/2400 MHz | 868/915/2400 MHz |
| **Bidirectional MAVLink** | ✅ Native | ✅ Native | ✅ Native | ⚠️ Limited (Airport mode) |
| **RC Channels** | ✅ Planned | ❌ No | ✅ 16 channels | ✅ 16 channels |
| **Hardware Cost** | ~$8 total | ~$30–50 | ~$25–60 | ~$15–40 |
| **MCU** | STM32F411 (100MHz Cortex-M4) | C8051F930 (25MHz 8-bit) | STM32G431/WLE5 | ESP32/STM32 |
| **DIY Friendly** | ✅ BlackPill + breakout module | ❌ Integrated PCB | ⚠️ Specific boards needed | ⚠️ Specific boards needed |
| **Interface to Radio** | UART (simple wiring) | UART | SPI (direct register access) | SPI |
| **Open Source** | ✅ MIT | ✅ BSD | ✅ GPL-3.0 | ✅ GPL-2.0 |
| **ArduPilot Optimized** | ✅ | ✅ | ✅ | ⚠️ Primary focus is RC |

### Key Differentiators

1. **Cheapest possible BOM**: A BlackPill F411 costs ~$3, an E22-900T30D costs ~$5. Total hardware cost under $8 per node, $16 for a complete Air+Ground pair. No custom PCB required for prototyping.

2. **UART-based radio interface**: Unlike mLRS which requires direct SPI access to the SX1262 chip (needing the bare `E22-900M30S` module), Dual-LRS works with the common UART-based `E22-900T30D` module that most hobbyists already own. Just 5 wires: TX, RX, AUX, M0, M1.

3. **Designed for ArduPilot from day one**: Native MAVLink v2 passthrough with intelligent `RADIO_STATUS` (Message ID 109) injection, so Mission Planner automatically displays RSSI, link quality, and packet loss without any plugins or configuration.

4. **Genuine gap in the ecosystem**: Nobody has published open-source MAVLink telemetry firmware for the STM32F411 + Ebyte E22 UART module combination. This is a first.

---

## Hardware

### Bill of Materials (per node — you need 2 identical sets)

| Component | Quantity | Approx. Cost |
| :--- | :---: | :---: |
| STM32F411CEU6 "BlackPill" development board | 1 | $3 |
| Ebyte E22-900T30D (SX1262, 30dBm, 868/915 MHz) | 1 | $5 |
| 915 MHz SMA Antenna (or spring antenna) | 1 | $1 |
| 470µF–1000µF electrolytic capacitor (for TX current spikes) | 1 | $0.20 |
| Dupont jumper wires / hookup wire | 5 | — |
| **Total per node** | | **~$8** |
| **Total for Air + Ground pair** | | **~$16** |

### Wiring Diagram

```
  STM32F411CE BlackPill              Ebyte E22-900T30D
  ─────────────────────              ──────────────────
        PA9  (TX) ──────────────────► RXD  (Pin 3)
        PA10 (RX) ◄──────────────────  TXD  (Pin 4)
        PB0  (GPIO) ────────────────► M0   (Pin 1)
        PB1  (GPIO) ────────────────► M1   (Pin 2)
        PB10 (GPIO) ◄────────────────  AUX  (Pin 5)
        GND  ────────────────────────  GND  (Pin 7)
                    5V BEC ──────────► VCC  (Pin 6)
                                       │
                                    ┌──┴──┐
                                    │470µF│  (Low-ESR cap)
                                    └──┬──┘
                                       │
                                      GND
```

> ⚠️ **CRITICAL**: The E22-900T30D draws up to **1.2A peak** during 1W transmission bursts. **Do NOT power it from the BlackPill's 3.3V LDO** (max ~200mA). Use a dedicated 5V BEC on the drone (Air unit) or the 5V USB rail (Ground unit). Always add a bulk capacitor right at the E22 VCC/GND pins.

### Air Unit Connection to Flight Controller

```
  BlackPill (Air)                   Prometheus FC / ArduPilot
  ─────────────                     ─────────────────────────
        PA2  (TX) ──────────────────► TELEM1 RX
        PA3  (RX) ◄──────────────────  TELEM1 TX
        GND  ────────────────────────  GND
```

### Ground Unit Connection to Laptop

Simply plug the BlackPill's USB-C port into your laptop. It enumerates as a USB CDC Virtual COM Port — no FTDI adapter needed.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        GROUND STATION                                │
│                                                                      │
│  Mission Planner ◄──USB CDC──► [BlackPill F411] ◄──UART──► [E22 1W] │
│                                  Ground Firmware              ┌─┐    │
│                                                               │A│    │
│                                                               │N│    │
│                                                               │T│    │
└──────────────────────────────────────────────────────────────┤ │────┘
                                                                └─┘
                          915 MHz LoRa RF Link (TDM, 20 Hz)
                              ↕↕↕↕↕↕↕↕↕↕↕↕↕↕↕
┌──────────────────────────────────────────────────────────────┤ │────┐
│                                                               ┌─┐    │
│                                                               │A│    │
│                                                               │N│    │
│                                                               │T│    │
│  ArduPilot FC ◄──TELEM UART──► [BlackPill F411] ◄──UART──► [E22 1W] │
│  (Prometheus)                    Air Firmware                  └─┘    │
│                                                                      │
│                           DRONE (AIR UNIT)                           │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Building & Flashing

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- USB-C cable
- (Optional) ST-Link V2 programmer

### Build

```bash
# Build Air Unit firmware (for the drone)
pio run -e dual_lrs_air

# Build Ground Unit firmware (for your laptop / GCS)
pio run -e dual_lrs_ground
```

### Flash via DFU (USB-C, no programmer needed)

1. On the BlackPill, hold **BOOT0**, press **NRST** (Reset), release **BOOT0**.
2. The board enters DFU bootloader mode.
3. Flash:

```bash
# Flash Air Unit
pio run -e dual_lrs_air -t upload

# Flash Ground Unit
pio run -e dual_lrs_ground -t upload
```

### Flash via ST-Link

Change `upload_protocol` in `platformio.ini`:

```ini
upload_protocol = stlink
```

Then run `pio run -e dual_lrs_air -t upload`.

---

## ArduPilot Configuration

Connect to your FC via USB and set these parameters in Mission Planner for the TELEM port connected to Dual-LRS:

```
SERIAL1_PROTOCOL = 2        # MAVLink2
SERIAL1_BAUD     = 115      # 115200 baud

# Recommended telemetry stream rates (Hz)
SR1_POSITION     = 3        # GPS position
SR1_EXTRA1       = 5        # Attitude (Roll/Pitch/Yaw)
SR1_EXTRA2       = 3        # VFR HUD
SR1_EXTRA3       = 2        # Battery, sensors
SR1_RAW_SENS     = 1        # Raw IMU
SR1_RC_CHAN       = 2        # RC channel values
```

### Mission Planner Connection

1. Plug Ground Unit USB-C into laptop
2. In Mission Planner: select `COMx` → baud `115200` → **Connect**
3. Telemetry streams immediately — RSSI and link quality appear automatically in the HUD from the injected `RADIO_STATUS` packets

---

## Over-the-Air Protocol

### Packet Format

```
Byte 0:    0x44 ('D')        — Magic byte 0
Byte 1:    0x4C ('L')        — Magic byte 1
Byte 2:    Packet Type        — 0x01=Sync, 0x02=MAVLink, 0x03=RC, 0x04=RadioStatus
Byte 3:    Sequence Number    — Rolling 0–255 counter for loss detection
Byte 4:    Payload Length     — 0–240 bytes
Byte 5..N: Payload Data       — Raw MAVLink bytes or RC channel data
Byte N+1:  CRC-16 Low         — CRC-16-CCITT over header + payload
Byte N+2:  CRC-16 High
```

### Link Quality Monitoring

Every received packet is checked against the expected sequence number. Gaps increment a `packets_dropped` counter. Link quality is computed as:

```
Link Quality (%) = packets_received / (packets_received + packets_dropped) × 100
```

This is mapped to the MAVLink `RADIO_STATUS` message (ID 109) and injected into the local serial stream once per second, so Mission Planner shows real-time signal quality bars natively — no plugins required.

---

## Project Structure

```
Dual-LRS/
├── platformio.ini           # Build targets: dual_lrs_air, dual_lrs_ground
├── README.md                # This file
├── .gitignore
│
├── include/
│   ├── config.h             # Pin definitions, baud rates, TDM timing constants
│   ├── e22_driver.h         # E22-900T30D UART driver API
│   ├── tdm_engine.h         # TDM state machine, packet types, link stats
│   └── mavlink_handler.h    # Ring buffer, MAVLink passthrough, RADIO_STATUS
│
├── src/
│   ├── main.cpp             # Setup, main loop, role-based initialization
│   ├── e22_driver.cpp       # E22 mode control (M0/M1), AUX busy monitoring
│   ├── tdm_engine.cpp       # 20Hz TDM scheduler, CRC-16, frame sync
│   └── mavlink_handler.cpp  # High-throughput ring buffer, MAVLink injection
│
└── docs/
    ├── PINOUT_AND_WIRING.md  # Detailed wiring tables with power warnings
    ├── TDM_PROTOCOL.md       # Timing breakdown, packet format spec
    └── ARDUPILOT_SETUP.md    # ArduPilot parameters and Mission Planner setup
```

---

## Firmware Memory Usage

```
Environment      RAM                     Flash
─────────────    ─────────────────────   ─────────────────────────
dual_lrs_air     4.5% (5,916 / 131,072)  5.2% (27,492 / 524,288)
dual_lrs_ground  4.5% (5,916 / 131,072)  5.2% (27,492 / 524,288)
```

Only **5% of Flash and 4.5% of RAM** used — massive headroom for future features (RC channels, FHSS, OTA configuration, OLED display, etc.).

---

## Roadmap

### Current (v0.1 — Foundation)
- [x] Half-duplex TDM engine with deterministic 20 Hz framing
- [x] Bidirectional MAVLink transparent passthrough
- [x] E22-900T30D UART driver with hardware AUX handshake
- [x] CRC-16-CCITT packet validation
- [x] Sequence-based link quality tracking
- [x] Native MAVLink RADIO_STATUS injection for Mission Planner
- [x] USB CDC ground station interface (no FTDI needed)
- [x] Dual build targets (Air / Ground)

### Planned (v0.2 — RC Control)
- [ ] RC channel packing (8ch × 11-bit + 4ch × 8-bit in Ground→Air slot)
- [ ] SBUS output on Air unit for direct FC connection
- [ ] CRSF protocol support
- [ ] Failsafe detection and configurable behavior

### Planned (v0.3 — Reliability)
- [ ] Frequency Hopping Spread Spectrum (FHSS) across 915 MHz ISM band
- [ ] Adaptive data rate (auto SF7↔SF9 based on link quality)
- [ ] AES-128 encryption with bind phrase
- [ ] Selective ARQ (retransmit critical packets only)

### Future (v1.0 — Product)
- [ ] Custom PCB design (integrated BlackPill + E22 + antenna connector)
- [ ] 3D-printed enclosure for JR module bay (TX side)
- [ ] Compact receiver form factor for airframe mounting
- [ ] Web-based configuration UI (via USB or WiFi ESP32 bridge)
- [ ] OLED status display (RSSI, mode, channel, power)

---

## Technical Specifications

| Parameter | Value |
| :--- | :--- |
| MCU | STM32F411CEU6 (ARM Cortex-M4, 100 MHz, 512KB Flash, 128KB RAM) |
| Radio Module | Ebyte E22-900T30D (Semtech SX1262) |
| Frequency | 850–930 MHz (configurable, ISM 868/915 MHz) |
| Max TX Power | 30 dBm (1 Watt) |
| Modulation | LoRa spread spectrum |
| Interface | UART (115200 baud) between MCU and radio |
| TDM Frame Rate | 20 Hz (50 ms per frame) |
| Downlink Bandwidth | ~60% (telemetry from drone) |
| Uplink Bandwidth | ~24% (commands from GCS) |
| Packet Integrity | CRC-16-CCITT with sequence tracking |
| GCS Interface | USB CDC Virtual COM Port (Ground) / UART (Air) |
| Supported Protocols | MAVLink v1/v2 (transparent passthrough) |
| Power Consumption | ~50mA idle, ~600–1200mA peak TX (radio module) |
| Weight (Air unit) | ~15g (BlackPill + E22 + wires, no case) |

---

## Contributing

This is an early-stage project. Contributions, bug reports, and ideas are welcome!

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m "feat: add my feature"`
4. Push and open a Pull Request

---

## License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

---

## Acknowledgments

- **[mLRS](https://github.com/olliw42/mLRS)** by olliw42 — Architectural inspiration for TDM timing, MAVLink prioritization, and link quality concepts. mLRS is an excellent project targeting SPI-based radio modules; Dual-LRS fills the gap for UART-based Ebyte modules.
- **[ArduPilot](https://ardupilot.org/)** — The flight controller firmware that Dual-LRS is designed to work with.
- **[Ebyte](https://www.ebyte.com/)** — For making affordable, high-power LoRa modules accessible to hobbyists.

---

*Built with ❤️ for the open-source drone community by [Ritesh Kumar Ray](https://github.com/RiteshKumarRay)*
