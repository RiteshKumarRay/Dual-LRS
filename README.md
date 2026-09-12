# Dual-LRS

**Dual-LRS** is an open-source, bidirectional long-range telemetry and control link firmware designed specifically for the **STM32F411CEU6 ("BlackPill")** and the **Ebyte E22-900T30D** (1-Watt Semtech SX1262 LoRa module).

It provides a unified, low-latency, half-duplex **TDM (Time Division Multiplexing)** data bridge between **ArduPilot** flight controllers (such as Prometheus FC) and Ground Control Stations like **Mission Planner**.

---

## Key Features

- **Bidirectional MAVLink Bridge**: Stream high-rate telemetry downlink (attitude, GPS, battery, status) and uplink commands/parameters simultaneously over a single RF link.
- **Deterministic 20 Hz TDM Engine**: Fixed 50 ms time frames partitioned into asymmetric Air (60%), Ground (24%), and turnaround guard slots (16%) to prevent packet collisions on half-duplex LoRa.
- **1-Watt (30 dBm) Power Output**: Utilizes the full 30 dBm capability of the Ebyte E22-900T30D on 868 MHz / 915 MHz bands.
- **Native MAVLink Link Quality (`RADIO_STATUS`)**: Injects standard Message ID 109 packets directly into the telemetry stream so Mission Planner natively displays RSSI and packet loss in real time.
- **USB CDC Native Port**: Ground Unit connects directly via USB-C to your laptop without requiring an external USB-UART FTDI adapter.
- **Hardware AUX Pin Handshake**: Monitors the E22 hardware `AUX` pin to eliminate buffer overruns and detect active RF transmission windows.
- **Data Integrity**: Full CRC-16-CCITT packet validation and sequence counter loss tracking.

---

## System Architecture

```mermaid
flowchart LR
    subgraph Ground ["Ground Station"]
        MP["Mission Planner (Laptop)"] <-->|USB-C (Virtual COM)| BP_GND["BlackPill (F411)"]
        BP_GND <-->|UART1 + AUX/M0/M1| E22_GND["E22-900T30D (1W)"]
    end

    E22_GND <==>|915MHz LoRa TDM Link (20 Hz)| E22_AIR["E22-900T30D (1W)"]

    subgraph Air ["Drone (Air Unit)"]
        E22_AIR <-->|UART1 + AUX/M0/M1| BP_AIR["BlackPill (F411)"]
        BP_AIR <-->|USART2 (TELEM)| FC["Prometheus FC (ArduPilot)"]
    end
```

---

## Hardware Pinout (STM32F411CE BlackPill)

| BlackPill Pin | Function | Connected To |
| :--- | :--- | :--- |
| **PA9** | USART1_TX | E22-900T30D **RXD** |
| **PA10** | USART1_RX | E22-900T30D **TXD** |
| **PB0** | GPIO Output | E22-900T30D **M0** (Mode control) |
| **PB1** | GPIO Output | E22-900T30D **M1** (Mode control) |
| **PB10** | GPIO Input (Pull-up) | E22-900T30D **AUX** (Busy indicator) |
| **PA2** | USART2_TX | Flight Controller **TELEM RX** (Air Unit) |
| **PA3** | USART2_RX | Flight Controller **TELEM TX** (Air Unit) |
| **PC13** | Onboard LED | TDM Synchronization & Link Quality Indicator |
| **USB-C** | Hardware USB FS | Laptop / Mission Planner (Ground Unit) |

> [!CAUTION]
> **Power Notice**: The E22-900T30D draws up to **1.2A peak** during 1W transmission bursts. **Do not power the E22 from the BlackPill 3.3V pin.** Power the E22 directly from a dedicated 5V BEC and solder a 470µF–1000µF capacitor across `VCC` and `GND`.

---

## Building and Flashing

This project is configured with **PlatformIO**.

### 1. Build Air Unit Firmware
```bash
pio run -e dual_lrs_air
```

### 2. Build Ground Unit Firmware
```bash
pio run -e dual_lrs_ground
```

### 3. Flash via DFU (USB-C)
1. Hold the **BOOT0** button on the BlackPill, press **NRST** (Reset), then release **BOOT0**.
2. Flash using PlatformIO:
```bash
pio run -e dual_lrs_air -t upload
```
*(Or flash using ST-Link by setting `upload_protocol = stlink` in `platformio.ini`)*

---

## Documentation

Detailed technical documentation is available in the [`docs/`](docs/) directory:
- [Pinout & Wiring Guide](docs/PINOUT_AND_WIRING.md)
- [TDM Protocol & Timing Specification](docs/TDM_PROTOCOL.md)
- [ArduPilot & Mission Planner Configuration](docs/ARDUPILOT_SETUP.md)

---

## License
Open-source under the MIT License.
