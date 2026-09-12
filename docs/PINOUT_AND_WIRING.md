# Dual-LRS Hardware Pinout & Wiring Guide

This guide details the exact electrical connections between the **STM32F411CEU6 BlackPill** microcontroller, the **Ebyte E22-900T30D (1W LoRa)** radio module, and the **Flight Controller (ArduPilot)** or **Ground Control Station (Mission Planner)**.

---

## 1. STM32F411CEU6 BlackPill to Ebyte E22-900T30D

The E22-900T30D has 7 pin header pads (2.54mm pitch):

| E22-900T30D Pin | Function | BlackPill Pin | Description |
| :--- | :--- | :--- | :--- |
| **Pin 1 (M0)** | Operating Mode Control 0 | **PB0** | GPIO Output (Mode selector) |
| **Pin 2 (M1)** | Operating Mode Control 1 | **PB1** | GPIO Output (Mode selector) |
| **Pin 3 (RXD)** | UART Data Input | **PA9** (USART1_TX) | 3.3V Logic. Data from MCU to Radio |
| **Pin 4 (TXD)** | UART Data Output | **PA10** (USART1_RX) | 3.3V Logic. Data from Radio to MCU |
| **Pin 5 (AUX)** | Radio Status / Busy | **PB10** | Input with pull-up. LOW = Busy, HIGH = Idle |
| **Pin 6 (VCC)** | Power Supply (3.3V - 5.5V) | **5V / External BEC** | **CRITICAL: See Power Requirements below!** |
| **Pin 7 (GND)** | Ground | **GND** | Common Ground |

---

## 2. Air Unit: BlackPill to Flight Controller (Prometheus FC / ArduPilot)

The Air Unit connects to the Flight Controller's `TELEM1` or `TELEM2` port:

| BlackPill Pin | Function | Flight Controller Port | Description |
| :--- | :--- | :--- | :--- |
| **PA2** | USART2_TX | **FC TELEM RX** | Telemetry uplink / commands to ArduPilot |
| **PA3** | USART2_RX | **FC TELEM TX** | Telemetry downlink from ArduPilot |
| **GND** | Ground | **FC GND** | Common ground reference |
| **5V** | 5V Input | **FC 5V** (or BEC) | Logic power for BlackPill |

---

## 3. Ground Unit: BlackPill to Laptop (Mission Planner)

The Ground Unit can connect to your laptop in two ways:
1. **USB-C Cable directly to Laptop** (Recommended):
   - The BlackPill's onboard USB-C port runs in **USB CDC (Virtual COM Port)** mode.
   - Simply plug a USB-C cable from the BlackPill into your laptop.
   - Windows will detect it as `COMx (STM32 Virtual COM Port)`.
   - In Mission Planner, select this COM port and `115200` baud.
2. **External USB-UART FTDI / CP2102** (Optional):
   - Connect **PA2** (USART2_TX) to FTDI RX and **PA3** (USART2_RX) to FTDI TX.

---

## 4. ⚠️ Critical Power Supply Notice (1W RF Output)

> [!CAUTION]
> The **E22-900T30D transmits at up to 1000 mW (30 dBm)**. During transmission bursts, current draw can peak between **600 mA to 1200 mA (1.2A)**.
> - **DO NOT** power the E22 VCC from the BlackPill's onboard 3.3V LDO regulator! The onboard LDO can only supply ~150-200mA and will overheat or reset the MCU.
> - **DO** supply 5V directly from a dedicated 5V BEC (Air unit) or directly from the 5V USB rail / external 5V regulator (Ground unit).
> - **Add a 470µF to 1000µF Low-ESR electrolytic capacitor** across the E22 `VCC` and `GND` pins right next to the module to absorb RF transmission current spikes.
