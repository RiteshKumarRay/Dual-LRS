# ArduPilot & Mission Planner Setup Guide

Follow these configuration steps to connect your **Dual-LRS** telemetry system to your **Prometheus FC** (or any ArduPilot flight controller) and **Mission Planner**.

---

## 1. ArduPilot Parameter Configuration

Connect your flight controller to Mission Planner via USB, navigate to **Config -> Full Parameter List**, and verify or update the following parameters for the serial port connected to the Dual-LRS Air Unit (e.g., `TELEM1` which is typically `SERIAL1`):

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **SERIAL1_PROTOCOL** | `2` | MAVLink2 (Enables full MAVLink v2 support) |
| **SERIAL1_BAUD** | `115` | 115200 baud (Matches Dual-LRS FC_UART_BAUD) |
| **BRD_SER1_RTSCTS** | `0` | Disabled (Hardware flow control not required) |

### Recommended Telemetry Stream Rates (Hz)
To optimize telemetry bandwidth over the 900MHz LoRa link, set the stream rates in ArduPilot:

| Parameter | Recommended Value | Description |
| :--- | :--- | :--- |
| **SR1_POSITION** | `3` to `5` | GPS and altitude (3–5 Hz) |
| **SR1_EXTRA1** | `5` to `10` | Attitude / Roll / Pitch (5–10 Hz) |
| **SR1_EXTRA2** | `3` to `5` | VFR HUD / Airspeed / Heading (3–5 Hz) |
| **SR1_EXTRA3** | `2` | Battery voltage, current, sensors (2 Hz) |
| **SR1_RAW_SENS** | `1` | IMU raw sensors (1 Hz) |
| **SR1_RC_CHAN** | `2` | RC channel inputs (2 Hz) |

---

## 2. Mission Planner Connection

1. Connect the **Dual-LRS Ground Unit** to your laptop using a USB-C cable.
2. In Windows Device Manager, verify a new COM port appears (e.g. `COM5 - STMicroelectronics Virtual COM Port`).
3. In Mission Planner (top-right corner):
   - Select the corresponding `COMx` port.
   - Select baud rate: `115200`.
   - Click **Connect**.
4. Mission Planner will download parameters from the drone and display live telemetry on the flight HUD, including RSSI link quality from the injected `RADIO_STATUS` packets.
