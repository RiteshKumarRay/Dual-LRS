# Dual-LRS TDM (Time Division Multiplexing) Protocol Specification

Because LoRa transceivers (Semtech SX1262 / Ebyte E22-900T30D) operate on a single RF carrier frequency, physical transmission is **half-duplex** (the module can only transmit or receive at any given instant).

To achieve seamless, bidirectional, low-latency MAVLink telemetry between the Ground Control Station (QGroundControl / Mission Planner) and the drone's Flight Controller (ArduPilot), Dual-LRS implements a deterministic **Time Division Multiplexing (TDM)** frame scheduler disciplined by a graduated Phase-Locked Loop (PLL).

---

## 1. Calibrated TDM Frame Schedule (50 ms Cycle / 20 Hz)

Each 50 ms cycle is partitioned into asymmetric slots optimized for drone telemetry:

```text
+----------------------------------------+---------+--------------------+---------+
|          Slot 1: Air Transmit          |  Gap 1  |   Slot 2: Ground   |  Gap 2  |
|          (Downlink Telemetry)          |  Guard  |   (Uplink / Cmds)  |  Guard  |
|                 31 ms                  |  4 ms   |       12 ms        |  3 ms   |
|               (0 - 31 ms)              | (31-35) |     (35 - 47 ms)   | (47-50) |
+----------------------------------------+---------+--------------------+---------+
|<------------------------------ 50 ms Total Frame ------------------------------>|
```

### Slot Allocations & Physical Margin Derivation

1. **Slot 1 (Air Transmit — 31 ms / 62% bandwidth):**
   * **Payload Cap:** Max 40 bytes per slot (`MAX_PAYLOAD_AIR_SLOT = 40`).
   * **Framed Size:** 5B header + 40B payload + 2B CRC = **47 bytes** over UART.
   * **Measured Hardware Airtime:** At 115200 baud UART + 62.5 kbps LoRa air rate, the E22 AUX busy duration measures **30.002 ms** (`AuxUs: 30002`).
   * **Air End Time:** Exactly $30.002\text{ ms}$, leaving $0.998\text{ ms}$ before the 31.0 ms slot boundary.

2. **Guard Gap 1 (4 ms / 31.0 ms – 35.0 ms):**
   * Provides a dedicated silence window between the end of Air transmission and the start of Ground transmission.
   * Total clearance margin before Ground begins transmitting:
     $$35.000\text{ ms} - 30.002\text{ ms} = \mathbf{4.998\text{ ms}}$$
   * Allows Ground's E22 receiver to clock out trailing bytes to the MCU and ensures the E22 AUX line returns HIGH (Idle) before Ground transmits.

3. **Slot 2 (Ground Transmit — 12 ms / 35.0 ms – 47.0 ms):**
   * **Payload Cap:** Max 4 bytes per slot (`MAX_PAYLOAD_GROUND_SLOT = 4`).
   * **Framed Size:** 5B header + 4B payload + 2B CRC = **11 bytes** over UART.
   * **Total Ground TX Duration:** 11 bytes @ 115200 baud UART ($0.95\text{ ms}$) + LoRa RF airtime ($7.5\text{ ms}$) = **8.5 ms**.
   * **Ready Budget:** Ground monitors AUX with `waitForReady(5)`. If AUX clears at $35.5 - 36.5\text{ ms}$, transmission begins immediately and concludes by $44.0 - 45.0\text{ ms}$. If delayed, transmission starts at $40.0\text{ ms}$ latest and completes by **48.5 ms**.

4. **Guard Gap 2 (3 ms / 47.0 ms – 50.0 ms):**
   * Leaves at least **1.5 ms to 2.5 ms of clean silence** before the 50.0 ms frame wraps back to 0.0 ms (Air slot start).
   * Guarantees Ground's uplink transmission never collides with Air's downlink slot.

---

## 2. Linear Transit-Compensated PLL Discipline

The Dual-LRS link uses a master-slave timing hierarchy:
* **Ground Node:** Time Master. Runs a free-running hardware timer advancing unconditionally in 50.0 ms periods.
* **Air Node:** Time Slave. Disciplines its local `_frameStartTimeUs` timer to Ground transmissions.

### Transit Delay Formula

Ground transmits both `HEARTBEAT_SYNC` (0-byte payload) and `MAVLINK_DATA` (1- to 4-byte payload). Because LoRa airtime and UART clocking increase linearly with packet length, Air applies length-compensated arrival calculation:

$$\text{transitDelayUs} = 10200 + (\text{payload\_len} \times 215)\text{ µs}$$

$$\text{expectedArrivalUs} = ((TDM\_AIR\_SLOT\_MS + TDM\_GUARD\_GAP1\_MS) \times 1000) + \text{transitDelayUs}$$

* **0-byte Beacon:** $10,200\text{ µs}$ transit $\rightarrow$ Expected arrival at $45,200\text{ µs}$.
* **4-byte Payload:** $11,060\text{ µs}$ transit $\rightarrow$ Expected arrival at $46,060\text{ µs}$.

### Graduated Slew Control

Air calculates phase error: $\text{err} = \text{targetStartUs} - \text{_frameStartTimeUs}$, wrapped to $[-25000, +25000\text{ µs}]$:

| Phase Error ($|\text{err}|$) | Correction Mode | Step Size | Behavior |
| :--- | :--- | :--- | :--- |
| $> 3000\text{ µs}$ | Fast Snap | $\text{_frameStartTimeUs} += \text{err}$ | Instant 1-step lock on boot/reconnection |
| $1000\text{ µs} - 3000\text{ µs}$ | Fast Slew | $\pm 100\text{ µs}$ per frame | Rapid recovery without packet loss |
| $500\text{ µs} - 1000\text{ µs}$ | Medium Slew | $\pm 25\text{ µs}$ per frame | Smooth convergence |
| $250\text{ µs} - 500\text{ µs}$ | Micro Slew | $\pm 5\text{ µs}$ per frame | Crystal drift compensation |
| $< 250\text{ µs}$ | Deadband | $0\text{ µs}$ (no adjustment) | Rejects UART & MCU execution jitter |

On hardware, Air achieves steady-state phase error of **$\pm 4\text{ µs}$**.

---

## 3. Traffic Prioritization & Congestion Control

Because bandwidth is asymmetric ($760\text{ B/s}$ Downlink vs $80\text{ B/s}$ Uplink), Dual-LRS incorporates multi-level traffic management:

```text
GROUND UPLINK INGESTION:
[QGC USB] ──► readFromLocal()
                  ├── msgid == 111 (TIMESYNC) ──► DROPPED OUTRIGHT (Saves ~280 B/s)
                  ├── msgid == 0   (HEARTBEAT) ─► _hbCache (4B/slot Priority Bypass)
                  └── Commands / Params / Modes ─► _txQueue (1 KB FIFO)

AIR DOWNLINK INGESTION:
[FC UART] ──► readFromLocal()
                  ├── msgid == 0   (HEARTBEAT) ──► _hbCache (High Priority Bypass)
                  ├── msgid == 253 (STATUSTEXT) ─► _stCache (Mode Change Bypass)
                  ├── msgid == 22  (PARAM_VALUE) ─► _txQueue (48 KB FIFO Burst Buffer)
                  └── Routine IMU / HUD ────────► Rate-limited during param bursts
```

1. **QGC `TIMESYNC` Shedding:** QGC generates ~280 B/s of `TIMESYNC` (msgid 111). Dropping it on Ground eliminates queue bloat.
2. **Fragment-Aware GCS Heartbeat Bypass:** GCS Heartbeats bypass `_txQueue` into `_hbCache`, draining 4 bytes per slot over ~5 slots without clobbering in-flight commands.
3. **Air `STATUSTEXT` Priority Bypass:** Flight mode changes and pre-arm warnings jump to the front of Air's queue, downlinking in $< 1\text{ second}$.
4. **Role-Aware Buffers:** 48 KB on Air (`TELEM_BUFFER_SIZE = 49152`) absorbs the full 1037 parameter table (~38 KB) without throttling ArduPilot; 1 KB on Ground bounds worst-case uplink delay.

---

## 4. Over-the-Air Binary Frame Format

All RF packets sent over the E22 LoRa transceiver are framed with binary headers and CRC-16-CCITT integrity verification:

```text
Byte Index  Field          Type     Description
────────────────────────────────────────────────────────────────────────
[0]         magic0         uint8_t  DUAL_LRS_MAGIC_0 (0x44 = 'D')
[1]         magic1         uint8_t  DUAL_LRS_MAGIC_1 (0x4C = 'L')
[2]         packet_type    uint8_t  0x01 = HEARTBEAT_SYNC (Beacon)
                                    0x02 = MAVLINK_DATA (Telemetry / Commands)
                                    0x03 = RC_OVERRIDE (Reserved)
                                    0x04 = RADIO_STATUS (Reserved)
[3]         seq_num        uint8_t  Rolling 8-bit packet sequence counter (0 - 255)
[4]         payload_len    uint8_t  Length of payload (0 - 40 bytes)
[5..4+len]  payload        uint8_t* Raw MAVLink packet bytes
[5+len]     crc_low        uint8_t  CRC-16-CCITT low byte
[6+len]     crc_high       uint8_t  CRC-16-CCITT high byte
```

### CRC-16 Calculation
* **Polynomial:** `0x1021` ($X^{16} + X^{12} + X^5 + 1$)
* **Initial Value:** `0xFFFF`
* **Coverage:** Calculated over `LrsFrameHeader` (5 bytes) followed by all payload bytes.

### Packet Sequence & Link Quality Tracking
* Receiver checks each arriving packet sequence against `last_rx_seq`.
* Any gap increments `packets_dropped` and `seq_drops`.
* Link Quality percentage is calculated continuously:
  $$\text{Link Quality (\%)} = \frac{\text{packets\_received}}{\text{packets\_received} + \text{packets\_dropped}} \times 100$$
* Mapped into standard MAVLink `RADIO_STATUS` (Message ID 109) and emitted to QGC/Mission Planner at 1 Hz.
