# Dual-LRS TDM (Time Division Multiplexing) Protocol Specification

Because LoRa transceivers (Semtech SX1262 / Ebyte E22-900T30D) operate on a single RF carrier frequency, physical transmission is **half-duplex** (the module can only transmit or receive at any given instant).

To achieve simultaneous, deterministic, low-latency control and bidirectional telemetry between the Ground Control Station (QGroundControl / Mission Planner), the RC Handset, and the drone's Flight Controller (ArduPilot), Dual-LRS implements a deterministic **Time Division Multiplexing (TDM)** frame scheduler disciplined by a graduated Phase-Locked Loop (PLL).

---

## 1. Phase 2 TDM Frame Schedule (90.0 ms Cycle / ~11.1 Hz)

Each 90.0 ms cycle is partitioned into asymmetric slots optimized for prioritized RC uplink and high-throughput telemetry downlink:

```text
+-----------------------+---------+-----------------------------------+---------+
|     Slot 1: Ground    |  Gap 1  |        Slot 2: Air Transmit       |  Gap 2  |
|     (RC / Commands)   |  Guard  |        (Downlink Telemetry)       |  Guard  |
|          32 ms        |   5 ms  |               45 ms               |   8 ms  |
|       (0 - 32 ms)     | (32-37) |            (37 - 82 ms)           | (82-90) |
+-----------------------+---------+-----------------------------------+---------+
|<------------------------------ 90 ms Total Frame ---------------------------->|
```

### Slot Allocations & Physical Timing Margins

1. **Slot 1 (Ground Transmit — 32 ms / 0.0 ms – 32.0 ms):**
   * **Primary Function:** Unconditional RC Control uplink frame (`RC_CONTROL`, 24B payload, 39B wire frame) transmitted when handset signal is active.
   * **Secondary Function:** Interleaved GCS command fragments (`MAVLINK_UPLINK`, $\le 49\text{B}$ payload, $\le 64\text{B}$ frame) or Link Control beacons (`LINK_CONTROL`, 0B payload, 15B frame).
   * **Single-Burst Limit:** Max payload is 49 bytes; maximum frame size is 64 bytes.
   * **Observed Airtime:** A 39B frame over 115,200 baud UART + 62.5 kbps LoRa air rate completes in $\approx 28.5\text{ ms}$, leaving $\ge 3.5\text{ ms}$ clearance before the slot boundary.

2. **Guard Gap 1 (5 ms / 32.0 ms – 37.0 ms):**
   * Turnaround silence window allowing Ground's E22 transmission to complete and the Air modem's AUX busy line to return HIGH (Idle).
   * Prevents modem collisions and provides receiver clock-out margin.

3. **Slot 2 (Air Transmit — 45 ms / 37.0 ms – 82.0 ms):**
   * **Primary Function:** High-throughput MAVLink telemetry downlink (`ATTITUDE`, `GLOBAL_POSITION_INT`, `SYS_STATUS`, `VFR_HUD`, `PARAM_VALUE`).
   * **Payload Cap:** Max 49 bytes per slot (`MAX_PAYLOAD_AIR_SLOT = 49`).
   * **Framed Size:** 13B header + up to 49B payload + 2B CRC = **$\le 64$ bytes** over UART (strictly respects E22 sub-packet boundary).
   * **Observed Airtime:** A full 64B frame finishes transmission in $\approx 38.7\text{ ms}$, concluding safely before the 82.0 ms boundary.

4. **Guard Gap 2 (8 ms / 82.0 ms – 90.0 ms):**
   * Turnaround margin and PLL synchronization window before the 90.0 ms frame wraps back to 0.0 ms (Ground slot start).

---

## 2. Linear Transit-Compensated PLL Discipline

The Dual-LRS link uses a master-slave timing hierarchy:
* **Ground Node:** Time Master. Runs a free-running hardware microsecond timer advancing unconditionally in 90.0 ms periods.
* **Air Node:** Time Slave. Disciplines its local `_frameStartTimeUs` timer to Ground frame arrivals.

### Transit Delay Formula

Ground transmits both `LINK_CONTROL` beacons (0-byte payload) and `RC_CONTROL` frames (24-byte payload). Air applies length-compensated arrival calculation:

$$\text{expectedArrivalUs} = (\text{plen} == 0) \ ?\ 10200\ :\ (10200 + \text{plen} \times 500)\,\mu\text{s}$$

Clamped at $32,000\,\mu\text{s}$ (Ground slot boundary).

### Graduated Slew Control

Air calculates phase error: $\text{err} = \text{targetStartUs} - \text{frameStartTimeUs}$, wrapped to $[-45000, +45000]\,\mu\text{s}$:

| Phase Error ($|\text{err}|$) | Correction Mode | Step Size | Behavior |
| :--- | :--- | :--- | :--- |
| $> 3000\,\mu\text{s}$ | Fast Snap | $\pm 250\,\mu\text{s}$ per frame | Rapid recovery without packet loss |
| $1000\,\mu\text{s} - 3000\,\mu\text{s}$ | Fast Slew | $\pm 100\,\mu\text{s}$ per frame | Smooth convergence |
| $500\,\mu\text{s} - 1000\,\mu\text{s}$ | Medium Slew | $\pm 25\,\mu\text{s}$ per frame | Steady approach |
| $250\,\mu\text{s} - 500\,\mu\text{s}$ | Micro Slew | $\pm 5\,\mu\text{s}$ per frame | Crystal drift compensation |
| $< 250\,\mu\text{s}$ | Deadband | $0\,\mu\text{s}$ (no adjustment) | Rejects UART & MCU execution jitter |

---

## 3. Phase 2 Over-the-Air Transport Framing (13-Byte Header + CRC-16)

All RF packets sent over the E22 LoRa transceiver use the Phase 2 `TransportHeader` with CRC-16-CCITT integrity verification:

```text
Byte Offset:   0                 12 13                   (13+N-1) (13+N)    (14+N)
             +---------------------+----------------------------+----------------+
             | TransportHeader     | Payload                    | CRC-16-CCITT   |
             | (13 Bytes Fixed)    | (N Bytes, N = 0 to 49)     | (2 Bytes)      |
             +---------------------+----------------------------+----------------+
```

### Exact `TransportHeader` Layout

| Byte Offset | Field Name | Type | Wire Order | Description |
|:---:|:---|:---:|:---:|:---|
| **0** | `magic0` | `uint8_t` | Byte | Synchronization byte 0: `'D'` (`0x44`) |
| **1** | `magic1` | `uint8_t` | Byte | Synchronization byte 1: `'L'` (`0x4C`) |
| **2** | `version` | `uint8_t` | Byte | Protocol version identifier: `0x01` |
| **3** | `channel` | `uint8_t` | Byte | Logical channel (`RC_CONTROL=1`, `LINK_CONTROL=2`, `MAVLINK_UPLINK=3`, `MAVLINK_DOWNLINK=4`) |
| **4** | `flags` | `uint8_t` | Byte | Control flags (`FIRST_FRAG=0x08`, `LAST_FRAG=0x10`, `RELIABLE=0x01`) |
| **5..6** | `sequence` | `uint16_t` | Big-Endian | Hop sequence counter ($0 \dots 65535$) |
| **7..8** | `transfer_id` | `uint16_t` | Big-Endian | Segmented message transaction identifier |
| **9..10** | `fragment_offset`| `uint16_t` | Big-Endian | Byte offset within logical multi-fragment message |
| **11..12** | `payload_length` | `uint16_t` | Big-Endian | Attached payload size ($0 \dots 49$ bytes) |

### CRC-16 Calculation
* **Polynomial:** `0x1021` ($X^{16} + X^{12} + X^5 + 1$)
* **Initial Value:** `0xFFFF`
* **Coverage:** Calculated over `TransportHeader` (13 bytes) followed by all payload bytes. Trailing 2 bytes serialized in Big-Endian order.

---

## 4. Multi-Fragment Segmentation & Reassembly

Messages larger than 49 bytes (e.g. MAVLink parameter tables, mission waypoints, command acknowledgements) are fragmented by `TransportFragmenter` into single-burst $\le 49\text{B}$ slices:

- **Offset 0:** `flags = TRANSPORT_FLAG_FIRST_FRAG`
- **Intermediate Chunks:** `flags = 0`, `fragment_offset = current_offset`
- **Final Chunk:** `flags = TRANSPORT_FLAG_LAST_FRAG`

The receiving node's `TransportReassembler`:
- Reconstructs fragments up to 512 bytes.
- Enforces range coverage, duplicate rejection, and gap detection.
- Emits intact, verified MAVLink packets directly to `telemHandler.writeToLocal()`.
