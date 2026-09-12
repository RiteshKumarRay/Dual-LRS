# Dual-LRS TDM (Time Division Multiplexing) Protocol

Because LoRa transceivers (Semtech SX1262) operate on a single RF frequency, physical transmission is **half-duplex** (the radio can only transmit or receive at any given instant).

To achieve seamless, bidirectional MAVLink telemetry between Mission Planner and ArduPilot, Dual-LRS implements a deterministic **Time Division Multiplexing (TDM)** frame scheduler.

---

## 1. TDM Frame Structure (50 ms Cycle / 20 Hz)

Each 50 ms cycle is partitioned into asymmetric slots optimized for drone telemetry:

```
+------------------------------------+-------+----------------+-------+
|        Slot 1: Air Transmit        | Gap 1 | Slot 2: Ground | Gap 2 |
|      (Downlink Telemetry)          | Guard | (Uplink / RC)  | Guard |
|              30 ms                 | 4 ms  |     12 ms      | 4 ms  |
+------------------------------------+-------+----------------+-------+
|<--------------------------- 50 ms Total Frame --------------------->|
```

- **Slot 1 (Air Transmit - 30 ms / 60% bandwidth)**:
  ArduPilot generates a high-volume stream of telemetry packets (`ATTITUDE`, `GLOBAL_POSITION_INT`, `SYS_STATUS`, `VFR_HUD`, `HEARTBEAT`). Allocating 60% of the frame ensures high refresh rates on the Ground Control Station.
- **Guard Gap 1 (4 ms)**:
  Allows the E22 internal RF switch to turnaround from TX to RX, finishes transmitting trailing bytes, and clears MCU serial buffers.
- **Slot 2 (Ground Transmit - 12 ms / 24% bandwidth)**:
  Mission Planner sends infrequent parameter requests, waypoint updates, mode switches, or high-rate RC channel overrides.
- **Guard Gap 2 (4 ms)**:
  Prepares the Air node for the next downlink burst.

---

## 2. Frame Synchronization & Clock Drift Compensation

1. **Master / Slave Hierarchy**:
   - **Ground Node** is the **Time Master**: Its hardware timer increments unconditionally in 50 ms periods.
   - **Air Node** is the **Time Slave**: It locks its local frame timer to the exact arrival timestamp of Ground packets.
2. **Sync Recovery (Loss of Signal)**:
   - If the Air node does not receive a Ground packet for **1.5 seconds**, it flags `synchronized = false`.
   - In unsynchronized mode, the Air unit listens continuously, and as soon as the Ground node transmits, the Air unit immediately re-locks its phase without manual intervention.

---

## 3. Over-the-Air Packet Structure

Each RF packet sent over the E22 link is framed with a binary header and CRC-16 validation:

```
[0..1] Magic Bytes: 0x44, 0x4C ('D', 'L')
[2]    Packet Type: 
         - 0x01: HEARTBEAT_SYNC
         - 0x02: MAVLINK_DATA
         - 0x03: RC_OVERRIDE
         - 0x04: RADIO_STATUS
[3]    Sequence Number: 8-bit rolling counter (0-255)
[4]    Payload Length: N bytes (0 - 240)
[5..N+4] Payload Data: Raw MAVLink bytes
[N+5..N+6] CRC-16-CCITT: Polynomial 0x1021
```

### Sequence Tracking & Link Quality
- Every received packet is checked against the previous sequence number.
- Any gaps in the sequence increment the `packets_dropped` counter.
- Link Quality is computed every second:
  $$\text{Link Quality} = \frac{\text{Packets Received}}{\text{Packets Received} + \text{Packets Dropped}} \times 100\%$$
- This percentage is automatically packed into a standard MAVLink `RADIO_STATUS` (Message ID 109) packet and sent to Mission Planner to populate the HUD signal bars.
