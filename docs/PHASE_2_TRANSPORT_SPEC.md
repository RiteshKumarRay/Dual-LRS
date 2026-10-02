# Dual-LRS Phase 2: Transport Layer Protocol Specification

**Document Version:** 1.2.0
**Status:** Architecture Review Package — Final Revision, No Production Transport Implemented
**Date:** 2026-10-01
**Project:** Dual-LRS (Air: STM32F411 BlackPill, Ground: ESP32-WROOM-32, Modem: Ebyte E22-900T30D)

---

## 1. Scope & System Architectural Position

The Dual-LRS transport protocol resides between the application layer (packed RC, MAVLink v1/v2 streams, Link Control) and the physical RF modem interface (Ebyte E22-900T30D transparent LoRa transceiver):

```text
+---------------------------------------------------------------------------------+
|                               APPLICATION LAYER                                 |
|   RC Control (Packed) |   MAVLink Commands/Missions   |   MAVLink Telemetry     |
+---------------------------------------------------------------------------------+
                                        │
                                        ▼
+---------------------------------------------------------------------------------+
|                                TRANSPORT LAYER                                  |
|   - 13-Byte Explicit Framing Header & 16-Bit CRC-CCITT Checksum                 |
|   - Explicit Big-Endian Wire Serialization (No Raw Struct Memory Copies)        |
|   - Exact Consecutive 11-Bit Little-Endian Bit-Packed RC Payload (24 Bytes)     |
|   - Strict Single-Burst Framing (Total Frame <= 64 Bytes)                       |
|   - Channel Multiplexing: Strict Priority Scheduling (RC > Link > Cmd > Telem)  |
|   - Bounded Segmentation & Reassembly (Max Transfer: 512B, Max 11 Fragments)    |
|   - Stop-and-Wait / Selective ACK/NACK Engine (Max 1 Active Transfer / Dir)     |
|   - Decoupled RC Failsafe Detection (Signals FC Without Modifying PWM/Channels) |
+---------------------------------------------------------------------------------+
                                        │
                                        ▼
+---------------------------------------------------------------------------------+
|                            PHYSICAL TDM TIME SLOTS                              |
|   - Conservative ~11.1 Hz Cycle (90.0 ms Total Period)                          |
|   - Ground Uplink Slot (RC / Link Ctrl)     : 32.0 ms (<= 39B Frame)            |
|   - Guard Gap 1 (Modem AUX & UART settling) :  5.0 ms                           |
|   - Air Downlink Slot  (MAVLink Telemetry)  : 45.0 ms (<= 64B Frame)            |
|   - Guard Gap 2 (Modem AUX & PLL re-sync)   :  8.0 ms                           |
+---------------------------------------------------------------------------------+
                                        │
                                        ▼
+---------------------------------------------------------------------------------+
|                        HARDWARE MODEM: EBYTE E22-900T30D                        |
|   - 64-Byte Hardware Sub-Packet Boundary (REG1 = 0x83)                          |
|   - 115,200 Baud UART (8N1) | 62.5 kbps Air Data Rate (REG0 = 0xE7)             |
+---------------------------------------------------------------------------------+
```

---

## 2. Framing Contract & Explicit Big-Endian Serialization

### 2.1 Transport Frame Structure
Every transport frame consists of a fixed **13-byte header**, followed by a **variable-length payload**, followed by a **2-byte CRC-16**:

```text
Byte Offset:   0                 12 13                   (13+N-1) (13+N)    (14+N)
             +---------------------+----------------------------+----------------+
             | TransportHeader     | Payload                    | CRC-16-CCITT   |
             | (13 Bytes Fixed)    | (N Bytes, N = 0 to 49)     | (2 Bytes)      |
             +---------------------+----------------------------+----------------+
```

### 2.2 Exact `TransportHeader` Field Layout and Wire Byte Offsets

| Byte Offset | Field Name | Type | Wire Order | Description |
|:---:|:---|:---:|:---:|:---|
| **0** | `magic0` | `uint8_t` | Byte | Protocol synchronization byte 0: `'D'` (`0x44`) |
| **1** | `magic1` | `uint8_t` | Byte | Protocol synchronization byte 1: `'L'` (`0x4C`) |
| **2** | `version` | `uint8_t` | Byte | Protocol version identifier: `0x01` |
| **3** | `channel` | `uint8_t` | Byte | Logical channel identifier (`TransportChannel` enum: `0x01`..`0x04`) |
| **4** | `flags` | `uint8_t` | Byte | Control bitmask (`RELIABLE`, `ACK`, `NACK`, `FIRST_FRAG`, `LAST_FRAG`) |
| **5..6** | `sequence` | `uint16_t` | Big-Endian | Hop-by-hop sequence number ($0\text{ to }65535$, Byte 5: MSB, Byte 6: LSB) |
| **7..8** | `transfer_id` | `uint16_t` | Big-Endian | Transaction identifier for segmented messages (Byte 7: MSB, Byte 8: LSB) |
| **9..10** | `fragment_offset`| `uint16_t` | Big-Endian | Byte offset of payload within message (Byte 9: MSB, Byte 10: LSB) |
| **11..12** | `payload_length` | `uint16_t` | Big-Endian | Payload byte count in this frame ($0\text{ to }49$, Byte 11: MSB, Byte 12: LSB) |

```cpp
#pragma pack(push, 1)
struct TransportHeader {
    uint8_t  magic0;          // Byte 0:  'D' (0x44)
    uint8_t  magic1;          // Byte 1:  'L' (0x4C)
    uint8_t  version;         // Byte 2:  0x01
    uint8_t  channel;         // Byte 3:  TransportChannel
    uint8_t  flags;           // Byte 4:  Control flags
    uint16_t sequence;        // Bytes 5-6:   Hop sequence (Host-Endian in RAM, Big-Endian on wire)
    uint16_t transfer_id;     // Bytes 7-8:   Transfer ID (Host-Endian in RAM, Big-Endian on wire)
    uint16_t fragment_offset; // Bytes 9-10:  Fragment byte offset (Host-Endian in RAM, Big-Endian on wire)
    uint16_t payload_length;  // Bytes 11-12: Payload bytes (Host-Endian in RAM, Big-Endian on wire)
};
#pragma pack(pop)

static_assert(sizeof(TransportHeader) == 13, "TransportHeader must be exactly 13 bytes");
```

### 2.3 Explicit Wire Serialization Contract (No Raw Struct Copies)
Packed C++ structs store multi-byte integer fields in native host endianness (little-endian on ARM Cortex-M4 STM32 and Tensilica Xtensa ESP32). Directly copying `TransportHeader` to the UART output buffer produces little-endian wire fields, violating the protocol contract.

Therefore, the protocol mandates explicit big-endian serialization helpers:
- `transport_write_u16_be(uint8_t* dest, uint16_t val)`: Writes `(val >> 8) & 0xFF` to `dest[0]` and `val & 0xFF` to `dest[1]`.
- `transport_read_u16_be(const uint8_t* src)`: Reconstructs `(src[0] << 8) | src[1]`.
- `transport_encode_header()` / `transport_decode_header()`: Serializes and deserializes the exact 13-byte wire buffer.
- `transport_encode_link_heartbeat()` / `transport_decode_link_heartbeat()`: Serializes and deserializes the exact 6-byte wire buffer.
- `transport_encode_link_ack_nack()` / `transport_decode_link_ack_nack()`: Serializes and deserializes the exact 8-byte wire buffer.

Raw memory copies (`memcpy` or type-punning pointer casts) to the radio UART are strictly forbidden.

### 2.4 Checksum Placement and Algorithm
- **Placement:** The CRC-16 checksum occupies the final 2 bytes of every frame, immediately following `payload[payload_length - 1]`.
  - Byte $13 + \text{payload\_length}$: CRC-16 MSB
  - Byte $14 + \text{payload\_length}$: CRC-16 LSB
- **Algorithm:** Standard CRC-16-CCITT:
  - Generator polynomial: $x^{16} + x^{12} + x^5 + 1$ (`0x1021`)
  - Initial value: `0xFFFF`
  - Input: Calculated over the entire 13-byte serialized wire header plus all payload bytes (`13 + payload_length` bytes).
  - Trailing bytes: Appended in Big-Endian order via `transport_write_u16_be`.

---

## 3. Sizing Proofs & Hardware Boundary Arithmetic

In Phase 1, empirical testing demonstrated that the E22-900T30D configured with `REG1 = 0x83` has a hard physical sub-packet limit of **64 bytes**.

### Frame Size Formula
$$\text{total\_frame\_size} = \text{transport\_header\_size} + \text{payload\_size} + \text{transport\_crc\_size}$$
Where:
- $\text{transport\_header\_size} = 13\text{ bytes}$
- $\text{transport\_crc\_size} = 2\text{ bytes}$
- $\text{transport\_overhead\_size} = 13 + 2 = 15\text{ bytes}$

### 3.1 Maximum Single-Burst Payload Proof
To guarantee that every frame is transmitted within a single LoRa sub-packet burst:
$$\text{total\_frame\_size} \le 64\text{ bytes}$$
$$13 + \text{payload\_size} + 2 \le 64$$
$$\text{payload\_size} \le 64 - 15 = 49\text{ bytes}$$

$$\mathbf{TRANSPORT\_MAX\_SINGLE\_BURST\_PAYLOAD = 49\text{ Bytes}}$$

- **Frame with 49B payload:** $\text{total\_frame\_size} = 13 + 49 + 2 = \mathbf{64\text{ Bytes}}$.
  $\rightarrow$ **Fits completely within single E22 sub-packet.** AUX busy duration $\approx 360\ \mu\text{s}$, zero inter-chunk gaps, one-way latency $\approx 38.68\text{ ms}$.
- **Frame with 50B payload:** $\text{total\_frame\_size} = 13 + 50 + 2 = \mathbf{65\text{ Bytes}}$.
  $\rightarrow$ **Crosses hardware boundary.** Triggers modem packet splitting, AUX busy expands to $\approx 27.0\text{ ms}$, inter-chunk receive gap of $5.8\text{--}6.8\text{ ms}$, one-way latency expands to $\approx 45.4\text{ ms}$.

### 3.2 Phase 2 v1 Single-Burst Rule & Removal of Split-Frame Chunk
**Phase 2 v1 strictly enforces:**
- **Maximum total transport frame:** **64 bytes**.
- **Maximum transport payload:** **49 bytes**.
- **All MAVLink fragments:** $\le 49\text{ bytes}$.
- **No normal split-frame transport path.**

> [!NOTE]
> **Deferred Future Research:** The previously considered 100-byte split-frame mode (total frame 115 bytes) is **strictly deferred** to future research and is NOT part of the Phase 2 v1 contract. Any frame $\ge 65$ bytes is rejected by the Phase 2 v1 parser.

---

## 4. Logical Channel Specifications & Sub-Protocols

### 4.1 Channel 1: `RC_CONTROL` (`0x01`, Ground $\rightarrow$ Air Only)
- **Role:** Deterministic flight control commands from ground transmitter to drone flight controller.
- **Priority:** Level 1 (Highest). Preempts all other channels.
- **Custom Packed RC Payload (`TransportPackedRc`):**
  - **Terminology Clarification:** This is a **custom packed RC payload**, NOT a complete CRSF frame over the RF link. It eliminates standard CRSF address, frame-type, and CRC bytes across the air to conserve byte budget. On the Air unit, the firmware unpacks these 16 channels and formats a standard CRSF or SBUS UART stream to the flight controller.
  - **Size:** Exactly 24 bytes ($13\text{B Header} + 24\text{B RC} + 2\text{B CRC} = \mathbf{39\text{ Bytes}} \le 64\text{ Bytes}$).
  - **Byte Layout:**
    - Bytes 0..21 (`channels[22]`): 16 channels packed into 22 bytes via exact 11-bit little-endian bit-packing.
    - Byte 22 (`rc_sequence`): 8-bit rolling sequence counter ($0\text{ to }255$) for link freshness tracking.
    - Byte 23 (`flags`): Control bitmask:
      - Bit 0 (`TRANSPORT_RC_FLAG_FAILSAFE` = `0x01`): Transmitter active failsafe condition.
      - Bit 1 (`TRANSPORT_RC_FLAG_FRAME_LOST` = `0x02`): Transmitter indicates uplink frame loss.
      - Bits 2..7: Reserved (`0x00`).

```cpp
#define TRANSPORT_RC_FLAG_FAILSAFE   (1 << 0)
#define TRANSPORT_RC_FLAG_FRAME_LOST (1 << 1)
#define TRANSPORT_RC_NUM_CHANNELS    16
#define TRANSPORT_RC_CHANNEL_MIN     0
#define TRANSPORT_RC_CHANNEL_MAX     2047

#pragma pack(push, 1)
struct TransportPackedRc {
    uint8_t  channels[22]; // 16 x 11-bit channels = 176 bits = 22 bytes
    uint8_t  rc_sequence;  // 8-bit rolling sequence counter (0..255)
    uint8_t  flags;        // Bit 0: Failsafe, Bit 1: Lost, Bits [7:2]: 0
};
#pragma pack(pop)

static_assert(sizeof(TransportPackedRc) == 24, "TransportPackedRc must be exactly 24 bytes");
```

### 4.2 Exact RC Bit-Packing Algorithm & Specification

The 16 channels are packed into 22 bytes using the following strict bit-level contract:
1. **Channel 0 Bit Position:** Starts at bit 0 of Byte 0.
2. **Bit Order:** Consecutive little-endian bit order. The least significant bit (LSB) of each channel is packed first.
3. **Channel Width & Range:** Each channel occupies exactly 11 bits (range: $0\text{ to }2047$, where 1024 represents center / 1500 µs).
4. **API Input Contract & Out-of-Range Behavior:**
   - The C++ API accepts `const uint16_t* channels`. Channel values are unsigned 16-bit integers already normalized to unsigned representation $[0\text{--}2047]$ by the RC input driver (e.g. CRSF or SBUS input decoder). Negative values cannot reach this unsigned API.
   - Out-of-range behavior for unsigned values $> 2047$:
     - If clamping is enabled (`clamp == true`): Values $> 2047$ clamp to $2047$.
     - If clamping is disabled (`clamp == false`): Values $> 2047$ cause the packing function to reject the input (returns `false` or raises `ValueError` in reference models).
5. **Exact Bit Mapping Across Bytes:**
   - Byte 0: Ch0[0..7]
   - Byte 1: Ch0[8..10] (bits 0..2) | Ch1[0..4] (bits 3..7)
   - Byte 2: Ch1[5..10] (bits 0..5) | Ch2[0..1] (bits 6..7)
   - Byte 3: Ch2[2..9] (bits 0..7)
   - Byte 4: Ch2[10] (bit 0) | Ch3[0..6] (bits 1..7)
   - Byte 5: Ch3[7..10] (bits 0..3) | Ch4[0..3] (bits 4..7)
   - Byte 6: Ch4[4..10] (bits 0..6) | Ch5[0] (bit 7)
   - Byte 7: Ch5[1..8] (bits 0..7)
   - Byte 8: Ch5[9..10] (bits 0..1) | Ch6[0..5] (bits 2..7)
   - Byte 9: Ch6[6..10] (bits 0..4) | Ch7[0..2] (bits 5..7)
   - Byte 10: Ch7[3..10] (bits 0..7)
   - Byte 11: Ch8[0..7] (bits 0..7)
   - Byte 12: Ch8[8..10] (bits 0..2) | Ch9[0..4] (bits 3..7)
   - Byte 13: Ch9[5..10] (bits 0..5) | Ch10[0..1] (bits 6..7)
   - Byte 14: Ch10[2..9] (bits 0..7)
   - Byte 15: Ch10[10] (bit 0) | Ch11[0..6] (bits 1..7)
   - Byte 16: Ch11[7..10] (bits 0..3) | Ch12[0..3] (bits 4..7)
   - Byte 17: Ch12[4..10] (bits 0..6) | Ch13[0] (bit 7)
   - Byte 18: Ch13[1..8] (bits 0..7)
   - Byte 19: Ch13[9..10] (bits 0..1) | Ch14[0..5] (bits 2..7)
   - Byte 20: Ch14[6..10] (bits 0..4) | Ch15[0..2] (bits 5..7)
   - Byte 21: Ch15[3..10] (bits 0..7)
   - Total bits: $16 \times 11 = 176\text{ bits} = 22\text{ bytes}$.

### 4.3 Channel 2: `LINK_CONTROL` (`0x02`, Bidirectional)
- **Role:** Link health monitoring, clock synchronization, and selective ACK/NACK signaling.
- **Priority:** Level 2.
- **Sub-Types:**

#### 1. Heartbeat & Link Statistics (`0x01`, 6 Bytes):
- Wire layout:
  - Byte 0: `subtype` (`0x01` = `HEARTBEAT`)
  - Byte 1: `link_quality` ($0\text{--}100\%$)
  - Byte 2: `rssi` (signed `int8_t` in dBm)
  - Byte 3: `snr` (signed `int8_t` in dB)
  - Bytes 4..5: `rtt_ms` (16-bit unsigned integer, Big-Endian)
- $\text{Total Frame} = 13 + 6 + 2 = \mathbf{21\text{ Bytes}} \le 64\text{ B}$.

#### 2. Selective ACK/NACK (`0x02`, 8 Bytes):
- Wire layout:
  - Byte 0: `subtype` (`0x02` = `ACK_NACK`)
  - Byte 1: `target_channel` (`TransportChannel` of acknowledged stream: `0x03` or `0x04`)
  - Bytes 2..3: `transfer_id` (16-bit Big-Endian transaction identifier)
  - Bytes 4..5: `fragment_offset` (16-bit Big-Endian byte offset of target fragment)
  - Byte 6: `nack_reason` (`TransportNackReason`: `0x00` = ACK_OK, `>0` = NACK reason code)
  - Byte 7: `reserved` (`0x00` padding byte)
- $\text{Total Frame} = 13 + 8 + 2 = \mathbf{23\text{ Bytes}} \le 64\text{ B}$.

```cpp
enum class TransportNackReason : uint8_t {
    NONE             = 0x00, // Valid ACK (no error)
    BAD_CRC          = 0x01, // Frame CRC failure detected by link
    GAP_DETECTED     = 0x02, // Fragment gap observed (missing fragment)
    BUFFER_FULL      = 0x03, // Reassembly buffer already occupied by ongoing active transfer
    OFFSET_OVERRUN   = 0x04, // Fragment offset exceeds TRANSPORT_MAX_TRANSFER_SIZE (512B)
    TRANSFER_TIMEOUT = 0x05, // Inactivity timeout reached before reassembly finished
    OVERLAP_CONFLICT = 0x06  // Partial range overlap detected with previously received fragment
};
```

### 4.4 Channels 3 & 4: `MAVLINK_UPLINK` (`0x03`) and `MAVLINK_DOWNLINK` (`0x04`)
- **Role:** Transparent transport of arbitrary MAVLink v1 or v2 packets.
- **Direction Enforcement:**
  - `MAVLINK_UPLINK` (`0x03`): Ground $\rightarrow$ Air only. Rejected if received on Ground.
  - `MAVLINK_DOWNLINK` (`0x04`): Air $\rightarrow$ Ground only. Rejected if received on Air.
- **Payload Sizing:** Chunks of **$1\text{ to }49\text{ bytes}$**.
- **Frame Sizing:** Total frame size strictly **$\le 64\text{ bytes}$**.

---

## 5. Sequence, Segmentation & Transfer Semantics

### 5.1 Sequence Number Width & Wraparound
- **Width:** 16-bit unsigned integer (`uint16_t`, $0\text{ to }65535$).
- **Scope:** Maintained independently per logical channel.
- **Wraparound Rule:** Sequences wrap from $65535 \rightarrow 0$.
- **Signed Distance Formula:**
  $$\Delta = (\text{int16\_t})(seq_{\text{rx}} - seq_{\text{exp}})$$
  - If $\Delta \ge 0 \land \Delta < 32768$: Received frame is newer or in-sequence.
  - If $\Delta < 0 \lor \Delta \ge 32768$: Received frame is duplicate or stale; discarded.

### 5.2 Transfer ID Semantics
- **Width:** 16-bit unsigned integer (`uint16_t`).
- **Function:** Identifies a logical multi-fragment message (such as a 280-byte MAVLink v2 packet).
- **Lifecycle:** Assigned on message dequeue, shared by all fragments, increments monotonically. Receiver pairs fragments via `(channel, transfer_id)`.

### 5.3 Fragment Offset Units, Bounds & Reassembly Rules
- **Units:** **Bytes** (0-indexed byte offset within the reassembled message).
- **Maximum Transfer Size:** **512 Bytes** (`TRANSPORT_MAX_TRANSFER_SIZE = 512`).
- **Maximum Fragment Count:** **11 Fragments** (`TRANSPORT_MAX_FRAGMENTS = 11`).
  - Arithmetic: $\lceil 512 / 49 \rceil = 11$.
  - 10 fragments of 49 bytes ($10 \times 49 = 490\text{ B}$) + 1 fragment of 22 bytes ($1 \times 22 = 22\text{ B}$) = 512 bytes total.
- **Maximum Concurrent Transfers:** Strictly **1 active transfer per direction** (`MAVLINK_UPLINK` Ground $\rightarrow$ Air, `MAVLINK_DOWNLINK` Air $\rightarrow$ Ground). Eliminates dynamic memory allocation on microcontrollers.
- **Active Transfer Preservation & No Preemption:**
  - While an active transfer is in progress (`!is_complete`), any arriving `FIRST_FRAG` with a different `transfer_id` is rejected immediately with `NACK_REASON_BUFFER_FULL`.
  - The ongoing in-progress transfer is preserved and continues uninterrupted until completion or inactivity timeout.
- **Completed Transfer Lifecycle & Atomic Takeover:**
  - Once a transfer completes (`is_complete == true`), the reassembled data remains valid and readable in memory.
  - The application may explicitly consume and release the buffer via `mark_complete_consumed()`.
  - If a new transfer arrives with `FIRST_FRAG` after completion, the reassembler atomically transitions to the new transfer, replacing the completed buffer.
  - Duplicate fragments of an already-completed transfer continue to be acknowledged idempotently.
- **Transfer Inactivity Timeout:** **1000 ms** (`TRANSPORT_TRANSFER_TIMEOUT_MS = 1000`). If incomplete after 1000 ms, receiver resets buffer and emits a NACK with `NACK_REASON_TRANSFER_TIMEOUT`.
- **Missing First Fragment Handling:** If a fragment arrives with `flags & FIRST_FRAG == 0` when no reassembly session is active for that `(channel, transfer_id)`, the receiver drops the fragment immediately and emits a NACK with `NACK_REASON_GAP_DETECTED`.
- **Duplicate & Overlap Handling:**
  - **Exact Duplicate Range:** If an arriving fragment matches an exact previously received range ($start == r\_start \land end == r\_end$):
    - Handled idempotently; an ACK is re-sent if `TRANSPORT_FLAG_RELIABLE` is set.
    - Buffer memory copy is bypassed (no duplicate write).
  - **Partial Range Overlap:** If an arriving fragment partially overlaps a previously received range ($\max(start, r\_start) < \min(end, r\_end)$ but ranges are not identical):
    - Rejected immediately with `NACK_REASON_OVERLAP_CONFLICT`.
    - Existing buffer memory is preserved and NOT overwritten.
  - **Offset Overrun:** If `fragment_offset + payload_length > 512`, frame is rejected immediately with `NACK_REASON_OFFSET_OVERRUN`.
- **Out-of-Order Handling:** Valid non-overlapping intermediate fragments arriving out of order are copied to `buffer + fragment_offset`. Reassembly is complete only when `LAST_FRAG` has arrived AND all byte ranges from 0 to total length are contiguous.

---

## 6. Reliability, Retransmission & Error Recovery

### 6.1 Reliability Classification by Channel
- **Unreliable (Best Effort, No Retransmissions):** `RC_CONTROL`. Freshness takes absolute precedence over recovery. Stale control frames are dropped immediately upon arrival of newer ones.
- **Reliable (ACK / Retransmission Enabled):** Critical MAVLink commands, mission uploads, parameter writes, and parameter downloads.

### 6.2 Retransmission Timing & Retry Policy
- **TDM Cycle Period:** $90.0\text{ ms}$ (~11.1 Hz).
- **Half-Duplex Round-Trip Duration:** In a 2-slot TDM cycle, a frame transmitted in slot 1 arrives within 32 ms; its ACK is queued and transmitted in slot 2 (37--82 ms), arriving back at the sender by $t = 82\text{ ms}$. One full exchange completes within 1 TDM cycle ($90\text{ ms}$).
- **Retransmission Timeout ($T_{\text{RTO}}$):**
  - Allowing for 1-cycle queuing latency plus scheduling jitter:
    $$T_{\text{RTO}} = 2 \times T_{\text{TDM}} + \text{margin} = 2 \times 90.0\text{ ms} + 20.0\text{ ms} = \mathbf{200\text{ ms}}$$
- **Retry Limit:** Strictly **5 retry attempts** (`TRANSPORT_MAX_RETRIES = 5`).
  - If 5 retries expire without receiving an ACK: transfer is aborted, `statTxTransferAborts++` is recorded, and the channel advances to the next queued message.

### 6.3 Reconnection & Link Loss Behavior
- **Link Loss Threshold:** If no valid frames arrive for $> 1,000\text{ ms}$ (~11 missing TDM cycles):
  - Link status transitions to `LINK_SEARCHING / RECONNECTING`.
  - All pending unacknowledged reassembly buffers are cleared.
  - RC failsafe state is asserted.
- **Reconnection:** Once valid magic bytes and header are decoded, link transitions to `LINK_SYNCHRONIZED` after **3 consecutive valid frames** without requiring MCU reboot.

---

## 7. RC Failsafe Decoupling & Memory Bounds

### 7.1 RC Failsafe Specification (Decoupled Architecture)
- **Architectural Principle:** The transport layer is responsible **only** for link loss detection and protocol-level failsafe signaling. It **DOES NOT** manipulate control channel PWM values, DOES NOT force Channel 3 to 900 µs, and DOES NOT force Channel 5 to RTL.
- **Monitoring:** Air unit tracks arrival timestamps of `RC_CONTROL` frames.
- **Configurable Loss Timeout:** Default **500 ms** (`TRANSPORT_RC_FAILSAFE_TIMEOUT_MS = 500`), configurable between 200 ms and 2000 ms.
- **Action upon Timeout:**
  - Air transport engine asserts internal failsafe state (`failsafe_active = true`).
  - The Air unit asserts the protocol-native failsafe indicator in frames passed to the Flight Controller (e.g. CRSF failsafe flag bit or SBUS failsafe bit).
  - The Flight Controller executes its own autonomous failsafe policy (RTL, LAND, HOLD, or TERMINATE) as configured by ArduPilot / Betaflight parameters.
- **Restoration:** Requires **3 consecutive valid `RC_CONTROL` frames** (`TRANSPORT_RC_RESTORE_FRAME_COUNT = 3`) to disengage the failsafe flag and resume normal channel forwarding.

### 7.2 Memory Bounds & Queue Capacities

| Queue / Buffer | Storage Location | Capacity | Allocation | Sizing Rationale |
|:---|:---|:---:|:---:|:---|
| **RC Queue** | RAM | 1 Frame | 39 Bytes | Single-entry mailbox; newest frame overwrites older |
| **Link Control Queue**| RAM | 4 Frames | 256 Bytes | Fixed ring buffer for pending ACKs/Stats |
| **MAVLink Cmd Queue** | RAM | 2 Transfers | 1,024 Bytes| Holds up to 2 complete 512B commands |
| **MAVLink Telem FIFO**| RAM | 48 KB (Air) / 1 KB (Gnd) | Circular | Absorbs full ArduPilot parameter table burst |
| **Reassembly Buffer** | RAM | 1 Active Buffer | 512 Bytes | Reassembles incoming multi-fragment message |
| **Total Transport RAM**| RAM | — | **$< 3.0\text{ KB}$** | Safe on BlackPill (128 KB) & ESP32 (320 KB) |

---

## 8. Conservative TDM Slot Budget Grounded in Phase 1 Data

Phase 1 established that single-packet frames ($\le 64\text{ B}$) require $\approx 21.2\text{--}38.7\text{ ms}$ total transfer time.

### 8.1 Correction of the RC Uplink Slot Contradiction
In earlier iterations, the Ground uplink slot was assigned $25\text{ ms}$ while a 39-byte `TransportPackedRc` frame required approximately $28.5\text{ ms}$ transfer time. A $28.5\text{ ms}$ physical transfer **cannot fit** in a $25\text{ ms}$ slot.

Therefore:
- **The $25\text{ ms}$ Ground uplink slot is explicitly marked UNAPPROVED** pending dedicated physical simplex measurements with shortened RF preambles.
- The operational bench schedule expands the Ground uplink slot to **$32.0\text{ ms}$**, ensuring $+3.5\text{ ms}$ of positive headroom for 39-byte RC frames.

### 8.2 Conservative Initial TDM Schedule (~11.1 Hz / 90.0 ms Total Cycle)

```text
0.0 ms                 32.0 ms 37.0 ms                             82.0 ms 90.0 ms
|-----------------------|-------|-----------------------------------|-------|
| Slot 1: Ground Uplink | Guard | Slot 2: Air Downlink              | Guard |
| (RC + Link Control)   | Gap 1 | (MAVLink Telemetry & Commands)    | Gap 2 |
| Duration: 32.0 ms     | 5.0 ms| Duration: 45.0 ms                 | 8.0 ms |
| Max Frame: <= 39 B    |       | Max Frame: <= 64 B (Single-Burst) |       |
```

### 8.3 Detailed Arithmetic and Margin Breakdown

1. **Slot 1: Ground Uplink ($32.0\text{ ms}$):**
   - **Payloads:** 1 `RC_CONTROL` frame ($39\text{ B}$) OR 1 `LINK_CONTROL` frame ($21\text{--}23\text{ B}$).
   - **Empirical Basis & Headroom:**
     - For 39-byte RC frame: Phase 1 measured Mode C ping-pong transfer time for 41B frame at $29.34\text{ ms}$ (including turnaround). Simplex baseline estimate without turnaround: $\approx 28.5\text{ ms}$.
       $$\text{Margin}_{\text{RC}} = 32.0\text{ ms} - 28.5\text{ ms} = \mathbf{+3.5\text{ ms}}$$
     - For 21-byte Link Control frame: Phase 1 measured physical transfer time is **$21.22\text{ ms}$** (Mode C 10B payload).
       $$\text{Margin}_{\text{Link}} = 32.0\text{ ms} - 21.22\text{ ms} = \mathbf{+10.78\text{ ms}}$$
2. **Guard Gap 1 ($5.0\text{ ms}$):**
   - **Purpose:** Accommodates Ground E22 AUX pin fall time to IDLE, RF transmission decay, and Air UART input FIFO drain.
3. **Slot 2: Air Downlink ($45.0\text{ ms}$):**
   - **Payloads:** 1 `MAVLINK_DOWNLINK` frame ($49\text{B}$ payload, $64\text{B}$ total frame).
   - **Empirical Basis & Headroom:** Phase 1 measured physical one-way transfer time for a full 64-byte frame is **38.68 ms** (Mode C 53B payload).
     $$\text{Margin}_{\text{Downlink}} = 45.0\text{ ms} - 38.68\text{ ms} = \mathbf{+6.32\text{ ms}}$$
     Provides $+6.32\text{ ms}$ of safe clearance before Guard Gap 2.
4. **Guard Gap 2 ($8.0\text{ ms}$):**
   - **Purpose:** Accommodates Air E22 AUX pin return to IDLE, Ground UART FIFO drain, and PLL clock phase drift re-synchronization between unsynchronized microcontroller crystal oscillators.
5. **Total Cycle Arithmetic:**
   $$T_{\text{cycle}} = 32.0\text{ ms} + 5.0\text{ ms} + 45.0\text{ ms} + 8.0\text{ ms} = \mathbf{90.0\text{ ms}}$$
   $$f_{\text{cycle}} = \frac{1000}{90.0} \approx \mathbf{11.11\text{ Hz}\ (\sim 11.1\text{ Hz})}$$

> [!WARNING]
> **Operational Status:** This schedule is an initial conservative proposal derived from benchtop empirical measurements. **It is NOT yet flight-proven.** Real flight dynamics, antenna orientations, and RF interference will require further tuning during Phase 3 flight validation.

---

## 9. Malformed-Input Rejection Rules

Any frame failing any of the following checks must be discarded immediately without memory copying:
1. **Magic Rejection:** If `magic0 != 'D' || magic1 != 'L'`, parser drops bytes until next `'D'`.
2. **Version Rejection:** If `version != 0x01`, frame rejected (`statRxUnsupportedVersion++`).
3. **Channel Rejection:** If `channel` is not one of `{0x01, 0x02, 0x03, 0x04}`, frame rejected (`statRxInvalidChannel++`).
4. **Direction Rejection:** If a frame arrives on an illegal link direction (e.g. `RC_CONTROL` or `MAVLINK_UPLINK` arriving on Ground, or `MAVLINK_DOWNLINK` arriving on Air), frame rejected (`statRxInvalidDirection++`).
5. **Length Overrun Rejection:** If `payload_length > TRANSPORT_MAX_SINGLE_BURST_PAYLOAD` (49), frame rejected (`statRxPayloadTooLarge++`).
6. **CRC Mismatch:** Computed CRC-16 over `[0 .. 13 + payload_length - 1]` must equal received CRC. If mismatched, frame rejected (`statRxCrcErrors++`).
7. **Offset Overrun:** If `fragment_offset + payload_length > TRANSPORT_MAX_TRANSFER_SIZE` (512), frame rejected (`statRxOffsetOverrun++`).
8. **Orphan Fragment:** If an intermediate or last fragment arrives (`flags & FIRST_FRAG == 0`) with no active reassembly transfer, frame rejected (`statRxOrphanFragment++`).
