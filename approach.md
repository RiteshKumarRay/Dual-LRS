# Dual-LRS Engineering Approach

## Purpose

This document is the implementation contract for Dual-LRS.

Dual-LRS is a dedicated, open-source RC and MAVLink telemetry link for the
specific hardware already available:

- STM32F411 BlackPill air unit
- ESP32-WROOM-32 or STM32F411 ground unit
- One Ebyte E22-900T30D UART LoRa modem at each end
- ArduPilot flight controller
- A ground RC input source
- QGroundControl and/or Mission Planner

The goal is not to reproduce all of mLRS. The goal is to build a reliable,
understandable system for this hardware which provides:

1. RC channels from ground to air
2. RC output from air to the flight controller
3. Bidirectional MAVLink
4. Flight-controller telemetry to the ground station
5. Mission upload
6. Mission download
7. Parameter read and write
8. Command and flight-mode changes
9. Heartbeats and failsafe behavior
10. Link statistics
11. Reliable reconnection
12. Optional MAVLink passthrough to a GCS

Features intentionally out of scope:

- FHSS
- Direct SX1262/SX127x SPI control
- Native mLRS firmware compatibility
- mLRS binding/configuration compatibility
- Diversity algorithms
- Dual-band operation
- General-purpose transparent serial operation
- A market claim of being mLRS

mLRS is a reference for behavior, protocol handling, failsafe design,
prioritization, and validation discipline. It is not a drop-in firmware base
for this hardware.

---

## 1. Non-negotiable technical decision

Normal mLRS firmware cannot be flashed onto this hardware and expected to work.
mLRS assumes direct control of supported Semtech radios, normally through a
radio driver with SPI, IRQ, DIO, BUSY, reset, and precise TX/RX state control.

The E22-900T30D hides the SX1262 behind its own UART modem firmware. The MCU
can use UART, AUX, M0, and M1, but cannot reliably control individual LoRa
packets, direct radio interrupts, exact modem airtime, native FHSS, or direct
radio RSSI behavior.

Therefore:

```text
Dual-LRS application protocol
        ↓
Dual-LRS E22 modem transport
        ↓
E22 UART transparent LoRa modem
```

Do not spend project time trying to make the existing mLRS SPI radio driver
operate through the E22 UART interface. A future mLRS E22 backend would be a
large separate port and is not the current project.

---

## 2. Hardware contract

The implementation must not silently assume a different board or modem.
Record any hardware change in this file and in the relevant source header.

### 2.1 Air unit

- STM32F411CEU6 BlackPill
- Radio UART: USART1
  - PA9: MCU TX → E22 RXD
  - PA10: MCU RX ← E22 TXD
- Flight-controller UART: USART2
  - PA2: MCU TX → FC RX
  - PA3: MCU RX ← FC TX
- E22 control:
  - PB0: M0
  - PB1: M1
  - PB10: AUX
- USB CDC is diagnostic/maintenance only and must never pollute the FC MAVLink
  stream.

### 2.2 Ground unit

ESP32 is the preferred ground target because it has separate hardware UARTs.

- Radio UART: Serial2
  - GPIO17: MCU TX → E22 RXD
  - GPIO16: MCU RX ← E22 TXD
- GCS UART/USB: Serial/USB bridge
- E22 control:
  - GPIO21: M0
  - GPIO22: M1
  - GPIO19: AUX

The STM32 ground target may be retained, but its USB CDC and USART ownership
must be explicitly verified before it is treated as a supported target.

### 2.3 E22 baseline

The exact modem configuration must be captured and tested at both ends:

- UART: 115200, 8N1
- Transparent mode
- Same frequency channel
- Same air data rate
- Same packet/sub-packet setting
- Same bandwidth, spreading factor, coding rate, and power configuration as
  supported by the E22 firmware
- AUX connected and interpreted as busy/ready

Do not claim a specific RF throughput from a datasheet alone. Measure the
complete path:

```text
MCU UART write
→ E22 buffering
→ RF transmission
→ remote E22 buffering
→ remote MCU UART read
```

The measured result controls slot sizing.

### 2.4 Safety and security

- Never test with propellers installed.
- Begin at the lowest practical RF power and short range.
- Use dummy loads or adequate separation for bench tests.
- Do not commit passwords, Wi-Fi credentials, SSH credentials, or private
  infrastructure details.
- Any credentials previously stored in context or configuration files must be
  removed and rotated if they were real.

---

## 3. System architecture

The firmware must be divided into independently testable layers.

```text
                 AIR                                  GROUND

 ArduPilot FC ── FC UART                  GCS/RC input ── local interfaces
       │                                      │
       ▼                                      ▼
 MAVLink endpoint                         MAVLink endpoint
 RC output endpoint                       RC input endpoint
       │                                      │
       └────────────── Application channels ──┘
                          │
                   Link transport
                          │
                    TDM scheduler
                          │
                     E22 driver
                          │
                    E22 UART modem
```

Required source-level separation:

1. `e22_driver`: UART, AUX, M0/M1, modem configuration, ready/busy behavior.
2. `tdm_engine`: frame timing, slot ownership, transport frame encoding,
   decoding, CRC, synchronization.
3. `link_transport`: sequence numbers, fragmentation, acknowledgements,
   duplicate suppression, retransmission policy, channel queues, link state.
4. `mavlink_endpoint`: MAVLink parsing, validation, frame preservation,
   prioritization, mission/parameter/command handling.
5. `rc_endpoint`: ground RC input, compact RC frame encoding, air RC output,
   age tracking, failsafe.
6. `link_stats`: packet counters, loss, retries, latency, RSSI if actually
   available, and status generation.
7. `main`: hardware wiring and cooperative scheduling only.

Do not continue adding unrelated special cases to `mavlink_handler.cpp` if the
same behavior belongs in `link_transport` or `rc_endpoint`.

---

## 4. Implementation order

The order is mandatory. Do not implement RC and mission reliability at the
same time before the base transport is measurable.

### Phase 0: Freeze and document the baseline

Before changing protocol behavior:

1. Identify the exact E22 module variant and firmware/configuration.
2. Confirm the air MCU and ground MCU target.
3. Confirm the RC input protocol available from the transmitter.
4. Confirm the FC RC output protocol: CRSF, SBUS, or MAVLink RC override.
5. Confirm the FC telemetry UART and baud.
6. Capture the current E22 AUX timing for several payload lengths.
7. Remove credentials from source and lab notes.
8. Make all README, context, TDM, and configuration timing values agree.
9. Record the current uncommitted state before making further changes.

Deliverable: a hardware baseline table and a reproducible bench setup.

### Phase 1: Prove the raw E22 link

Do not involve MAVLink or RC yet.

Implement a deterministic test mode that sends numbered payloads in both
directions and records:

- Sequence received
- Missing sequence numbers
- Duplicate sequence numbers
- Corrupted payloads
- One-way latency
- Round-trip latency
- AUX busy duration
- UART overruns
- Reconnection time after power or RF interruption

Acceptance gate:

- No unexpected corruption during a sustained test
- Loss and duplicate behavior are measured rather than guessed
- A disconnected modem is detected
- Reconnection works without rebooting the MCU

### Phase 2: Implement the transport protocol

Replace raw MAVLink-byte scheduling with an explicit Dual-LRS transport.
Transport frames must be independent of MAVLink framing.

Minimum frame fields:

```cpp
struct TransportHeader {
    uint8_t  magic0;
    uint8_t  magic1;
    uint8_t  version;
    uint8_t  channel;
    uint8_t  flags;
    uint16_t sequence;
    uint16_t transfer_id;
    uint16_t fragment_offset;
    uint16_t payload_length;
};
```

Follow the repository's actual packing and endianness conventions. Add a
compile-time size assertion and bounds checks.

Every transport frame must have:

- Magic/version validation
- Channel validation
- Maximum-length validation
- Sequence handling
- CRC over header and payload
- Explicit rejection statistics

Required channels:

```text
RC_CONTROL       ground → air
MAVLINK_UPLINK   ground → air
MAVLINK_DOWNLINK air → ground
LINK_CONTROL     either direction
```

Priority order:

1. RC control
2. Link control and acknowledgements
3. MAVLink command/mission traffic
4. MAVLink heartbeat and urgent status
5. MAVLink parameter traffic
6. Essential telemetry
7. Routine telemetry

Do not let a large parameter transfer block RC or link-control frames.

### Phase 3: MAVLink-only reliability

Before adding RC, make the MAVLink path reliable.

Required behavior:

- Parse complete MAVLink v1 and v2 frames.
- Validate MAVLink CRC before forwarding.
- Preserve signed MAVLink frames without corrupting their signature bytes.
- Fragment complete MAVLink frames into transport frames.
- Reassemble by transfer ID and offset.
- Reject gaps, duplicates, invalid lengths, and stale transfers.
- Forward only a complete validated MAVLink frame to the destination UART.
- Never inject RADIO_STATUS into a partially forwarded MAVLink frame.
- Keep separate uplink and downlink queues.
- Keep mission traffic independent from routine telemetry.

Required test cases:

1. Heartbeat both directions.
2. Parameter download.
3. Parameter write and read-back.
4. Command acknowledgement.
5. Flight-mode change.
6. Mission upload.
7. Mission download.
8. Mission transfer while routine telemetry is active.
9. Corrupted transport frame.
10. Dropped fragment.
11. Duplicate fragment.
12. RF interruption and recovery.

Acceptance gate:

- Ten consecutive parameter downloads succeed.
- Ten consecutive mission uploads succeed.
- Ten consecutive mission downloads succeed.
- No invalid MAVLink bytes reach the FC or GCS in the test logs.
- The link recovers after interruption without firmware restart.

### Phase 4: RC transport

Only begin RC after Phase 3 passes.

Use a dedicated fixed-rate RC channel. RC must never depend on the MAVLink
queue being empty.

Ground RC frame requirements:

- Protocol version
- Sequence number
- Channel count
- Packed channel values
- Validity flags
- Optional switch/mode flags
- Sender timestamp or frame age
- CRC

Start conservatively:

- 8 channels at full required resolution
- 20 Hz update rate
- Explicit mapping table
- Fixed reserved TDM capacity

Expand to more channels only after measuring transport margin.

Air-side RC behavior:

1. Accept only valid and newer frames.
2. Reject stale or duplicated frames.
3. Track time since last valid RC frame.
4. Output the selected protocol to ArduPilot.
5. Enter an explicitly configured failsafe state after timeout.
6. Recover only after a valid, newer RC frame is received.
7. Record failsafe enter and exit counters.

Choose the air-to-FC RC protocol based on actual FC wiring:

- Prefer CRSF if the FC and UART arrangement support it.
- Use SBUS if inversion and electrical levels are correctly handled.
- Use MAVLink RC override only as a deliberate fallback; it has different
  latency and failsafe semantics and should not be treated as equivalent to
  native RC input.

Acceptance gate:

- Every channel maps correctly.
- RC update rate and latency are measured.
- Loss of the ground unit produces the intended ArduPilot failsafe.
- Reconnection restores control without stale channel jumps.
- RC remains responsive during parameter and mission transfers.

### Phase 5: Link statistics and operational hardening

Add only statistics that can be measured:

- Transport frames sent/received
- CRC errors
- Invalid headers
- Dropped frames
- Duplicate frames
- Reordered frames
- Retransmissions
- Per-channel queue depth
- Maximum queue depth
- RC frame age
- MAVLink frame latency
- Last valid packet time
- Link state
- Failsafe state
- RSSI only if the E22 interface exposes trustworthy RSSI data

Generate RADIO_STATUS only from real counters. Do not report hardcoded 100%
link quality or fabricated RSSI.

Implement a link state machine:

```text
STARTING
  → SEARCHING
  → SYNCHRONIZED
  → DEGRADED
  → LOST
  → RECOVERING
  → SYNCHRONIZED
```

State transitions must be timeout-based and logged. Reconnection must reset
stale reassembly state and queues safely without losing the RC failsafe
transition.

---

## 5. TDM synchronization

The E22 is half-duplex and opaque. TDM timing must be based on measurement.

Initial schedule may be:

```text
50 ms frame / 20 Hz

Air → Ground slot
Guard gap
Ground → Air slot
Guard gap
```

The exact slot widths are not permanent until measured. Documentation,
configuration, and tests must use the same values.

Rules:

1. Only one endpoint may transmit in a slot.
2. A node must not transmit while its E22 AUX is busy.
3. A node must skip a slot if it cannot prove the modem will finish safely.
4. A skipped slot is preferable to a collision.
5. RC has reserved capacity in the ground-to-air slot.
6. Control/acknowledgement traffic has reserved capacity in both directions.
7. MAVLink fragments use the remaining capacity.
8. Slot timing must be tested with minimum and maximum payloads.
9. The scheduler must not use blocking waits that starve UART parsing.
10. All timer arithmetic must be wraparound-safe.

The initial implementation may use Ground as the time master and Air as the
slave. Sync beacons must include enough information to distinguish:

- A new frame
- A duplicate beacon
- A late beacon
- A stale beacon
- A beacon with invalid CRC

Avoid continuously moving the frame clock based on noisy packet arrival.
Use bounded correction and a measured deadband. Log phase error during
development.

---

## 6. MAVLink behavior requirements

### 6.1 Endpoint rules

The Air endpoint talks only to the flight controller. The Ground endpoint
talks only to the GCS/ground application and the RC source. Binary MAVLink
must not be mixed with diagnostics on the same serial stream.

### 6.2 Priority

MAVLink queues must use message classes, not an uncontrolled FIFO:

1. Command acknowledgement and mode response
2. Mission protocol
3. Heartbeat
4. Parameter requests/responses
5. STATUSTEXT
6. Essential state telemetry
7. Routine telemetry

Message filtering must be conservative. Do not silently drop unknown messages
unless a documented policy explicitly permits it. TIMESYNC shedding may be
enabled only after verifying that the selected GCS and vehicle do not require
it.

### 6.3 Mission protocol

Mission upload and download are request/response protocols. Do not treat them
as a bulk stream.

The implementation must preserve ordering and support:

- `MISSION_COUNT`
- `MISSION_REQUEST`
- `MISSION_REQUEST_INT`
- `MISSION_ITEM`
- `MISSION_ITEM_INT`
- `MISSION_ACK`
- `MISSION_REQUEST_LIST`
- `MISSION_CLEAR_ALL`
- Relevant mission type fields where present

Each complete MAVLink frame must be delivered once, in order, or explicitly
retried. Mission state must not be lost because a heartbeat or telemetry
packet arrived.

### 6.4 Parameter protocol

Parameter transfers require enough buffering and flow control to avoid
deadlock. Queue occupancy must not be reported to ArduPilot using arbitrary
thresholds. If RADIO_STATUS `txbuf` is used for flow control, document and
test its relationship to actual available buffer space.

---

## 7. Testing strategy

Every protocol change must have a test or a reproducible hardware procedure.

### 7.1 Host-side tests

Where possible, extract pure C++ protocol code so it can be compiled on the
host without Arduino hardware. Test:

- CRC vectors
- Header encoding/decoding
- Length bounds
- Sequence comparison and wraparound
- Fragmentation/reassembly
- Duplicate suppression
- Timeout behavior
- Queue priority
- RC packing/unpacking
- MAVLink v1/v2 validation

### 7.2 Hardware bench tests

Use a fixed test log format containing:

- Git commit
- Board target
- E22 configuration
- RF power
- Frequency/channel
- UART baud
- ArduPilot version
- GCS version
- Test duration
- Distance and antenna setup
- Result and failure counters

Minimum hardware tests:

1. Raw link soak test.
2. MAVLink heartbeat.
3. Parameter download.
4. Parameter write/read-back.
5. Mission upload.
6. Mission download.
7. Mode/command test.
8. RC channel sweep.
9. RC failsafe.
10. RF power interruption.
11. Ground MCU reset.
12. Air MCU reset.
13. GCS disconnect/reconnect.
14. Simultaneous RC, telemetry, and mission traffic.

Do not report “reliable” from one successful run. Use repeated runs and state
the count.

### 7.3 Flight safety gate

Flight testing is allowed only after:

- RC failsafe is proven on the bench.
- RC channel mapping is verified.
- Mission operations are stable on the bench.
- Link loss produces the intended ArduPilot action.
- Recovery from link loss is understood.
- No serial debug pollution is present.
- Antennas, power supply, grounding, and RF power are appropriate.

---

## 8. How to use Antigravity or VS Code AI agents

This file is the project contract. Agents must read it before changing code.
Do not ask an agent to “make Dual-LRS like mLRS” as one large task. That
produces broad, untestable changes.

### 8.1 Agent operating rules

Every agent prompt must require:

1. Read `approach.md`, `README.md`, `context.md`, and relevant source files.
2. State the exact phase and acceptance gate being worked on.
3. Inspect the current Git diff before editing.
4. Make one bounded change at a time.
5. Preserve unrelated user changes.
6. Add or update tests for changed behavior.
7. Run the smallest relevant build/test command.
8. Report changed files, test results, assumptions, and remaining risks.
9. Never claim hardware validation from a compile-only result.
10. Stop and ask for clarification when hardware behavior is unknown.

### 8.2 Recommended agent sequence

Use separate agent tasks in this order:

#### Agent A: Baseline and hygiene

```text
Read approach.md and audit the current Dual-LRS repository against Phase 0.
Do not redesign the protocol. Identify stale documentation, credentials,
build reproducibility problems, duplicate project copies, and current
uncommitted changes. Produce a report and a minimal cleanup patch only where
safe. Run available syntax/build checks.
```

#### Agent B: Raw E22 measurement harness

```text
Implement a bounded raw E22 link test harness for the exact configured
hardware. Measure sequence loss, duplicates, CRC errors, AUX busy duration,
and recovery. Do not modify MAVLink behavior. Provide a reproducible command
and a machine-readable log format.
```

#### Agent C: Transport protocol

```text
Implement only the Dual-LRS transport frame, CRC, sequence, and fragmentation
module described in approach.md. Keep it independent from Arduino hardware
where practical. Add host-side tests for bounds, CRC, sequence wraparound,
fragment loss, duplicates, and reassembly timeout. Do not add RC yet.
```

#### Agent D: MAVLink reliability

```text
Integrate the tested transport with MAVLink v1/v2. Preserve complete frames,
validate CRC, implement mission and parameter priority, and prevent diagnostic
injection into partial frames. Add tests and run the Phase 3 acceptance
procedures. Do not redesign TDM or add RC in this task.
```

#### Agent E: RC endpoint

```text
Implement the selected ground RC input and air-to-FC output protocol using
the dedicated RC_CONTROL transport channel. Add sequence, age, failsafe,
mapping, and bench diagnostics. RC must remain responsive during MAVLink
traffic. Do not alter mission handling except where explicit channel
reservation is required.
```

#### Agent F: Link statistics and recovery

```text
Implement measured link statistics and the STARTING/SEARCHING/SYNCHRONIZED/
DEGRADED/LOST/RECOVERING state machine. Test power loss, RF interruption,
reset, stale queues, and reconnection. Do not fabricate RSSI or link quality.
```

### 8.3 Review loop with this senior review process

After each agent task:

1. Review the agent's diff, not only its summary.
2. Check that it stayed within one phase.
3. Run the relevant host tests/build.
4. Perform a protocol review for ordering, timeouts, wraparound, queue
   starvation, and failsafe behavior.
5. Send the changed files and test output for senior review.
6. Do not start the next phase if the current acceptance gate is not met.

Suggested review request:

```text
Review the current Dual-LRS diff against approach.md as a senior RF/UAV
systems engineer. Check protocol correctness, E22 UART-modem timing,
half-duplex collisions, MAVLink frame integrity, mission ordering, RC
failsafe behavior, queue starvation, reset/reconnection handling, and test
coverage. Do not rewrite code yet. Report findings by severity with exact
file/line references and state which acceptance gate is blocked.
```

### 8.4 Required agent report format

Every agent must finish with:

```text
Phase:
Objective:
Files changed:
Behavior changed:
Tests added:
Commands run:
Results:
Hardware validation performed:
Known limitations:
Risks:
Next blocked or ready phase:
```

---

## 9. Definition of done

Dual-LRS is ready for controlled flight testing only when all of the
following are true:

- The exact hardware configuration is documented.
- The raw E22 link has a repeatable soak test.
- Transport frames have tested CRC and bounds handling.
- RC channels work with correct mapping and measured update rate.
- Air RC output is accepted by ArduPilot.
- RC failsafe and recovery work as designed.
- MAVLink heartbeats work both directions.
- Parameter read/write works repeatedly.
- Mission upload works repeatedly.
- Mission download works repeatedly.
- Commands and mode changes work during telemetry load.
- Telemetry reaches the GCS without corruption.
- Link statistics are based on actual counters.
- Reconnection works after RF and MCU interruptions.
- No debug text contaminates binary MAVLink ports.
- Build and test commands are documented.
- The README claims only behavior that has been measured.

The final system may be simpler than mLRS and still be successful. It must be
predictable, testable, honest about its limits, and safe when the link is
lost.
