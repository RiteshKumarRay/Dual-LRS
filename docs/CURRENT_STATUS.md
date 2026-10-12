# Dual-LRS Current Status

**Status date:** 2026-10-11
**Status:** Bench-verified development system; flight acceptance remains blocked.

This is the current-state reference for the repository. Historical audit and
phase documents preserve the facts and claims that were true when those tests
were run; they are not replacements for this page.

## Verified architecture

- Air unit: STM32F411 BlackPill.
- Ground unit: ESP32-WROOM-32 or STM32F411 BlackPill.
- Radio: Ebyte E22-900T30D UART LoRa modem at each end.
- Application protocols: CRSF RC input/output and MAVLink v1/v2 telemetry.
- Transport: explicit 13-byte header, CRC-16, maximum 49-byte payload and
  maximum 64-byte transport frame.
- Default TDM schedule: 90 ms total cycle:
  - Ground transmit: 0–32 ms.
  - Guard 1: 32–37 ms.
  - Air transmit: 37–82 ms.
  - Guard 2: 82–90 ms.

The 90 ms profile is intentional. A measured maximum transport-frame
completion time is approximately 38.68 ms, leaving approximately 6.32 ms of
margin in the 45 ms Air slot. The 70 ms and 50 ms profiles must not be enabled
for the current deployed modem timing without a new measured timing review.

## RC status

- FS-i6X/OpenI6X CRSF input is ingested on the Ground ESP32.
- Handset CH6 is intentionally routed to FC CH1; FC CH6 is parked at neutral.
- Fresh RC RF delivery is approximately 11.1 Hz because one Ground slot is
  available per 90 ms cycle.
- The Air unit may emit the latest accepted CRSF state to the FC at its local
  50 Hz output pacing; this is not the same as 50 Hz fresh RF input.
- Air output is suppressed during `NO_SIGNAL` and `FAILSAFE`.
- Recovery requires a candidate baseline plus three advancing accepted frames.
- Sequence filtering rejects duplicates and stale/ambiguous modular deltas.
- Live Radxa/ArduPilot bench validation showed RC present/healthy, smooth
  stick response, zero failsafe transitions during continuous transmission,
  and approximately 20 Hz observed FC-side RC output pacing.

## MAVLink status

- Production ACK/NACK and retry state machine (`LinkTransport`) is implemented
  and active on both Air and Ground nodes (stop-and-wait lockstep, 200 ms RTO,
  5-retry limit, strict ACK/NACK identity matching, and duplicate re-ACK
  without duplicate application delivery, including after a consumed message
  loses its final ACK).
- Clean transfer acceptance passed: 10 consecutive mission upload and download
  cycles verified with 100% bit-exact waypoint coordinate integrity.
- Full parameter table download verified: 1004/1004 parameters retrieved with
  100% first-pass completeness over the RF link (9.8 params/sec, 102.1s elapsed,
  zero dropped parameters, zero hole recovery required).
- Background telemetry is suppressed while mission or parameter transfers are
  active; mission/parameter protocol messages remain prioritized. Parameter
  downlink fragments use reliable stop-and-wait delivery and routine telemetry
  resumes only after every distinct parameter index has been delivered.
- Ground scheduler interleave guarantees RC control priority while allocating
  slots to MAVLink transactions (1:2 during active mission/ACK, 1:4 normal).
  Ground falls back to transmitting live RC rather than an empty beacon if no
  MAVLink packet is ready.
- Saturated traffic stress test passed with zero RC failsafes, zero MCU reboots,
  and zero `BAD_DATA` frames.

## Host and build verification

All 8 host test suites pass cleanly (**69 / 69 tests passing, 100% SUCCESS**):

- C++ Transport protocol suite: 5 tests.
- Transport engine suite: 10 tests.
- Production TDM scheduler suite: 7 tests.
- Production RC dispatch suite: 5 tests.
- Controlled RC adapter suite: 7 tests.
- Production MAVLink transport suite: 5 tests.
- MAVLink telemetry gating suite: 10 tests.
- Phase 3 Reliable LinkTransport suite: 20 tests.
- Master test runner: `bash tools/run_all_host_tests.sh`.
- PlatformIO targets: `dual_lrs_air`, `dual_lrs_ground`, and
  `dual_lrs_ground_esp32`.

## Remaining acceptance gates

- Powered-node RF loss (RF shielding/attenuation while both MCUs stay powered).
- Impaired-RF mission/parameter transfer recovery on live link.
- Armed failsafe action / throttle-cut validation with propellers removed.
- Final flight-readiness review and sign-off.
- No flight authorization has been granted by this document.

## Documentation policy

- `README.md`, this page, `docs/TDM_PROTOCOL.md`, and
  `audit/STAGE_3_2_STATUS.md` are current-facing references.
- `audit/PHASE_*`, raw Phase 1 measurement files, and dated maintenance/audit
  reports are historical evidence. They should be annotated when necessary,
  not rewritten to make old results look current.
- `context.md`, `audit.md`, build output, graph output, calibration captures,
  and Python caches are local/generated notes and are ignored by Git.
