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

- Mission upload and mission download were live bench verified with the
  propellers removed.
- Full parameter download was live bench verified.
- Background telemetry is suppressed while mission or parameter transfers are
  active; mission/parameter protocol messages remain prioritized.
- Normal-mode telemetry aggregation is implemented, but only complete MAVLink
  frames that fit together within the 49-byte transport payload can be
  combined. Large common messages such as `ATTITUDE`, `VFR_HUD`, GPS and
  position messages often still require separate transport opportunities.
- The Air slot still sends at most one RF transport frame per TDM cycle.
- Current production transfers remain best-effort at the transport layer;
  production ACK/NACK generation and retry-manager integration are not yet a
  complete loss-recovery guarantee.

## Host and build verification

The current worker changes have passed:

- RC adapter suite: 7 tests.
- Transport engine suite: 10 tests.
- Production RC dispatch suite: 5 tests.
- Production MAVLink transport suite: 5 tests.
- MAVLink telemetry gating and aggregation suite.
- `git diff --check`.
- PlatformIO targets: `dual_lrs_air`, `dual_lrs_ground`, and
  `dual_lrs_ground_esp32`.

## Remaining acceptance gates

- Intentional RF cut-and-restore with props removed, including FC-recognized
  failsafe action and recovery.
- Repeated mission and parameter runs under clean and impaired RF conditions.
- Production ACK/NACK/retry integration and packet-loss recovery.
- Queue/backpressure behavior under saturated MAVLink traffic.
- Flight-controller-specific failsafe parameter confirmation.
- No flight authorization has been granted by this document.

## Documentation policy

- `README.md`, this page, `docs/TDM_PROTOCOL.md`, and
  `audit/STAGE_3_2_STATUS.md` are current-facing references.
- `audit/PHASE_*`, raw Phase 1 measurement files, and dated maintenance/audit
  reports are historical evidence. They should be annotated when necessary,
  not rewritten to make old results look current.
- `context.md`, `audit.md`, build output, graph output, calibration captures,
  and Python caches are local/generated notes and are ignored by Git.
