# Stage 3.2 MAVLink Transport Status

## Scope

This report records the current implementation and verification state of the
Dual-LRS Stage 3.2 development mode. It does not authorize flight testing and
does not claim that missions, parameters, or commands are production-reliable.

## Verified

- Stage 3.2 remains opt-in. The default production configuration keeps
  `DUAL_LRS_STAGE31_RC_ONLY=1`.
- The native `test_production_mavlink_transport` target explicitly builds with
  `DUAL_LRS_STAGE31_RC_ONLY=0`.
- A single-burst MAVLink packet is transported and delivered without mutation.
- A 128-byte MAVLink packet is fragmented into three transport fragments and
  reassembled bit-for-bit.
- A ground-to-air command payload is fragmented and reassembled bit-for-bit.
- RC and MAVLink coexist in the host TDM simulation without RC starvation
  (seven RC frames and one command transfer in eight cycles).
- Oversized single-burst payloads are rejected rather than silently truncated.
- RC transmission is not disabled merely because Stage 3.2 MAVLink mode is
  enabled.
- The production firmware and diagnostic build matrix compiles:
  `dual_lrs_air`, `dual_lrs_ground`, `dual_lrs_ground_esp32`, `diag_air`,
  `diag_ground_esp32`, `test_air_crsf_uart`, and
  `test_production_mavlink_transport`.
- Existing transport, CRSF, and production RC dispatch host tests pass:
  Python transport tests (25/25), transport protocol tests, transport-engine
  tests, CRSF tests including the 4,096-case CH1/CH2 independence sweep, and
  all production RC dispatch tests.

## Important interpretation

The Stage 3.2 test currently exercises the TDM and transport objects with
simulated radio delivery. It does not yet execute the complete firmware
`main.cpp` loop with real UART queues, E22 AUX timing, modem buffering, or a
flight controller. Passing this test proves framing and basic coexistence, not
end-to-end MAVLink service reliability.

The production RC RF rate is approximately 11.1 Hz because the approved TDM
cycle is 90 ms. The handset input rate can be higher, but it is not the same
as the RF RC update rate. The production channel path also intentionally
remaps handset CH6 to FC CH1 and parks FC CH6 at neutral; it is not a
transparent channel-number bridge.

## Remaining blockers before enabling full MAVLink in production

1. Integrate ACK/NACK generation into production receive and transmit paths.
2. Integrate `TransportRetryManager` into the production scheduler.
3. Define and test best-effort versus reliable traffic classes. Telemetry may
   be best-effort; commands, parameters, and mission transfers require
   completion acknowledgement and retry behavior.
4. Add production-path tests for packet loss, duplicates, overlap conflicts,
   gaps, timeout, retry exhaustion, and reconnection.
5. Add parameter read/write, command acknowledgement, and mission
   upload/download tests against a real or faithful flight-controller endpoint.
6. Test queue backpressure and confirm RC latency and failsafe behavior while
   MAVLink traffic is saturated.
7. Validate the approximately 11.1 Hz RC input rate and FC failsafe settings
   on the target flight controller.
8. Repeat all tests with real E22 modules and then perform controlled,
   propellers-off bench tests before any flight.

Until these blockers are closed, Stage 3.2 should remain a development/test
configuration and the default RC-only mode should not be changed.

## RC rate interpretation

The current 90 ms TDM cycle yields approximately 11.1 fresh RC updates per
second. This is a schedule limit, not the maximum capability of the MCU or
the 420,000-baud CRSF UART. The E22 modem's approximately 28–30 ms transfer
time for a packed RC frame, half-duplex turnaround, guard intervals, and the
need to retain Air-to-Ground telemetry determine the practical limit.

The engineering target after reliability and physical modem testing is
approximately 15–20 Hz with telemetry, or approximately 20–30 Hz in an
RC-priority mode with reduced telemetry. A 50–100 Hz fresh RF RC rate is not a
realistic target for this UART modem architecture. The Air unit must not repeat
stale RC frames merely to create a higher apparent CRSF output rate.
