# Phase 1 Raw E22 Link Measurement Report
**Date:** 2026-10-01 19:49:51
**Log File:** `audit/phase1_mode_c_1000.jsonl`
**Target Pacing (Mode A/B):** 100 ms (10.0 Hz)
**Target Pacing (Mode C):** 200 ms (5.0 Hz)
**Requested Count per Payload:** 1000 frames

## 3. Mode C: Sequential Ping-Pong RTT Measurements
| Payload (B) | Frame (B) | Req Pings | Sent Pings | Timeouts (Lost) | Valid Pongs | PDR (%) | Pattern Err | CRC Err | Min RTT (ms) | Avg RTT (ms) | Max RTT (ms) | Avg Turnaround (µs) | Approx 1-Way (ms) | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 1000 | 1000 | 0 | 1000 | 100.0% | 0 | 0 | 33.67 | 34.00 | 40.20 | 10 | 17.00 | PASS |
| 10 | 21 | 1000 | 1000 | 0 | 1000 | 100.0% | 0 | 0 | 41.81 | 42.44 | 43.10 | 20 | 21.22 | PASS |
| 30 | 41 | 1000 | 1000 | 0 | 1000 | 100.0% | 0 | 0 | 57.73 | 58.68 | 58.99 | 41 | 29.34 | PASS |
| 53 | 64 | 1000 | 1000 | 0 | 1000 | 100.0% | 0 | 0 | 76.58 | 77.35 | 77.81 | 66 | 38.68 | PASS |
| 54 | 65 | 1000 | 1000 | 2 | 998 | 99.8% | 0 | 0 | 89.63 | 90.62 | 90.85 | 67 | 45.31 | INSUFFICIENT_SAMPLES |
| 55 | 66 | 1000 | 1000 | 4 | 996 | 99.6% | 0 | 0 | 90.53 | 90.86 | 91.76 | 68 | 45.43 | INSUFFICIENT_SAMPLES |

## 4. Hardware Configuration & System Health
- **Air Unit Register Read Status:** `OK`
- **Ground Unit Register Read Status:** `OK`
- **Logging Queue Overflows:** 0 (Total dropped events: 0)
- **Malformed Frames Detected:** 0
- **Malformed Host JSON Lines:** 0
- **Host Serial Status:** `OK`

## 5. Phase 1 Formal Acceptance Assessment
**Gate Status:** `INSUFFICIENT SAMPLES` (Minimum observed: 996/1000 frames per payload size)
> [!IMPORTANT]
> Physical acceptance requires at least 1,000 valid frames per payload size.

> [!NOTE]
> **Acceptance Gate Criteria:** Physical link validation requires $\ge 1,000$ frames per payload size collected on physical hardware.
> **Latency Disclaimer:** Mode C RTT = $(t_4 - t_1) - (t_3 - t_2)$. Approximate one-way latency ($	ext{RTT}/2$) assumes symmetric propagation and includes UART buffering and RF transmission time.
> **Timing Disclaimer:** AUX timing measures observed pin transitions, not guaranteed RF airtime.