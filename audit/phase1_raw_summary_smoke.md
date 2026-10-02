# Phase 1 Raw E22 Link Measurement Report
**Date:** 2026-10-01 19:16:03
**Log File:** `audit/phase1_raw_log.jsonl`
**Target Pacing (Mode A/B):** 200 ms (5.0 Hz)
**Target Pacing (Mode C):** 400 ms (2.5 Hz)
**Requested Count per Payload:** 10 frames
## 1. Mode A: Air -> Ground Simplex Measurements
| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 10 | 21 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 30 | 41 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 53 | 64 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 54 | 65 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | NO_VALID_DATA | 0 | INSUFFICIENT_SAMPLES |
| 55 | 66 | 10 | 0 | 0 | 0.0% | 0 | 0 | 0 | 0 | 0 | 0/0 | 0 | NO_VALID_DATA | 0 | INSUFFICIENT_SAMPLES |

## 2. Mode B: Ground -> Air Simplex Measurements
| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 967 | 662 | 10/10 | 93 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 10 | 21 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 1835 | 364 | 10/10 | 94 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 30 | 41 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 3571 | 365 | 10/10 | 94 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 53 | 64 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 5568 | 312 | 10/10 | 94 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 54 | 65 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 5658 | 26972 | 10/10 | 7503 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |
| 55 | 66 | 10 | 10 | 10 | 100.0% | 0 | 0 | 0 | 5742 | 26905 | 10/10 | 7529 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |

## 3. Mode C: Sequential Ping-Pong RTT Measurements
| Payload (B) | Frame (B) | Req Pings | Sent Pings | Timeouts (Lost) | Valid Pongs | PDR (%) | Pattern Err | CRC Err | Min RTT (ms) | Avg RTT (ms) | Max RTT (ms) | Avg Turnaround (µs) | Approx 1-Way (ms) | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |
| 10 | 21 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |
| 30 | 41 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |
| 53 | 64 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |
| 54 | 65 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |
| 55 | 66 | 10 | 10 | 0 | 0 | 0.0% | 0 | 0 | N/A | N/A | N/A | N/A | N/A | INSUFFICIENT_SAMPLES |

## 4. Hardware Configuration & System Health
- **Air Unit Register Read Status:** `OK`
- **Ground Unit Register Read Status:** `OK`
- **Logging Queue Overflows:** 0 (Total dropped events: 0)
- **Malformed Frames Detected:** 0
- **Malformed Host JSON Lines:** 0
- **Host Serial Status:** `OK`

## 5. Phase 1 Formal Acceptance Assessment
**Gate Status:** `INSUFFICIENT SAMPLES` (Minimum observed: 0/1000 frames per payload size)
> [!IMPORTANT]
> Physical acceptance requires at least 1,000 valid frames per payload size.

> [!NOTE]
> **Acceptance Gate Criteria:** Physical link validation requires $\ge 1,000$ frames per payload size collected on physical hardware.
> **Latency Disclaimer:** Mode C RTT = $(t_4 - t_1) - (t_3 - t_2)$. Approximate one-way latency ($	ext{RTT}/2$) assumes symmetric propagation and includes UART buffering and RF transmission time.
> **Timing Disclaimer:** AUX timing measures observed pin transitions, not guaranteed RF airtime.