# Phase 1 Raw E22 Link Measurement Report
**Date:** 2026-10-01 19:40:47
**Log File:** `audit/phase1_mode_b_1000.jsonl`
**Target Pacing (Mode A/B):** 100 ms (10.0 Hz)
**Target Pacing (Mode C):** 200 ms (5.0 Hz)
**Requested Count per Payload:** 1000 frames

## 2. Mode B: Ground -> Air Simplex Measurements
| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 967 | 368 | 1000/1000 | 95 | SINGLE_PACKET | 0 | PASS |
| 10 | 21 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 1835 | 364 | 1000/1000 | 95 | SINGLE_PACKET | 0 | PASS |
| 30 | 41 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 3571 | 365 | 1000/1000 | 95 | SINGLE_PACKET | 0 | PASS |
| 53 | 64 | 1000 | 1000 | 999 | 99.9% | 0 | 0 | 0 | 5568 | 312 | 1000/1000 | 95 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 54 | 65 | 1000 | 1000 | 999 | 99.9% | 0 | 0 | 0 | 5655 | 26963 | 1000/1000 | 7542 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |
| 55 | 66 | 1000 | 1000 | 997 | 99.7% | 0 | 0 | 0 | 5741 | 26904 | 1000/1000 | 8487 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |

## 4. Hardware Configuration & System Health
- **Air Unit Register Read Status:** `OK`
- **Ground Unit Register Read Status:** `OK`
- **Logging Queue Overflows:** 0 (Total dropped events: 0)
- **Malformed Frames Detected:** 0
- **Malformed Host JSON Lines:** 0
- **Host Serial Status:** `OK`

## 5. Phase 1 Formal Acceptance Assessment
**Gate Status:** `INSUFFICIENT SAMPLES` (Minimum observed: 997/1000 frames per payload size)
> [!IMPORTANT]
> Physical acceptance requires at least 1,000 valid frames per payload size.

> [!NOTE]
> **Acceptance Gate Criteria:** Physical link validation requires $\ge 1,000$ frames per payload size collected on physical hardware.
> **Latency Disclaimer:** Mode C RTT = $(t_4 - t_1) - (t_3 - t_2)$. Approximate one-way latency ($	ext{RTT}/2$) assumes symmetric propagation and includes UART buffering and RF transmission time.
> **Timing Disclaimer:** AUX timing measures observed pin transitions, not guaranteed RF airtime.