# Phase 1 Raw E22 Link Measurement Report
**Date:** 2026-10-01 19:30:22
**Log File:** `audit/phase1_mode_a_1000.jsonl`
**Target Pacing (Mode A/B):** 100 ms (10.0 Hz)
**Target Pacing (Mode C):** 200 ms (5.0 Hz)
**Requested Count per Payload:** 1000 frames

## 1. Mode A: Air -> Ground Simplex Measurements
| Payload (B) | Frame (B) | Req Count | TX Sent | Recv Valid | PDR (%) | CRC Err | Pattern Err | Malformed | Avg UART (µs) | Avg AUX Busy (µs) | AUX Edge Obs | Max Inter-Byte (µs) | Split Diagnosis | Overruns | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 11 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 967 | 363 | 1000/1000 | 41 | SINGLE_PACKET | 0 | PASS |
| 10 | 21 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 1836 | 360 | 1000/1000 | 41 | SINGLE_PACKET | 0 | PASS |
| 30 | 41 | 1000 | 1000 | 999 | 99.9% | 0 | 0 | 0 | 3573 | 360 | 1000/1000 | 40 | SINGLE_PACKET | 0 | INSUFFICIENT_SAMPLES |
| 53 | 64 | 1000 | 1000 | 1000 | 100.0% | 0 | 0 | 0 | 5571 | 360 | 1000/1000 | 40 | SINGLE_PACKET | 0 | PASS |
| 54 | 65 | 1000 | 1000 | 998 | 99.8% | 0 | 0 | 0 | 5658 | 27004 | 1000/1000 | 5799 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |
| 55 | 66 | 1000 | 1000 | 999 | 99.9% | 0 | 0 | 0 | 5745 | 26913 | 1000/1000 | 6826 | MODEM_SPLIT_DETECTED (2.0 chunks) | 0 | INSUFFICIENT_SAMPLES |

## 4. Hardware Configuration & System Health
- **Air Unit Register Read Status:** `OK`
- **Ground Unit Register Read Status:** `OK`
- **Logging Queue Overflows:** 0 (Total dropped events: 0)
- **Malformed Frames Detected:** 0
- **Malformed Host JSON Lines:** 0
- **Host Serial Status:** `OK`

## 5. Phase 1 Formal Acceptance Assessment
**Gate Status:** `INSUFFICIENT SAMPLES` (Minimum observed: 998/1000 frames per payload size)
> [!IMPORTANT]
> Physical acceptance requires at least 1,000 valid frames per payload size.

> [!NOTE]
> **Acceptance Gate Criteria:** Physical link validation requires $\ge 1,000$ frames per payload size collected on physical hardware.
> **Latency Disclaimer:** Mode C RTT = $(t_4 - t_1) - (t_3 - t_2)$. Approximate one-way latency ($	ext{RTT}/2$) assumes symmetric propagation and includes UART buffering and RF transmission time.
> **Timing Disclaimer:** AUX timing measures observed pin transitions, not guaranteed RF airtime.