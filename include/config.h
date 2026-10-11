#pragma once
#include <Arduino.h>

// =============================================================================
// DUAL-LRS SYSTEM CONFIGURATION
// Target Hardware: STM32F411CEU6 ("BlackPill") / ESP32 + Ebyte E22-900T30D
// =============================================================================

// Define Role if not passed via build flags
#if !defined(DUAL_LRS_ROLE_AIR) && !defined(DUAL_LRS_ROLE_GROUND)
    #define DUAL_LRS_ROLE_GROUND 1 // Default fallback
#endif

#if defined(ESP32)
// --- Pin Definitions (ESP32-WROOM-32 Ground Unit) ---
#define PIN_LED_BUILTIN    2    // Onboard Blue LED (Active HIGH on ESP32)
#define PIN_USER_KEY       0    // Onboard BOOT button (Active LOW)

// Radio UART (HardwareSerial Serial2 on ESP32)
#define PIN_RADIO_TX       17   // ESP32 TX2 -> E22 RXD
#define PIN_RADIO_RX       16   // ESP32 RX2 <- E22 TXD

// Radio Control Pins
#define PIN_RADIO_M0       21   // E22 Mode 0
#define PIN_RADIO_M1       22   // E22 Mode 1
#define PIN_RADIO_AUX      19   // E22 AUX (Busy line: LOW = Busy, HIGH = Idle)

// Diagnostic LEDs
#define PIN_LED_SYNC       2    // Link Sync Status
#define PIN_LED_TXRX       4    // Active RF activity indicator (optional)

#define LED_PIN_ON         HIGH
#define LED_PIN_OFF        LOW

// --- Wi-Fi Telemetry Settings (ESP32 Ground Unit only - Preserved for Future) ---
#define ENABLE_WIFI_TELEMETRY    0
#define WIFI_STA_SSID            "DISABLED_PLACEHOLDER_SSID"
#define WIFI_STA_PASS            "DISABLED_PLACEHOLDER_PASSWORD"
#define WIFI_AP_SSID             "Dual-LRS-Ground"
#define WIFI_AP_PASS             "DISABLED_PLACEHOLDER_PASSWORD"
#define WIFI_UDP_PORT            14550
#define WIFI_CONNECT_TIMEOUT_MS  4000   // 4s timeout before auto-fallback to AP mode

#else
// --- Pin Definitions (STM32F411CE BlackPill) ---
#define PIN_LED_BUILTIN    PC13 // Onboard Blue LED (Active LOW on BlackPill)
#define PIN_USER_KEY       PA0  // Onboard user button

// Radio UART (HardwareSerial USART1)
#define PIN_RADIO_TX       PA9  // BlackPill TX -> E22 RXD
#define PIN_RADIO_RX       PA10 // BlackPill RX <- E22 TXD

// Radio Control Pins
#define PIN_RADIO_M0       PB0  // E22 Mode 0
#define PIN_RADIO_M1       PB1  // E22 Mode 1
#define PIN_RADIO_AUX      PB10 // E22 AUX (Busy line: LOW = Busy, HIGH = Idle)

// Flight Controller / External Telemetry UART (HardwareSerial USART2)
// Used on Air unit to connect to Prometheus FC TELEM port
#define PIN_FC_TX          PA2  // BlackPill TX -> FC RX (TELEM)
#define PIN_FC_RX          PA3  // BlackPill RX <- FC TX (TELEM)

// Optional Diagnostic LEDs
#define PIN_LED_SYNC       PC13 // Link Sync Status
#define PIN_LED_TXRX       PA1  // Active RF activity indicator (optional external LED)

#define LED_PIN_ON         LOW
#define LED_PIN_OFF        HIGH
#endif

// --- Stage 3.2 Full MAVLink Telemetry Mode ---
// When enabled (0), enables bidirectional MAVLink RF telemetry between FC and GCS.
// Air requests streams from ArduPilot, fragments packets into 49B frames, and streams down to Ground.
// Ground reconstructs packets and outputs to Mission Planner / QGC over USB CDC / UART.
#ifndef DUAL_LRS_STAGE31_RC_ONLY
#define DUAL_LRS_STAGE31_RC_ONLY  0
#endif

// --- Flight Controller Telemetry UART (PA2/PA3 strictly reserved) ---
// PA2 (TX) and PA3 (RX) map to STM32F411 USART2. Must NOT be repurposed for RC.

// --- Air Node CRSF Output Pinout (To Flight Controller RC_IN) ---
// Hardware Note: BlackPill F411CE (UFQFPN48 48-pin) exposes hardware USART6 on PA11 (TX) and PA12 (RX).
// PC6 and PC7 do NOT exist on the 48-pin F411CE package (omitted from pinout and Arduino pinmap).
// PA11 connects to Flight Controller RC_IN. PA12 is available for bidirectional CRSF telemetry.
#define PIN_AIR_CRSF_TX           PA11    // BlackPill TX -> FC RC_IN (USART6 TX)
#define PIN_AIR_CRSF_RX           PA12    // BlackPill RX <- FC CRSF Telemetry (USART6 RX)
#define AIR_CRSF_BAUD             420000  // Standard CRSF protocol baud rate

// --- Ground Node RC Handset Ingest (ESP32 UART1) ---
// Handset connected to ESP32 UART1 with 400,000 baud inverted signal (OpenI6X CRSF / S.BUS)
#define PIN_GROUND_RC_RX          13      // ESP32 RX1 <- Handset TX
#define PIN_GROUND_RC_TX          14      // ESP32 TX1 -> Handset RX
#define GROUND_RC_BAUD            400000  // OpenI6X handset baud rate
#define GROUND_RC_INVERTED        1       // Inversion enabled for OpenI6X

// --- Baud Rate Settings ---
#define RADIO_UART_BAUD    115200  // High-speed UART between MCU and E22
#define FC_UART_BAUD       115200  // ArduPilot TELEM default baud rate
#define GCS_USB_BAUD       115200  // USB Virtual COM Port to Mission Planner

// --- Ebyte E22 Transmit Power Settings ---
// For E22-900T30D REG1 bits [1:0]:
//   00 = 30 dBm (1000 mW / 1 W)  - Maximum power (Flight range)
//   01 = 27 dBm (500 mW)
//   10 = 24 dBm (250 mW)
//   11 = 21 dBm (125 mW)         - Minimum power (Bench testing < 2m separation)
#define E22_TX_POWER_MIN       3  // Register bits 11 = 21 dBm (125 mW)
#define E22_TX_POWER_BENCH     3  // Register bits 11 = 21 dBm (125 mW) - prevents receiver LNA saturation
#define E22_TX_POWER_FLIGHT    0  // Register bits 00 = 30 dBm (1000 mW) - maximum range for flight
#define E22_ACTIVE_TX_POWER    E22_TX_POWER_BENCH // Active power: change to E22_TX_POWER_FLIGHT for real flight

// --- TDM Protocol Schedule Profiles ---
// Profile 0: 90.0 ms (11.1 Hz) - Telemetry Heavy (Phase 2 Legacy Baseline)
// Profile 1: 70.0 ms (14.3 Hz) - Balanced (32ms Ground, 5ms Gap1, 25ms Air, 8ms Gap2)
// Profile 2: 50.0 ms (20.0 Hz) - RC-Fast Sweet Spot (32ms Ground, 4ms Gap1, 10ms Air, 4ms Gap2)
#define TDM_PROFILE_90MS_11HZ   0
#define TDM_PROFILE_70MS_14HZ   1
#define TDM_PROFILE_50MS_20HZ   2

#ifndef DUAL_LRS_TDM_PROFILE
// The 25 ms Air slot in the 70 ms profile is shorter than the measured
// on-air/transparent-UART completion time of a maximum 64-byte transport
// frame. Keep the conservative 90 ms profile as the safe default until an
// airtime-aware scheduler is implemented and measured on the deployed radios.
#define DUAL_LRS_TDM_PROFILE    TDM_PROFILE_90MS_11HZ
#endif

#if (DUAL_LRS_TDM_PROFILE == TDM_PROFILE_50MS_20HZ)
    #define TDM_FRAME_PERIOD_MS    50  // Total period: 50 ms (20.0 Hz fresh RC update rate)
    #define TDM_GROUND_SLOT_MS     32  // Slot 1: Ground -> Air RC slot (0ms - 32ms, finishes in ~28.5ms)
    #define TDM_GUARD_GAP1_MS       4  // Turnaround guard delay 1 (32ms - 36ms)
    #define TDM_AIR_SLOT_MS        10  // Slot 2: Air sync / link quality beacon slot (36ms - 46ms)
    #define TDM_GUARD_GAP2_MS       4  // Turnaround guard delay 2 (46ms - 50ms)
#elif (DUAL_LRS_TDM_PROFILE == TDM_PROFILE_70MS_14HZ)
    #define TDM_FRAME_PERIOD_MS    70  // Total period: 70 ms (14.3 Hz)
    #define TDM_GROUND_SLOT_MS     32  // Slot 1: Ground -> Air RC slot (0ms - 32ms)
    #define TDM_GUARD_GAP1_MS       5  // Turnaround guard delay 1 (32ms - 37ms)
    #define TDM_AIR_SLOT_MS        25  // Slot 2: Air telemetry slot (37ms - 62ms)
    #define TDM_GUARD_GAP2_MS       8  // Turnaround guard delay 2 (62ms - 70ms)
#else
    #define TDM_FRAME_PERIOD_MS    90  // Total period: 90 ms (11.1 Hz)
    #define TDM_GROUND_SLOT_MS     32  // Slot 1: Ground -> Air RC slot (0ms - 32ms)
    #define TDM_GUARD_GAP1_MS       5  // Turnaround guard delay 1 (32ms - 37ms)
    #define TDM_AIR_SLOT_MS        45  // Slot 2: Air telemetry slot (37ms - 82ms)
    #define TDM_GUARD_GAP2_MS       8  // Turnaround guard delay 2 (82ms - 90ms)
#endif

// Safety policy: never replace a fresh RC frame with Ground-to-Air MAVLink.
// MAVLink uplink may use a Ground slot only when no fresh handset frame exists.
#ifndef DUAL_LRS_RESERVE_RC_EVERY_CYCLE
#define DUAL_LRS_RESERVE_RC_EVERY_CYCLE 1
#endif

// Buffer Sizes & Single-Burst Framing Bounds (E22 64-byte Subpacket Boundary)
#define RADIO_BUFFER_SIZE         1024
#if defined(DUAL_LRS_ROLE_AIR)
#define TELEM_BUFFER_SIZE         49152 // 48KB FIFO buffer: absorbs entire ArduPilot parameter table (~38KB) without dropping any packets
#else
#define TELEM_BUFFER_SIZE         1024  // 1KB FIFO buffer on Ground: bounds latency to < 10 seconds under worst-case storm
#endif
#define MAX_PAYLOAD_AIR_SLOT      49  // Max single-burst payload (64 - 13 - 2 = 49)
#define MAX_PAYLOAD_GROUND_SLOT   49  // Max single-burst payload (RC uses 24B payload, 39B frame)
#define MAX_PAYLOAD_PER_SLOT      49  // Single-burst limit
#define MAX_FRAME_PER_SLOT        64  // 13B Header + 49B Payload + 2B CRC = 64B

// Protocol Magic Bytes (Matches Phase 2 TransportHeader: 'D', 'L')
#define DUAL_LRS_MAGIC_0      0x44 // 'D'
#define DUAL_LRS_MAGIC_1      0x4C // 'L'
