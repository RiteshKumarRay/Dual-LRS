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
#define WIFI_STA_SSID            "Ritesh S22+"
#define WIFI_STA_PASS            "1234554321"
#define WIFI_AP_SSID             "Dual-LRS-Ground"
#define WIFI_AP_PASS             "duallrs123"
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

// --- Baud Rate Settings ---
#define RADIO_UART_BAUD    115200  // High-speed UART between BlackPill and E22
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

// --- TDM Protocol Timings (Time Division Multiplexing) ---
// Total Frame = 50ms (20 Hz cycle rate)
#define TDM_FRAME_PERIOD_MS    50  // Total period of 1 TDM frame in milliseconds (STRICT: NEVER CHANGE)
#define TDM_AIR_SLOT_MS        31  // Air -> Ground telemetry slot (0ms - 31ms): accommodates full 40B MAVLink payload
#define TDM_GUARD_GAP1_MS       4  // Turnaround guard delay 1 (31ms - 35ms): 4ms margin allows Air RF + Ground UART flush
#define TDM_GROUND_SLOT_MS     12  // Ground -> Air commands & RC slot (35ms - 47ms)
#define TDM_GUARD_GAP2_MS       3  // Turnaround guard delay 2 (47ms - 50ms): allows Ground RF + Air UART flush

// Buffer Sizes
#define RADIO_BUFFER_SIZE         1024
#if defined(DUAL_LRS_ROLE_AIR)
#define TELEM_BUFFER_SIZE         49152 // 48KB FIFO buffer: absorbs entire ArduPilot parameter table (~38KB) without dropping any packets
#else
#define TELEM_BUFFER_SIZE         1024  // 1KB FIFO buffer on Ground: bounds latency to < 10 seconds under worst-case storm
#endif
#define MAX_PAYLOAD_AIR_SLOT      40  // 1 full MAVLink frame per slot (up to 40B): fits in 31ms air slot
#define MAX_PAYLOAD_GROUND_SLOT   10  // 10B: 17B on-air frame finishes in ~5.2ms (by 40.2ms), leaving 6.8ms safety margin before 47ms slot end
#define MAX_PAYLOAD_PER_SLOT      64 // Frame buffer allocation (5B header + 40B payload + 2B CRC = 47B)

// Protocol Magic Bytes
#define DUAL_LRS_MAGIC_0      0x44 // 'D'
#define DUAL_LRS_MAGIC_1      0x4C // 'L'
