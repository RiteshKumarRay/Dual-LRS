#pragma once
#include <Arduino.h>

// =============================================================================
// DUAL-LRS SYSTEM CONFIGURATION
// Target Hardware: STM32F411CEU6 ("BlackPill") + Ebyte E22-900T30D
// =============================================================================

// Define Role if not passed via build flags
#if !defined(DUAL_LRS_ROLE_AIR) && !defined(DUAL_LRS_ROLE_GROUND)
    #define DUAL_LRS_ROLE_GROUND 1 // Default fallback
#endif

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

// --- Baud Rate Settings ---
#define RADIO_UART_BAUD    115200  // High-speed UART between BlackPill and E22
#define FC_UART_BAUD       115200  // ArduPilot TELEM default baud rate
#define GCS_USB_BAUD       115200  // USB Virtual COM Port to Mission Planner

// --- TDM Protocol Timings (Time Division Multiplexing) ---
// Total Frame = 50ms (20 Hz cycle rate)
#define TDM_FRAME_PERIOD_MS    50  // Total period of 1 TDM frame in milliseconds
#define TDM_AIR_SLOT_MS        30  // Air -> Ground telemetry slot (60% bandwidth)
#define TDM_GUARD_GAP1_MS       4  // Turnaround guard delay 1
#define TDM_GROUND_SLOT_MS     12  // Ground -> Air commands & RC slot (24% bandwidth)
#define TDM_GUARD_GAP2_MS       4  // Turnaround guard delay 2

// Buffer Sizes
#define RADIO_BUFFER_SIZE     512
#define TELEM_BUFFER_SIZE     1024
#define MAX_PAYLOAD_PER_SLOT  240 // E22 single transmission limit (max 240 bytes)

// Protocol Magic Bytes
#define DUAL_LRS_MAGIC_0      0x44 // 'D'
#define DUAL_LRS_MAGIC_1      0x4C // 'L'
