// Minimal verification for Air CRSF UART Mapping on BlackPill STM32F411CE
// Validates hardware USART6 (PA11 TX / PA12 RX) at 420,000 baud
// Keeps PA2/PA3 reserved for Flight Controller Telemetry (USART2)

#include <Arduino.h>
#include "config.h"

// Instantiate USART6 on BlackPill F411CE hardware pins
// PA11 = USART6_TX, PA12 = USART6_RX
#if defined(DUAL_LRS_ROLE_AIR)
Uart SerialAirCRSF(PIN_AIR_CRSF_RX, PIN_AIR_CRSF_TX);
#endif

void setup() {
#if defined(DUAL_LRS_ROLE_AIR)
    // 420,000 baud standard CRSF protocol rate
    SerialAirCRSF.begin(AIR_CRSF_BAUD);

    // Test transmit 1 byte
    SerialAirCRSF.write(0xC8); // CRSF sync byte
#endif
}

void loop() {
}
