#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"
#include "tdm_engine.h"
#include "mavlink_handler.h"

// Map to hardware USART instances configured by ENABLE_HWSERIALx
// USART1: PA9 (TX) and PA10 (RX) connected to Ebyte E22-900T30D
#define SerialRadio Serial1

// USART2: PA2 (TX) and PA3 (RX) connected to ArduPilot FC TELEM (Air)
#define SerialTELEM Serial2

// Drivers and Handlers
E22Driver radio(SerialRadio, PIN_RADIO_M0, PIN_RADIO_M1, PIN_RADIO_AUX);

#if defined(DUAL_LRS_ROLE_AIR)
    const NodeRole CURRENT_ROLE = NodeRole::AIR;
    // Air unit interfaces with Flight Controller via Hardware Serial (USART2)
    MavlinkHandler telemHandler(SerialTELEM);
#else
    const NodeRole CURRENT_ROLE = NodeRole::GROUND;
    // Ground unit interfaces with Mission Planner via USB CDC (or SerialTELEM if USB not active)
    #if defined(USBCON)
        MavlinkHandler telemHandler(Serial);
    #else
        MavlinkHandler telemHandler(SerialTELEM);
    #endif
#endif

TdmEngine tdm(radio, CURRENT_ROLE);

// Buffer for packing RF transmission
static uint8_t rfTxBuffer[MAX_PAYLOAD_PER_SLOT];
static uint32_t lastLedToggleMs = 0;
static uint32_t lastStatusInjectMs = 0;
static uint32_t lastBeaconMs = 0;
static bool sentThisSlot = false;

// Callback when packet is received from RF
void onRadioPacketReceived(LrsPacketType type, const uint8_t* payload, uint8_t length) {
    if (type == LrsPacketType::MAVLINK_DATA && length > 0) {
        telemHandler.writeToLocal(payload, length);
    }

    // Toggle LED to indicate active RF reception
    digitalWrite(PIN_LED_SYNC, !digitalRead(PIN_LED_SYNC));
}

void setup() {
    pinMode(PIN_LED_SYNC, OUTPUT);
    digitalWrite(PIN_LED_SYNC, HIGH); // LED off (active low)

    #if defined(DUAL_LRS_ROLE_GROUND) && defined(USBCON)
        Serial.begin(GCS_USB_BAUD);
    #endif

    SerialTELEM.begin(FC_UART_BAUD);

    tdm.onPacketReceived(onRadioPacketReceived);
    tdm.begin();
    telemHandler.begin();
}

void loop() {
    uint32_t now = millis();

    // 1. Read bytes from local MAVLink stream (FC or Mission Planner)
    telemHandler.readFromLocal();

    // 2. Update TDM state machine and process incoming radio frames
    tdm.update();

    // 3. If it is our turn in the TDM slot and radio is ready, transmit queued data
    if (tdm.canTransmit()) {
        if (!sentThisSlot) {
            size_t bytesToSend = telemHandler.getOutboundPayload(rfTxBuffer, sizeof(rfTxBuffer));
            if (bytesToSend > 0) {
                tdm.sendPacket(LrsPacketType::MAVLINK_DATA, rfTxBuffer, (uint8_t)bytesToSend);
                sentThisSlot = true;
            } else if (now - lastBeaconMs >= TDM_FRAME_PERIOD_MS) {
                // Send one sync beacon per frame when idle (not every loop iteration)
                tdm.sendPacket(LrsPacketType::HEARTBEAT_SYNC, nullptr, 0);
                lastBeaconMs = now;
                sentThisSlot = true;
            }
        }
    } else {
        // Reset the flag when we leave our transmit slot
        sentThisSlot = false;
    }

    // 4. Inject standard MAVLink RADIO_STATUS packet to local GCS/FC (1 Hz)
    if (now - lastStatusInjectMs >= 1000) {
        lastStatusInjectMs = now;
        const LinkStats& stats = tdm.getStats();

        // Calculate link quality percentage
        uint8_t quality = 0;
        if (stats.synchronized) {
            uint32_t totalExpected = stats.packets_received + stats.packets_dropped;
            if (totalExpected > 0) {
                quality = (uint8_t)((stats.packets_received * 100) / totalExpected);
            } else {
                quality = 100;
            }
        }

        // Map quality (0-100) to MAVLink RSSI (0-255)
        uint8_t mavRssi = (uint8_t)((quality * 255) / 100);
        uint8_t txBufPct = (uint8_t)((telemHandler.pendingBytes() * 100) / TELEM_BUFFER_SIZE);

        telemHandler.injectRadioStatus(mavRssi, mavRssi, 100 - txBufPct, (uint16_t)stats.packets_dropped);
    }

    // 5. Unsynchronized status blink indicator
    if (!tdm.getStats().synchronized && (now - lastLedToggleMs >= 500)) {
        lastLedToggleMs = now;
        digitalWrite(PIN_LED_SYNC, !digitalRead(PIN_LED_SYNC));
    }
}
