#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"
#include "tdm_engine.h"
#include "mavlink_handler.h"

#if defined(ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// Map to hardware USART instances
#if defined(ESP32)
    // ESP32 Ground Unit: Serial2 for E22, Serial for USB-to-UART bridge to GCS
    #define SerialRadio Serial2
    const NodeRole CURRENT_ROLE = NodeRole::GROUND;
    MavlinkHandler telemHandler(Serial);
#elif defined(DUAL_LRS_ROLE_AIR)
    #define SerialRadio Serial1
    #define SerialTELEM Serial2
    const NodeRole CURRENT_ROLE = NodeRole::AIR;
    // Air unit interfaces with Flight Controller via Hardware Serial (USART2)
    MavlinkHandler telemHandler(SerialTELEM);
#else
    #define SerialRadio Serial1
    #define SerialTELEM Serial2
    const NodeRole CURRENT_ROLE = NodeRole::GROUND;
    // Ground unit interfaces with QGroundControl / Mission Planner via USB CDC (or SerialTELEM if USB not active)
    #if defined(USBCON)
        MavlinkHandler telemHandler(Serial);
    #else
        MavlinkHandler telemHandler(SerialTELEM);
    #endif
#endif

// Drivers and Handlers
E22Driver radio(SerialRadio, PIN_RADIO_M0, PIN_RADIO_M1, PIN_RADIO_AUX);
TdmEngine tdm(radio, CURRENT_ROLE);

// Buffer for packing RF transmission
static uint8_t rfTxBuffer[MAX_PAYLOAD_PER_SLOT];
static uint32_t lastLedToggleMs   = 0;
static uint32_t lastStatusInjectMs = 0;
static uint32_t lastBeaconMs      = 0;
static bool sentThisSlot          = false;

#if defined(DUAL_LRS_ROLE_AIR)
// Auto-Baud scanner for Flight Controller UART:
// ArduPilot ports can be 57600 (TELEM default), 115200 (Companion/Fast), or 230400 (RCIN).
static const uint32_t FC_BAUDS[] = {115200, 57600, 230400};
static uint8_t fcBaudIdx = 0;
static uint32_t currentFcBaud = 115200;
static uint32_t lastBaudSwitchMs = 0;
static bool fcBaudLocked = false;
#endif

static uint32_t gcsDataPktsRx = 0;

void onRadioPacketReceived(LrsPacketType type, const uint8_t* payload, uint8_t length) {
    if (type == LrsPacketType::MAVLINK_DATA && length > 0) {
        gcsDataPktsRx++;
        // Ground: writeToLocal() tracks frame boundaries for safe RADIO_STATUS injection.
        // Air: writeToLocal() forwards GCS commands to FC UART.
        telemHandler.writeToLocal(payload, length);
    }

    // Active RF reception: turn LED on
    digitalWrite(PIN_LED_SYNC, LED_PIN_ON);
}

#if defined(DUAL_LRS_ROLE_AIR)
// Build and send a REQUEST_DATA_STREAM packet directly to the FC.
// Ensures QGC gets battery, GPS, attitude even before parameter download completes.
static void requestStream(Stream& port, uint8_t streamId, uint16_t rateHz) {
    static uint8_t seq = 0;
    uint8_t packet[14]; // 6 header + 6 payload + 2 CRC
    packet[0] = 0xFE;
    packet[1] = 6;
    packet[2] = seq++;
    packet[3] = 255; // GCS SysID
    packet[4] = 190; // GCS CompID
    packet[5] = 66;  // MAVLINK_MSG_ID_REQUEST_DATA_STREAM

    // Payload: rate (2 bytes LE), target_system, target_component, req_stream_id, start_stop
    packet[6]  = (uint8_t)(rateHz & 0xFF);
    packet[7]  = (uint8_t)((rateHz >> 8) & 0xFF);
    packet[8]  = 1;          // target_system
    packet[9]  = 1;          // target_component
    packet[10] = streamId;
    packet[11] = 1;          // start_stop = 1 (start)

    // CRC_EXTRA for REQUEST_DATA_STREAM (msgid 66) is 148
    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < 12; ++i) {
        uint8_t tmp = packet[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    uint8_t tmp = 148 ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);

    packet[12] = (uint8_t)(crc & 0xFF);
    packet[13] = (uint8_t)((crc >> 8) & 0xFF);

    port.write(packet, sizeof(packet));
}
#endif

void setup() {
    #if defined(ESP32)
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
        Serial.setRxBufferSize(2048);
        Serial2.setRxBufferSize(2048);
    #endif

    pinMode(PIN_LED_SYNC, OUTPUT);
    digitalWrite(PIN_LED_SYNC, LED_PIN_OFF);

    #if defined(ESP32)
        Serial.begin(GCS_USB_BAUD);
    #elif defined(USBCON)
        Serial.begin(GCS_USB_BAUD);
    #endif

    #if defined(SerialTELEM)
        #if defined(DUAL_LRS_ROLE_AIR)
            SerialTELEM.begin(currentFcBaud);
        #else
            SerialTELEM.begin(FC_UART_BAUD);
        #endif
    #endif

    tdm.onPacketReceived(onRadioPacketReceived);
    tdm.begin();
    telemHandler.begin();
}

void loop() {
    uint32_t now = millis();

    // 1. Read bytes from local MAVLink stream (FC or Mission Planner)
    telemHandler.readFromLocal();

    #if defined(DUAL_LRS_ROLE_AIR) && defined(USBCON)
    // USB command handler: ESC (0x1B) resets the BlackPill into DFU bootloader mode.
    // Raw CDC bytes are NOT forwarded to SerialTELEM to prevent PC port probes from corrupting FC stream.
    if (Serial) {
        while (Serial.available() > 0) {
            uint8_t b = (uint8_t)Serial.read();
            if (b == 0x1B) {
                HAL_RCC_DeInit();
                HAL_DeInit();
                SysTick->CTRL = 0;
                SysTick->LOAD = 0;
                SysTick->VAL = 0;
                __disable_irq();
                uint32_t appStack = *(__IO uint32_t*)0x1FFF0000;
                void (*SysMemBootJump)(void) = (void (*)(void))(*(__IO uint32_t*)(0x1FFF0000 + 4));
                __set_MSP(appStack);
                SysMemBootJump();
                while (1);
            }
        }
    }
    #endif

    #if defined(DUAL_LRS_ROLE_AIR)
    // Request telemetry streams from ArduPilot on startup (or after baud switch).
    static uint32_t lastStreamReqMs = 0;
    static uint8_t  reqCount = 0;
    if ((reqCount < 8 || (now - telemHandler.lastValidPacketMs() > 5000)) && (now - lastStreamReqMs >= 2500)) {
        lastStreamReqMs = now;
        if (reqCount < 8) reqCount++;
        requestStream(SerialTELEM, 2,  2); // SYS_STATUS (Battery)    at 2 Hz
        requestStream(SerialTELEM, 6,  2); // POSITION (GPS)           at 2 Hz
        requestStream(SerialTELEM, 1,  2); // EXTRA1 (Attitude)        at 2 Hz
        requestStream(SerialTELEM, 10, 2); // EXTRA2 (VFR_HUD)         at 2 Hz
    }

    // Auto-Baud scanner for Flight Controller UART:
    // Locks when valid Heartbeat packets arrive from ArduPilot.
    // If no valid Heartbeat for 4.0s, cycle to the next baud rate (115200 -> 57600 -> 230400).
    if (telemHandler.validHeartbeats() > 0 && (now - telemHandler.lastHeartbeatMs() < 5000)) {
        fcBaudLocked = true;
    } else {
        fcBaudLocked = false;
        if (now - lastBaudSwitchMs >= 4000) {
            lastBaudSwitchMs = now;
            fcBaudIdx = (fcBaudIdx + 1) % (sizeof(FC_BAUDS) / sizeof(FC_BAUDS[0]));
            currentFcBaud = FC_BAUDS[fcBaudIdx];
            SerialTELEM.end();
            SerialTELEM.begin(currentFcBaud);
            reqCount = 0; // re-request streams on the new baud rate
        }
    }
    #endif

    // 2. Update TDM state machine and process incoming radio frames
    tdm.update();

    // 3. Transmit queued data when it is our TDM slot
    //
    // FIX: We track the slot directly (getCurrentSlot) in addition to canTransmit().
    // sentThisSlot resets as soon as we leave the AIR_TRANSMIT or GROUND_TRANSMIT slot,
    // so we don't try to send twice per slot.
    TdmSlot slot = tdm.getCurrentSlot();
    bool inMySlot = (CURRENT_ROLE == NodeRole::AIR)
                        ? (slot == TdmSlot::AIR_TRANSMIT)
                        : (slot == TdmSlot::GROUND_TRANSMIT);

    static uint32_t gcsMavlinkRfSent = 0;
    static uint8_t pendingBytesToSend = 0;
    static bool hasPendingTx = false;

    static uint32_t lastTxStartUs = 0;
    static uint32_t lastTxAuxUs = 0;
    static uint32_t maxTxAuxUs = 0;
    static bool trackingTxAux = false;

    if (trackingTxAux && !radio.isBusy()) {
        lastTxAuxUs = micros() - lastTxStartUs;
        if (lastTxAuxUs > maxTxAuxUs) maxTxAuxUs = lastTxAuxUs;
        trackingTxAux = false;
    }

    if (inMySlot) {
        if (!sentThisSlot) {
            size_t maxBytes = (CURRENT_ROLE == NodeRole::GROUND)
                                ? MAX_PAYLOAD_GROUND_SLOT
                                : MAX_PAYLOAD_AIR_SLOT;

            if (!hasPendingTx) {
                size_t n = telemHandler.getOutboundPayload(rfTxBuffer, maxBytes);
                pendingBytesToSend = (uint8_t)n;
                hasPendingTx = true;
            }

            bool ok = false;
            lastTxStartUs = micros();
            if (pendingBytesToSend > 0) {
#if defined(DUAL_LRS_ROLE_AIR) && defined(USBCON)
                if (Serial && rfTxBuffer[0] == 0xFD && pendingBytesToSend >= 10) {
                    uint32_t mid = rfTxBuffer[7] | ((uint32_t)rfTxBuffer[8] << 8) | ((uint32_t)rfTxBuffer[9] << 16);
                    if (mid == 22) {
                        Serial.printf("[AIR_TX_PARAM] Sent PARAM_VALUE over RF (%u B)\n", (unsigned int)pendingBytesToSend);
                    }
                }
#endif
                ok = tdm.sendPacket(LrsPacketType::MAVLINK_DATA, rfTxBuffer, pendingBytesToSend);
                if (ok) gcsMavlinkRfSent++;
            } else {
                ok = tdm.sendPacket(LrsPacketType::HEARTBEAT_SYNC, nullptr, 0);
            }
            if (ok) {
                sentThisSlot = true;
                hasPendingTx = false;
                pendingBytesToSend = 0;
                trackingTxAux = true;
            }
        }
    } else {
        sentThisSlot = false;
        // Do NOT discard pendingBytesToSend if transmission could not occur in this slot!
        // The popped bytes are safely buffered in rfTxBuffer and will be transmitted in the next slot.
        if (pendingBytesToSend == 0) {
            hasPendingTx = false;
        }
    }

    // 4. RADIO_STATUS injection
    //
    // AIR UNIT: inject to FC UART at 2 Hz (500 ms) for ArduPilot flow control.
    // Dynamic txbuf pacing based on ArduPilot GCS_Param.cpp / GCS_Common.cpp:
    //   txbuf <= 33: freezes parameter sending (GCS_Param.cpp:78) to let queue drain
    //   txbuf < 50:  adds slowdown per packet (GCS_Common.cpp:921-937)
    //   txbuf > 95:  speeds up streams
    //
    // GROUND UNIT: inject to QGC USB at 1 Hz.
    // injectRadioStatus() defers the packet; writeToLocal() injects it at the
    // next MAVLink frame boundary — preventing mid-packet stream corruption.
    #if defined(DUAL_LRS_ROLE_AIR)
    if (now - lastStatusInjectMs >= 500) {
        lastStatusInjectMs = now;
        size_t queued = telemHandler.pendingBytes();
        // Scale txbuf to actual 48KB buffer size (49152 bytes):
        // 1037 parameters * 37B = ~38KB total table, fits completely in RAM without freezing
        uint8_t txbufPct = 96;
        if (queued > 40000) {
            txbufPct = 10;  // Emergency: buffer nearly full (>80%)
        } else if (queued > 32000) {
            txbufPct = 30;  // Backlog: freeze parameters (>65%) to avoid overflow
        } else if (queued > 20000) {
            txbufPct = 50;  // Gentle pacing
        } else if (queued > 10000) {
            txbufPct = 75;  // Moderate load
        } else {
            txbufPct = 96;  // Free queue: maximum speed
        }
        telemHandler.injectRadioStatus(200, 200, txbufPct, 0);
    }
    #else
    // Ground: 1 Hz diagnostic report in RADIO_STATUS
    if (now - lastStatusInjectMs >= 1000) {
        lastStatusInjectMs = now;
        uint16_t rxPackets = (uint16_t)(tdm.getStats().packets_received & 0xFFFF);
        uint16_t txPackets = (uint16_t)(tdm.getStats().packets_sent & 0xFFFF);
        uint16_t droppedCount = (uint16_t)(tdm.getStats().packets_dropped & 0xFFFF);
        uint8_t dataPkts = (uint8_t)(gcsDataPktsRx & 0xFF);
        uint8_t rfSent = (uint8_t)(gcsMavlinkRfSent & 0xFF);
        uint8_t txQueueBytes = (uint8_t)(telemHandler.pendingBytes() > 255 ? 255 : telemHandler.pendingBytes());
        telemHandler.injectRadioStatus(dataPkts, rfSent, txQueueBytes, droppedCount, txPackets);
    }
    #endif

    // 5. Link status LED
    if (!tdm.getStats().synchronized) {
        if (now - lastLedToggleMs >= 250) {
            lastLedToggleMs = now;
            digitalWrite(PIN_LED_SYNC, !digitalRead(PIN_LED_SYNC));
        }
    } else {
        if (now - tdm.getStats().last_sync_ms > 200) {
            digitalWrite(PIN_LED_SYNC, LED_PIN_OFF);
        }
    }

    #if defined(DUAL_LRS_ROLE_AIR) && defined(USBCON)
    static uint32_t lastAirDiagMs = 0;
    if (Serial && (now - lastAirDiagMs >= 1000)) {
        lastAirDiagMs = now;
        Serial.printf("[AIR] Baud:%lu(%s) PKTS:%lu PARAMS_FC:%lu TX:%lu(DATA:%lu) RX:%lu DROPS:%lu(CRC:%lu,SEQ:%lu) GCS_RX:%lu Sync:%d Arr:%lu Err:%ld AuxUs:%lu MaxAux:%lu Q:%u FC_avail:%d\n",
            (unsigned long)currentFcBaud,
            fcBaudLocked ? "LOCKED" : "SCAN",
            (unsigned long)telemHandler.validPackets(),
            (unsigned long)telemHandler.validParamValues(),
            (unsigned long)tdm.getStats().packets_sent,
            (unsigned long)gcsMavlinkRfSent,
            (unsigned long)tdm.getStats().packets_received,
            (unsigned long)tdm.getStats().packets_dropped,
            (unsigned long)tdm.getStats().crc_errors,
            (unsigned long)tdm.getStats().seq_drops,
            (unsigned long)gcsDataPktsRx,
            tdm.getStats().synchronized ? 1 : 0,
            (unsigned long)tdm.getStats().last_arrival_us,
            (long)tdm.getStats().last_pll_err,
            (unsigned long)lastTxAuxUs,
            (unsigned long)maxTxAuxUs,
            (unsigned int)telemHandler.pendingBytes(),
            SerialTELEM.available()
        );
    }
    #endif
}
