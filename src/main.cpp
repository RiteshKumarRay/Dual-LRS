#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"
#include "tdm_engine.h"
#include "mavlink_handler.h"
#include "rc_adapter.h"

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
    static RcGroundAdapter rcGroundAdapter;
#elif defined(DUAL_LRS_ROLE_AIR)
    #define SerialRadio Serial1
    #define SerialTELEM Serial2
    const NodeRole CURRENT_ROLE = NodeRole::AIR;
    // Air unit interfaces with Flight Controller via Hardware Serial (USART2)
    MavlinkHandler telemHandler(SerialTELEM);
    // Air unit CRSF output to FC RC_IN via Hardware USART6 (PA12 RX / PA11 TX)
    Uart SerialAirCRSF(PIN_AIR_CRSF_RX, PIN_AIR_CRSF_TX);
    static RcAirAdapter rcAirAdapter;
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
    static RcGroundAdapter rcGroundAdapter;
#endif

// [WIFI TELEMETRY - PRESERVED FOR FUTURE USE]
// #if defined(ESP32) && defined(ENABLE_WIFI_TELEMETRY) && (ENABLE_WIFI_TELEMETRY == 1)
// #include "wifi_telemetry.h"
// static WifiTelemetry wifiTelem;
// #endif

// Drivers and Handlers
E22Driver radio(SerialRadio, PIN_RADIO_M0, PIN_RADIO_M1, PIN_RADIO_AUX);
TdmEngine tdm(radio, CURRENT_ROLE);

#if defined(DUAL_LRS_ROLE_AIR)
static TransportFragmenter airDownlinkFragmenter;
static TransportReassembler airReassembler;
static uint8_t airTxMsgBuf[TRANSPORT_MAX_TRANSFER_SIZE];
#else
static TransportFragmenter groundUplinkFragmenter;
static TransportReassembler groundReassembler;
static uint8_t groundTxMsgBuf[TRANSPORT_MAX_TRANSFER_SIZE];
#endif

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
    (void)type;
    (void)payload;
    (void)length;
    // Active RF reception indicator
    digitalWrite(PIN_LED_SYNC, LED_PIN_ON);
}

void onTransportFrameReceived(const TransportHeader& hdr, const uint8_t* payload, uint16_t length) {
#if defined(DUAL_LRS_ROLE_AIR)
    if (hdr.channel == (uint8_t)TransportChannel::RC_CONTROL) {
        if (length >= sizeof(TransportPackedRc) && payload != nullptr) {
            const TransportPackedRc* packed = reinterpret_cast<const TransportPackedRc*>(payload);
            rcAirAdapter.ingest_rc_frame(*packed, millis());
        }
    }
#if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
    else if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_UPLINK) {
        TransportNackReason nack = airReassembler.process_fragment(hdr, payload, millis());
        if (nack == TransportNackReason::NONE && airReassembler.is_complete()) {
            telemHandler.writeToLocal(airReassembler.get_reassembled_data(),
                                      airReassembler.get_reassembled_length());
            airReassembler.mark_complete_consumed();
        }
    }
#endif
#else
#if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
    if (hdr.channel == (uint8_t)TransportChannel::MAVLINK_DOWNLINK) {
        TransportNackReason nack = groundReassembler.process_fragment(hdr, payload, millis());
        if (nack == TransportNackReason::NONE && groundReassembler.is_complete()) {
            gcsDataPktsRx++;
            telemHandler.writeToLocal(groundReassembler.get_reassembled_data(),
                                      groundReassembler.get_reassembled_length());
            groundReassembler.mark_complete_consumed();
        }
    }
#endif
#endif
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

#if defined(ESP32)
struct RcScanProfile {
    uint32_t baud;
    uint32_t uart_config;
    bool invert;
    bool is_sbus;          // always false — TX2 outputs CRSF only
    const char* desc;
};
// TX2 on FS-i6X (OpenI6X) outputs CRSF inverted at configurable baudrate.
// Radio Settings -> Hardware -> Baudrate controls TX2 rate. 400000 is the ELRS default.
// SBUS profiles removed: PA10 is an SBUS *input*, PA9 is telemetry mirror — neither
// is the RC output. 0x0F inside CRSF payloads was triggering false SBUS matches.
static const RcScanProfile SCAN_PROFILES[] = {
    {400000, SERIAL_8N1, true,  false, "400k Inv (CRSF - ELRS default, TX2)"},
    {115200, SERIAL_8N1, true,  false, "115k Inv (CRSF - high reliability)"},
    {420000, SERIAL_8N1, true,  false, "420k Inv (CRSF - EdgeTX/OpenTX default)"},
    {921600, SERIAL_8N1, true,  false, "921k Inv (CRSF - high-speed)"},
    {460800, SERIAL_8N1, true,  false, "460k Inv (CRSF - high-speed ELRS)"},
    {115200, SERIAL_8N1, false, false, "115k Norm (CRSF 3.3V low-speed)"},
    {400000, SERIAL_8N1, false, false, "400k Norm (CRSF 3.3V, no invert)"},
    {420000, SERIAL_8N1, false, false, "420k Norm (CRSF 3.3V, no invert)"},
    {921600, SERIAL_8N1, false, false, "921k Norm (CRSF - high-speed)"},
};
static uint8_t currentScanIdx = 0;
static bool rcHandsetLocked = false;
static uint32_t lastScanSwitchMs = 0;
static uint32_t lastValidCheckCount = 0;

// Standard CRSF DEVICE_INFO response (announces as ELRS v2.0 module to unlock OpenI6X channel streaming)
static const uint8_t CRSF_DEVICE_INFO_REPLY[] = {
    0xEA, 0x17, 0x29, 0xEA, 0xEE,
    'E', 'L', 'R', 'S', 0x00,
    'E', 'L', 'R', 'S',
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0xB9
};
static uint32_t lastPingReplyMs = 0;

static void sendCrsfDeviceInfoReply(uint32_t baud) {
    Serial1.end();
    // Temporarily reconfigure UART1 with TX on GPIO 13 (inverted) to transmit reply to radio
    Serial1.begin(baud, SERIAL_8N1, -1, PIN_GROUND_RC_RX, true);
    Serial1.write(CRSF_DEVICE_INFO_REPLY, sizeof(CRSF_DEVICE_INFO_REPLY));
    Serial1.flush();
    delayMicroseconds(60);
    // Switch GPIO 13 back to RX
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, PIN_GROUND_RC_RX, -1, true);
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
        delay(300); // Let USB CDC settle so first log lines are not lost
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

    #if defined(ESP32)
        // Handset ingest on UART1 (starts with profile 0, auto-scans until locked)
        Serial1.begin(SCAN_PROFILES[0].baud, SCAN_PROFILES[0].uart_config, PIN_GROUND_RC_RX, PIN_GROUND_RC_TX, SCAN_PROFILES[0].invert);
        lastScanSwitchMs = millis();
    #elif defined(DUAL_LRS_ROLE_AIR)
        // Air CRSF output on hardware USART6 (PA11 TX / PA12 RX @ 420,000 baud)
        SerialAirCRSF.begin(AIR_CRSF_BAUD);
    #endif

    tdm.onPacketReceived(onRadioPacketReceived);
    tdm.onTransportFrameReceived(onTransportFrameReceived);
    tdm.begin();
    telemHandler.begin();
}

void loop() {
    uint32_t now = millis();

#if defined(ESP32)
    // 1. Ingest handset RC frames from OpenI6X on UART1
    static uint8_t rawRing[32] = {0};
    static uint8_t rawRingIdx = 0;
    static uint32_t lastByteUs = 0;

    // Send CRSF handshake reply if radio is pinging or we do not have handset lock yet
    if (!rcGroundAdapter.has_handset_signal(now) && (now - lastPingReplyMs >= 200)) {
        lastPingReplyMs = now;
        sendCrsfDeviceInfoReply(SCAN_PROFILES[currentScanIdx].baud);
    }

    while (Serial1.available() > 0) {
        uint32_t nowUs = micros();
        uint32_t dtUs = (lastByteUs == 0) ? 0 : (nowUs - lastByteUs);
        lastByteUs = nowUs;

        uint8_t b = (uint8_t)Serial1.read();
        rawRing[rawRingIdx++ % 32] = b;
        rcGroundAdapter.feed_byte(b, now, dtUs);
    }

    // If OpenI6X sent a CRSF Ping frame, reply IMMEDIATELY while PA2 is in receive mode!
    if (rcGroundAdapter.pop_ping_request()) {
        sendCrsfDeviceInfoReply(SCAN_PROFILES[currentScanIdx].baud);
    }

    // Auto-detect and lock exact handset baud rate and inversion
    if (rcHandsetLocked && !rcGroundAdapter.has_handset_signal(now)) {
        rcHandsetLocked = false;
        lastScanSwitchMs = now;
        lastValidCheckCount = rcGroundAdapter.get_total_valid_frames();
    }
    if (!rcHandsetLocked) {
        const RcScanProfile& p = SCAN_PROFILES[currentScanIdx];
        uint32_t validNow = rcGroundAdapter.get_total_valid_frames();
        if (validNow >= 10 && (validNow - lastValidCheckCount >= 5)) {
            rcHandsetLocked = true;
            Serial.printf("{\"event\":\"HANDSET_LOCKED\",\"baud\":%lu,\"invert\":%s,\"desc\":\"%s\"}\n",
                (unsigned long)p.baud,
                p.invert ? "true" : "false",
                p.desc);
        } else if (now - lastScanSwitchMs >= 3000 && (lastByteUs == 0 || (micros() - lastByteUs > 1000000))) {
            lastScanSwitchMs = now;
            currentScanIdx = (currentScanIdx + 1) % (sizeof(SCAN_PROFILES) / sizeof(SCAN_PROFILES[0]));
            Serial1.end();
            Serial1.begin(SCAN_PROFILES[currentScanIdx].baud, SCAN_PROFILES[currentScanIdx].uart_config,
                          PIN_GROUND_RC_RX, PIN_GROUND_RC_TX, SCAN_PROFILES[currentScanIdx].invert);
            // Reset framing state (not stats) then capture baseline for the new profile
            rcGroundAdapter.reset_protocol();
            lastValidCheckCount = rcGroundAdapter.get_total_valid_frames();
            const RcScanProfile& np = SCAN_PROFILES[currentScanIdx];
            Serial.printf("{\"event\":\"SCAN_TRY\",\"idx\":%u,\"baud\":%lu,\"invert\":%s,\"desc\":\"%s\"}\n",
                currentScanIdx,
                (unsigned long)np.baud,
                np.invert ? "true" : "false",
                np.desc);
        }
    }
#elif defined(DUAL_LRS_ROLE_AIR)
    // 1. Update Air RC failsafe watchdog and emit 26B CRSF frame to FC RC_IN on PA11
    rcAirAdapter.update(now);
    if (rcAirAdapter.has_new_frame()) {
        uint8_t fc_crsf_buf[CRSF_FRAME_RC_TOTAL_SIZE];
        size_t fc_crsf_len = rcAirAdapter.get_fc_frame(fc_crsf_buf);
        rcAirAdapter.clear_new_frame();
        if (fc_crsf_len > 0) {
            SerialAirCRSF.write(fc_crsf_buf, fc_crsf_len);
        }
    }
#endif

    // 2. Read bytes from local MAVLink stream (FC or Mission Planner)
    telemHandler.readFromLocal();

#if defined(DUAL_LRS_ROLE_AIR)
    airReassembler.check_timeout(now);
#else
    groundReassembler.check_timeout(now);
#endif

    // [WIFI TELEMETRY - PRESERVED FOR FUTURE USE]
    // #if defined(ESP32) && defined(ENABLE_WIFI_TELEMETRY) && (ENABLE_WIFI_TELEMETRY == 1)
    // wifiTelem.update(telemHandler);
    // #endif

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
    static uint8_t reqCount = 0;
    #if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
    // Request telemetry streams from ArduPilot on startup (or after baud switch).
    static uint32_t lastStreamReqMs = 0;
    if ((reqCount < 8 || (now - telemHandler.lastValidPacketMs() > 5000)) && (now - lastStreamReqMs >= 2500)) {
        lastStreamReqMs = now;
        if (reqCount < 8) reqCount++;
        requestStream(SerialTELEM, 2,  1); // SYS_STATUS (Battery)    at 1 Hz
        requestStream(SerialTELEM, 6,  2); // POSITION (GPS)           at 2 Hz
        requestStream(SerialTELEM, 10, 4); // EXTRA1 (Attitude)        at 4 Hz
        requestStream(SerialTELEM, 11, 2); // EXTRA2 (VFR_HUD)         at 2 Hz
    }
    #endif

    // Auto-Baud scanner for Flight Controller UART:
    // Locks when valid Heartbeat packets arrive from ArduPilot.
    // Once locked, it stays permanently locked to prevent comms blackouts during active operations.
    static bool fcBaudPermanentlyLocked = false;
    if (!fcBaudPermanentlyLocked) {
        if (telemHandler.validHeartbeats() > 0 && (now - telemHandler.lastHeartbeatMs() < 5000)) {
            fcBaudLocked = true;
            fcBaudPermanentlyLocked = true;
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
            sentThisSlot = true; // Exactly 1 attempt per 90ms cycle to prevent slot spill
            bool ok = false;
            lastTxStartUs = micros();

            if (CURRENT_ROLE == NodeRole::GROUND) {
#if !defined(DUAL_LRS_ROLE_AIR)
                TransportPackedRc packed_rc;
                bool has_rc = rcGroundAdapter.get_packed_rc(&packed_rc, now);

                // Priority 1: Handset RC control (unconditional)
                // When handset is active, send RC_CONTROL
                // If GCS uplink MAVLink is pending (and Stage 3.1 RC-only mode is OFF),
                // allow 1 uplink slot every 4 slots (or when handset is inactive)
                bool send_rc = has_rc;
                static uint8_t slotsSinceRc = 0;
#if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
                bool has_uplink_mavlink = groundUplinkFragmenter.has_next_fragment() || telemHandler.hasOutboundData();
                if (has_rc && has_uplink_mavlink && slotsSinceRc >= 3) {
                    send_rc = false;
                    slotsSinceRc = 0;
                } else if (has_rc) {
                    send_rc = true;
                    slotsSinceRc++;
                }

                if (!send_rc && has_uplink_mavlink) {
                    if (!groundUplinkFragmenter.has_next_fragment()) {
                        size_t pkt_len = telemHandler.getOutboundPacket(groundTxMsgBuf, sizeof(groundTxMsgBuf));
                        if (pkt_len > 0) {
                            groundUplinkFragmenter.start_transfer(TransportChannel::MAVLINK_UPLINK, groundTxMsgBuf, (uint16_t)pkt_len, false);
                        }
                    }
                    if (groundUplinkFragmenter.has_next_fragment()) {
                        uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
                        size_t frame_len = groundUplinkFragmenter.get_next_fragment(frame_buf, tdm.getNextSequence());
                        if (frame_len > 0) {
                            ok = tdm.sendRawTransportFrame(frame_buf, frame_len);
                            if (ok) gcsMavlinkRfSent++;
                        }
                    }
                }
#endif
                // RC transmission remains enabled in every production mode.
                if (send_rc) {
                    ok = tdm.sendTransportFrame(TransportChannel::RC_CONTROL,
                                                TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                                0, 0, (const uint8_t*)&packed_rc, sizeof(TransportPackedRc));
                } else if (!ok) {
                    // Fallback beacon if no RC and no MAVLink sent
                    ok = tdm.sendTransportFrame(TransportChannel::LINK_CONTROL,
                                                TRANSPORT_FLAG_FIRST_FRAG | TRANSPORT_FLAG_LAST_FRAG,
                                                0, 0, nullptr, 0);
                }
#endif
            } else {
                // CURRENT_ROLE == NodeRole::AIR (Air slot: 45 ms)
#if defined(DUAL_LRS_ROLE_AIR)
#if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
                // Stage 3.2 MAVLink Downlink Telemetry (Air -> Ground)
                if (!airDownlinkFragmenter.has_next_fragment()) {
                    size_t pkt_len = telemHandler.getOutboundPacket(airTxMsgBuf, sizeof(airTxMsgBuf));
                    if (pkt_len > 0) {
                        airDownlinkFragmenter.start_transfer(TransportChannel::MAVLINK_DOWNLINK, airTxMsgBuf, (uint16_t)pkt_len, false);
                    }
                }
                if (airDownlinkFragmenter.has_next_fragment()) {
                    uint8_t frame_buf[TRANSPORT_MAX_FRAME_SIZE];
                    size_t frame_len = airDownlinkFragmenter.get_next_fragment(frame_buf, tdm.getNextSequence());
                    if (frame_len > 0) {
                        ok = tdm.sendRawTransportFrame(frame_buf, frame_len);
                        if (ok) gcsMavlinkRfSent++;
                    }
                }
#else
                // STAGE 3.1 RC-ONLY MODE: Air transmits nothing to keep downlinks completely clean
                ok = true;
#endif
#endif
            }

            if (ok) {
                trackingTxAux = true;
            }
        }
    } else {
        sentThisSlot = false;
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
    #if (!defined(DUAL_LRS_STAGE31_RC_ONLY) || (DUAL_LRS_STAGE31_RC_ONLY == 0))
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
    #endif

    #if defined(ESP32)
    // Ground USB Live Monitor: emits 10 Hz status for live visualization / calibration
    static uint32_t lastGroundLogMs = 0;
    if (Serial && (now - lastGroundLogMs >= 100)) {
        lastGroundLogMs = now;
        const RcAdapterStats& rcStats = rcGroundAdapter.get_stats();
        const uint16_t* ch = rcGroundAdapter.get_channels();
        bool active = rcGroundAdapter.has_handset_signal(now);
        if (true) {  // Always emit so monitor shows scan progress + signal absence
            const uint8_t* raw22 = rcGroundAdapter.get_raw_channels22();
            char hexBuf[45] = {0};
            if (raw22) {
                for (int i = 0; i < 22; i++) sprintf(&hexBuf[i*2], "%02X", raw22[i]);
            }
            char wireBuf[65] = {0};
            for (int i = 0; i < 32; i++) sprintf(&wireBuf[i*2], "%02X", rawRing[(rawRingIdx + i) % 32]);
            Serial.printf("{\"event\":\"RC_GROUND_IN\",\"active\":%s,\"sbus\":%s,\"crsf_ok\":%lu,\"frames\":%lu,\"rf_tx\":%lu,\"crc_err\":%lu,\"wire\":\"%s\",\"hex\":\"%s\",\"ch\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}\n",
                active ? "true" : "false",
                rcGroundAdapter.is_using_sbus() ? "true" : "false",
                (unsigned long)rcGroundAdapter.get_crsf_valid_frames(),
                (unsigned long)rcStats.crsf_frames_in,
                (unsigned long)rcStats.rf_frames_sent,
                (unsigned long)rcStats.crsf_crc_errors,
                wireBuf,
                hexBuf,
                ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
                ch[8], ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15]);
        }
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
    static uint32_t lastAirRcLogMs = 0;
    if (Serial && (now - lastAirRcLogMs >= 50)) {
        lastAirRcLogMs = now;
        const uint16_t* ch = rcAirAdapter.get_channels();
        char jsonBuf[220];
        int n = snprintf(jsonBuf, sizeof(jsonBuf),
            "{\"event\":\"RC_AIR_RX\",\"rf_rx\":%lu,\"failsafe\":%s,\"fs_events\":%lu,\"ch\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}\n",
            (unsigned long)rcAirAdapter.get_stats().rf_frames_received,
            rcAirAdapter.is_failsafe_active() ? "true" : "false",
            (unsigned long)rcAirAdapter.get_stats().failsafe_events,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
            ch[8], ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15]);
        if (n > 0) {
            Serial.write((const uint8_t*)jsonBuf, (size_t)n);
        }
    }

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
