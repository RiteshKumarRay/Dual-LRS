#include <Arduino.h>
#include "config.h"
#include "e22_driver.h"
#include "diag_protocol.h"
#include "rc_adapter.h"

#if defined(ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// =============================================================================
// HARDWARE PORT MAPPING
// =============================================================================
#if defined(ESP32)
    #define SerialRadio Serial2
    static const char* ROLE_STR = "GROUND";
    static const uint8_t LOCAL_ROLE_ID = 1;
#elif defined(DUAL_LRS_ROLE_AIR)
    #define SerialRadio Serial1
    static const char* ROLE_STR = "AIR";
    static const uint8_t LOCAL_ROLE_ID = 0;
    Uart SerialAirCRSF(PIN_AIR_CRSF_RX, PIN_AIR_CRSF_TX);
#else
    #define SerialRadio Serial1
    static const char* ROLE_STR = "GROUND";
    static const uint8_t LOCAL_ROLE_ID = 1;
#endif

// Shared E22 driver instance
static E22Driver radio(SerialRadio, PIN_RADIO_M0, PIN_RADIO_M1, PIN_RADIO_AUX);

// System readiness flag (Fails closed)
static bool modemUsable = false;

// Operating states
enum class DiagMode : uint8_t {
    IDLE = 0,
    MODE_A_TX,     // Air -> Ground simplex TX (Air active)
    MODE_A_RX,     // Air -> Ground simplex RX (Ground silent)
    MODE_B_TX,     // Ground -> Air simplex TX (Ground active)
    MODE_B_RX,     // Ground -> Air simplex RX (Air silent)
    MODE_C_MASTER, // Ground Master Ping-Pong RTT
    MODE_C_SLAVE,  // Air Slave Ping-Pong responder
    MODE_RC_GROUND,// Ground: FS-i6X CRSF Ingest on GPIO 13/14 -> RF 39B Uplink
    MODE_RC_AIR    // Air: RF 39B Ingest -> USB CDC live channel monitor
};

static DiagMode currentMode = DiagMode::IDLE;

// Configuration & Pacing (Default 100ms / 10 Hz conservative)
static uint8_t  activePayloadLen     = 0;
static uint16_t targetFrameCount     = 0;
static uint16_t framesSent           = 0;
static uint32_t pacingIntervalMs     = 100;   // Default 100ms conservative
static uint32_t parserTimeoutUs      = 30000; // Default 30ms configurable parser timeout
static uint8_t  activeTestId         = 1;

// Statistics
static uint32_t statTxAttempts       = 0;
static uint32_t statTxSuccess        = 0;
static uint32_t statTxDeferred       = 0;
static uint32_t statOverruns         = 0;
static uint32_t statRxValidFrames    = 0;
static uint32_t statRxCrcErrors      = 0;
static uint32_t statRxPatternErrors  = 0;
static uint32_t statRxMalformedFrames= 0;
static uint32_t statRxTimeouts       = 0;
static uint32_t statRxGaps           = 0;
static uint32_t statRxDuplicates     = 0;
static uint16_t lastReceivedSeq      = 0xFFFF;
static bool     hasReceivedSeq       = false;

// =============================================================================
// BOUNDED NON-BLOCKING EVENT QUEUE (Prevents USB Serial from starving Radio RX)
// =============================================================================
enum class EventType : uint8_t {
    NONE = 0,
    TX_FRAME,
    TX_DEFERRED,
    RX_FRAME,
    RX_TIMEOUT,
    RX_MALFORMED,
    MODE_C_RTT,
    MODE_C_TIMEOUT,
    MODE_C_INVALID_PATTERN,
    MODE_C_PONG_SENT
};

struct LogEvent {
    EventType type;
    char      mode_char;
    uint8_t   test_id;
    uint8_t   dir;
    uint16_t  seq;
    uint8_t   payload_len;
    uint8_t   frame_len;
    uint8_t   seq_status; // 0=IN_ORDER, 1=GAP, 2=DUP, 3=OUT_OF_ORDER
    bool      crc_ok;
    bool      pattern_ok;
    bool      aux_pre_busy;
    bool      aux_edge_observed;
    bool      aux_wait_timeout;
    bool      overrun;
    uint16_t  chunks;
    uint32_t  t_uart_start_us;
    uint32_t  t_uart_end_us;
    uint32_t  uart_duration_us;
    uint32_t  aux_busy_start_us;
    uint32_t  aux_ready_us;
    uint32_t  aux_busy_duration_us;
    uint32_t  rx_duration_us;
    uint32_t  max_inter_byte_gap_us;
    uint32_t  t1_us;
    uint32_t  t4_us;
    uint32_t  turnaround_us;
    uint32_t  rtt_us;
    uint32_t  approx_one_way_us;
};

static const size_t EVENT_QUEUE_CAPACITY = 32;
static LogEvent eventQueue[EVENT_QUEUE_CAPACITY];
static size_t   eventQueueHead  = 0;
static size_t   eventQueueTail  = 0;
static size_t   eventQueueCount = 0;
static uint32_t statLogOverflows= 0;

static void enqueueEvent(const LogEvent& ev) {
    if (eventQueueCount >= EVENT_QUEUE_CAPACITY) {
        statLogOverflows++;
        return;
    }
    eventQueue[eventQueueHead] = ev;
    eventQueueHead = (eventQueueHead + 1) % EVENT_QUEUE_CAPACITY;
    eventQueueCount++;
}

static void serializeAndEmitEvent(const LogEvent& ev) {
    switch (ev.type) {
        case EventType::TX_FRAME:
            Serial.print(F("{\"event\":\"TX_FRAME\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"mode\":\""));
            Serial.print(ev.mode_char);
            Serial.print(F("\",\"test_id\":"));
            Serial.print(ev.test_id);
            Serial.print(F(",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.print(F(",\"frame_len\":"));
            Serial.print(ev.frame_len);
            Serial.print(F(",\"t_uart_start_us\":"));
            Serial.print(ev.t_uart_start_us);
            Serial.print(F(",\"t_uart_end_us\":"));
            Serial.print(ev.t_uart_end_us);
            Serial.print(F(",\"uart_duration_us\":"));
            Serial.print(ev.uart_duration_us);
            Serial.print(F(",\"aux_pre_busy\":"));
            Serial.print(ev.aux_pre_busy ? F("true") : F("false"));
            Serial.print(F(",\"aux_edge_observed\":"));
            Serial.print(ev.aux_edge_observed ? F("true") : F("false"));
            Serial.print(F(",\"aux_wait_timeout\":"));
            Serial.print(ev.aux_wait_timeout ? F("true") : F("false"));
            Serial.print(F(",\"aux_busy_start_us\":"));
            Serial.print(ev.aux_busy_start_us);
            Serial.print(F(",\"aux_ready_us\":"));
            Serial.print(ev.aux_ready_us);
            Serial.print(F(",\"aux_busy_duration_us\":"));
            Serial.print(ev.aux_busy_duration_us);
            Serial.print(F(",\"overrun\":"));
            Serial.print(ev.overrun ? F("true") : F("false"));
            Serial.println(F("}"));
            break;

        case EventType::TX_DEFERRED:
            Serial.print(F("{\"event\":\"TX_DEFERRED\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"reason\":\"AUX_BUSY_TIMEOUT\"}\n"));
            break;

        case EventType::RX_FRAME: {
            const char* seqStr = (ev.seq_status == 0) ? "IN_ORDER" :
                                 (ev.seq_status == 1) ? "GAP" :
                                 (ev.seq_status == 2) ? "DUPLICATE" : "OUT_OF_ORDER";
            Serial.print(F("{\"event\":\"RX_FRAME\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"test_id\":"));
            Serial.print(ev.test_id);
            Serial.print(F(",\"dir\":"));
            Serial.print(ev.dir);
            Serial.print(F(",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.print(F(",\"frame_len\":"));
            Serial.print(ev.frame_len);
            Serial.print(F(",\"crc_ok\":"));
            Serial.print(ev.crc_ok ? F("true") : F("false"));
            Serial.print(F(",\"pattern_ok\":"));
            Serial.print(ev.pattern_ok ? F("true") : F("false"));
            Serial.print(F(",\"seq_status\":\""));
            Serial.print(seqStr);
            Serial.print(F("\",\"rx_duration_us\":"));
            Serial.print(ev.rx_duration_us);
            Serial.print(F(",\"max_inter_byte_gap_us\":"));
            Serial.print(ev.max_inter_byte_gap_us);
            Serial.print(F(",\"chunks\":"));
            Serial.print(ev.chunks);
            Serial.println(F("}"));
            break;
        }

        case EventType::RX_TIMEOUT:
            Serial.print(F("{\"event\":\"RX_TIMEOUT\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"bytes_read\":"));
            Serial.print(ev.payload_len);
            Serial.print(F(",\"expected_bytes\":"));
            Serial.print(ev.frame_len);
            Serial.print(F(",\"max_inter_byte_gap_us\":"));
            Serial.print(ev.max_inter_byte_gap_us);
            Serial.print(F(",\"chunks\":"));
            Serial.print(ev.chunks);
            Serial.println(F("}"));
            break;

        case EventType::RX_MALFORMED:
            Serial.print(F("{\"event\":\"RX_MALFORMED\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"reason\":\"PAYLOAD_LENGTH_INVALID\",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.println(F("}"));
            break;

        case EventType::MODE_C_RTT:
            Serial.print(F("{\"event\":\"MODE_C_RTT\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.print(F(",\"crc_ok\":"));
            Serial.print(ev.crc_ok ? F("true") : F("false"));
            Serial.print(F(",\"pattern_ok\":"));
            Serial.print(ev.pattern_ok ? F("true") : F("false"));
            Serial.print(F(",\"t1_us\":"));
            Serial.print(ev.t1_us);
            Serial.print(F(",\"t4_us\":"));
            Serial.print(ev.t4_us);
            Serial.print(F(",\"turnaround_us\":"));
            Serial.print(ev.turnaround_us);
            Serial.print(F(",\"rtt_us\":"));
            Serial.print(ev.rtt_us);
            Serial.print(F(",\"approx_one_way_us\":"));
            Serial.print(ev.approx_one_way_us);
            Serial.println(F(",\"note\":\"approx_one_way assumes symmetric path, includes modem buffering and UART/RF transfer\"}"));
            break;

        case EventType::MODE_C_INVALID_PATTERN:
            Serial.print(F("{\"event\":\"MODE_C_INVALID_PATTERN\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.println(F(",\"reason\":\"PATTERN_MISMATCH\"}"));
            break;

        case EventType::MODE_C_TIMEOUT:
            Serial.print(F("{\"event\":\"MODE_C_TIMEOUT\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"payload_len\":"));
            Serial.print(ev.payload_len);
            Serial.print(F(",\"timeout_ms\":"));
            Serial.print(ev.t1_us);
            Serial.println(F("}"));
            break;

        case EventType::MODE_C_PONG_SENT:
            Serial.print(F("{\"event\":\"MODE_C_PONG_SENT\",\"role\":\""));
            Serial.print(ROLE_STR);
            Serial.print(F("\",\"seq\":"));
            Serial.print(ev.seq);
            Serial.print(F(",\"turnaround_us\":"));
            Serial.print(ev.turnaround_us);
            Serial.println(F("}"));
            break;

        default:
            break;
    }
}

// Receiver parser state
enum class DiagParserState : uint8_t {
    WAIT_MAGIC0 = 0,
    WAIT_MAGIC1,
    READ_HEADER,
    READ_PAYLOAD_AND_CRC
};

static DiagParserState rxParserState = DiagParserState::WAIT_MAGIC0;
static uint8_t  rxBuffer[DIAG_BUFFER_SIZE];
static size_t   rxBytesCount = 0;
static size_t   rxExpectedTotal = 0;
static uint32_t rxFirstByteUs = 0;
static uint32_t rxLastByteUs = 0;
static uint32_t rxMaxInterByteGapUs = 0;
static uint16_t rxChunksCount = 0;

// Shared parser reset helper
static inline void resetRxParser() {
    rxParserState = DiagParserState::WAIT_MAGIC0;
    rxBytesCount = 0;
    rxExpectedTotal = 0;
    rxMaxInterByteGapUs = 0;
    rxChunksCount = 0;
}

// Shared malformed frame reporting helper
static inline void reportMalformedFrame(uint8_t payloadLen) {
    statRxMalformedFrames++;
    LogEvent ev = {};
    ev.type = EventType::RX_MALFORMED;
    ev.payload_len = payloadLen;
    enqueueEvent(ev);
}

// Host command line buffer
static char cmdBuffer[64];
static size_t cmdBufferLen = 0;

// Forward declarations
static void processHostCommand(const char* cmd);
static void runTxStep();
static void runRxStep();
static void runModeCMasterStep();
static void runModeCSlaveStep();
static void startRcGround();
static void runRcGroundStep();
static void startRcAir();
static void runRcAirStep();
static void handleButtonPress();

// =============================================================================
// SETUP (Fail-Closed Passive Initialization)
// =============================================================================
void setup() {
#if defined(ESP32)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
#endif

    // Host communication USB port
    Serial.begin(115200);

    // Diagnostics LED
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, LED_PIN_OFF);

    // User Key / BOOT button for optional manual triggering
    pinMode(PIN_USER_KEY, INPUT_PULLUP);

    delay(200);

    // Passive E22 driver initialization (Zero 0xC0 writes, no EEPROM modification)
    bool initOk = radio.beginPassive(RADIO_UART_BAUD);

    if (!initOk) {
        modemUsable = false;
        Serial.print(F("{\"event\":\"BOOT_FAILURE\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.println(F("\",\"reason\":\"PASSIVE_INIT_FAILED\",\"note\":\"AUX pin not ready or mode switch timed out\"}"));
        return;
    }

    modemUsable = true;

    // Emit structured JSONL boot banner
    Serial.print(F("{\"event\":\"BOOT\",\"role\":\""));
    Serial.print(ROLE_STR);
    Serial.print(F("\",\"firmware\":\"phase1_diag_v3\",\"baud\":"));
    Serial.print(RADIO_UART_BAUD);
    Serial.print(F(",\"passive_init\":true,\"modem_usable\":true,\"default_pacing_ms\":"));
    Serial.print(pacingIntervalMs);
    Serial.println(F("}"));

    currentMode = DiagMode::IDLE;
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    // 1. Process host commands from USB Serial
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (cmdBufferLen > 0) {
                cmdBuffer[cmdBufferLen] = '\0';
                processHostCommand(cmdBuffer);
                cmdBufferLen = 0;
            }
        } else if (cmdBufferLen < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdBufferLen++] = c;
        }
    }

    // 2. Check onboard user button for quick bench test toggle
    handleButtonPress();

    // 3. Dispatch current active mode (Top priority: radio processing)
    if (modemUsable) {
        switch (currentMode) {
            case DiagMode::IDLE:
                while (radio.available()) {
                    radio.read();
                }
                break;

            case DiagMode::MODE_A_TX:
            case DiagMode::MODE_B_TX:
                runTxStep();
                break;

            case DiagMode::MODE_A_RX:
            case DiagMode::MODE_B_RX:
                runRxStep();
                break;

            case DiagMode::MODE_C_MASTER:
                runModeCMasterStep();
                break;

            case DiagMode::MODE_C_SLAVE:
                runModeCSlaveStep();
                break;

            case DiagMode::MODE_RC_GROUND:
                runRcGroundStep();
                break;

            case DiagMode::MODE_RC_AIR:
                runRcAirStep();
                break;
        }
    } else {
        // Modem in unusable/failed state: slow error blink
        static uint32_t lastErrBlink = 0;
        if (millis() - lastErrBlink > 1000) {
            lastErrBlink = millis();
            digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));
        }
    }

    // 4. Dequeue and emit at most 1 buffered log event per loop iteration (Non-blocking)
    if (eventQueueCount > 0) {
        LogEvent ev = eventQueue[eventQueueTail];
        eventQueueTail = (eventQueueTail + 1) % EVENT_QUEUE_CAPACITY;
        eventQueueCount--;
        serializeAndEmitEvent(ev);
    }

    // 5. Emit overflow notice if logging queue ever dropped events
    if (statLogOverflows > 0) {
        Serial.print(F("{\"event\":\"LOG_BUFFER_OVERFLOW\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.print(F("\",\"dropped_events\":"));
        Serial.print(statLogOverflows);
        Serial.println(F("}"));
        statLogOverflows = 0;
    }
}

// =============================================================================
// TRANSMITTER STEP (Simplex Stream for Mode A or Mode B)
// =============================================================================
static uint32_t lastTxScheduleUs = 0;

static void runTxStep() {
    if (framesSent >= targetFrameCount) {
        // Test batch complete
        Serial.print(F("{\"event\":\"TX_BATCH_COMPLETE\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.print(F("\",\"test_id\":"));
        Serial.print(activeTestId);
        Serial.print(F(",\"payload_len\":"));
        Serial.print(activePayloadLen);
        Serial.print(F(",\"total_sent\":"));
        Serial.print(framesSent);
        Serial.print(F(",\"tx_success\":"));
        Serial.print(statTxSuccess);
        Serial.print(F(",\"tx_deferred\":"));
        Serial.print(statTxDeferred);
        Serial.print(F(",\"overruns\":"));
        Serial.print(statOverruns);
        Serial.println(F("}"));

        digitalWrite(PIN_LED_BUILTIN, LED_PIN_OFF);
        currentMode = DiagMode::IDLE;
        return;
    }

    uint32_t nowUs = micros();
    uint32_t pacingUs = pacingIntervalMs * 1000UL;

    if (framesSent > 0 && (nowUs - lastTxScheduleUs < pacingUs)) {
        return;
    }

    lastTxScheduleUs = nowUs;
    statTxAttempts++;

    // 1. Record AUX state immediately before write
    bool preAuxBusy = radio.isBusy();
    if (preAuxBusy) {
        uint32_t waitStart = millis();
        while (radio.isBusy() && (millis() - waitStart < 10)) {
            delayMicroseconds(50);
        }
        if (radio.isBusy()) {
            statTxDeferred++;
            LogEvent ev = {};
            ev.type = EventType::TX_DEFERRED;
            ev.seq = framesSent;
            enqueueEvent(ev);
            return;
        }
    }

    // 2. Validate payload length before creating frame
    if (!diag_is_valid_frame_len(activePayloadLen)) {
        statTxDeferred++;
        return;
    }

    // 3. Build frame: Header (9B) + Payload (N B) + CRC-16 (2B)
    uint8_t txFrame[DIAG_BUFFER_SIZE];
    RawDiagHeader* hdr = (RawDiagHeader*)txFrame;
    hdr->magic0 = DIAG_MAGIC_0;
    hdr->magic1 = DIAG_MAGIC_1;
    hdr->test_id = activeTestId;
    hdr->direction = (currentMode == DiagMode::MODE_A_TX) ? DIAG_DIR_AIR_TO_GROUND : DIAG_DIR_GROUND_TO_AIR;
    hdr->seq_num = framesSent;
    hdr->payload_len = activePayloadLen;
    hdr->turnaround_or_time = (uint16_t)(millis() & 0xFFFF);

    // Populate deterministic payload pattern
    uint8_t* payloadPtr = txFrame + DIAG_HEADER_SIZE;
    diag_fill_payload(payloadPtr, activePayloadLen, framesSent);

    // Compute CRC-16 over Header + Payload
    size_t dataLen = DIAG_HEADER_SIZE + activePayloadLen;
    uint16_t crc = diag_crc16(txFrame, dataLen);
    txFrame[dataLen]     = (uint8_t)(crc & 0xFF);
    txFrame[dataLen + 1] = (uint8_t)((crc >> 8) & 0xFF);
    size_t totalFrameLen = dataLen + DIAG_CRC_SIZE;

    // 4. Transmit over radio UART and measure UART duration
    digitalWrite(PIN_LED_BUILTIN, LED_PIN_ON);
    uint32_t tUartStart = micros();
    radio.write(txFrame, totalFrameLen);
    radio.flush();
    uint32_t tUartEnd = micros();
    uint32_t uartDurationUs = tUartEnd - tUartStart;

    // 5. Observe AUX transitions
    bool aux_edge_observed = false;
    bool aux_wait_timeout = false;
    uint32_t aux_busy_start_us = 0;
    uint32_t aux_ready_us = 0;
    uint32_t aux_busy_duration_us = 0;

    uint32_t auxPollStart = micros();
    while (micros() - auxPollStart < 15000) {
        if (radio.isBusy()) {
            aux_edge_observed = true;
            aux_busy_start_us = micros();
            break;
        }
        delayMicroseconds(10);
    }

    if (aux_edge_observed) {
        uint32_t waitReadyStartMs = millis();
        while (radio.isBusy() && (millis() - waitReadyStartMs < 150)) {
            delayMicroseconds(50);
        }
        if (radio.isBusy()) {
            aux_wait_timeout = true;
        } else {
            aux_ready_us = micros();
            aux_busy_duration_us = aux_ready_us - aux_busy_start_us;
        }
    }

    digitalWrite(PIN_LED_BUILTIN, LED_PIN_OFF);

    // Detect slot overrun
    uint32_t totalElapsedUs = micros() - nowUs;
    bool isOverrun = (totalElapsedUs > pacingUs);
    if (isOverrun) statOverruns++;
    statTxSuccess++;

    // Enqueue non-blocking event
    LogEvent ev = {};
    ev.type = EventType::TX_FRAME;
    ev.mode_char = (currentMode == DiagMode::MODE_A_TX) ? 'A' : 'B';
    ev.test_id = activeTestId;
    ev.seq = framesSent;
    ev.payload_len = activePayloadLen;
    ev.frame_len = (uint8_t)totalFrameLen;
    ev.t_uart_start_us = tUartStart;
    ev.t_uart_end_us = tUartEnd;
    ev.uart_duration_us = uartDurationUs;
    ev.aux_pre_busy = preAuxBusy;
    ev.aux_edge_observed = aux_edge_observed;
    ev.aux_wait_timeout = aux_wait_timeout;
    ev.aux_busy_start_us = aux_busy_start_us;
    ev.aux_ready_us = aux_ready_us;
    ev.aux_busy_duration_us = aux_busy_duration_us;
    ev.overrun = isOverrun;
    enqueueEvent(ev);

    framesSent++;
}

// =============================================================================
// RECEIVER STEP (Firmware Silent: NEVER calls radio.write)
// =============================================================================
static void runRxStep() {
    uint32_t nowUs = micros();

    // Check parser timeout
    if (rxParserState != DiagParserState::WAIT_MAGIC0) {
        if (nowUs - rxLastByteUs > parserTimeoutUs) {
            statRxTimeouts++;
            LogEvent ev = {};
            ev.type = EventType::RX_TIMEOUT;
            ev.payload_len = (uint8_t)rxBytesCount;
            ev.frame_len = (uint8_t)rxExpectedTotal;
            ev.max_inter_byte_gap_us = rxMaxInterByteGapUs;
            ev.chunks = rxChunksCount;
            enqueueEvent(ev);

            resetRxParser();
        }
    }

    // Read incoming bytes without blocking delays
    while (radio.available()) {
        uint8_t b = (uint8_t)radio.read();
        uint32_t byteTimeUs = micros();

        if (rxBytesCount > 0) {
            uint32_t gap = byteTimeUs - rxLastByteUs;
            if (gap > rxMaxInterByteGapUs) {
                rxMaxInterByteGapUs = gap;
            }
            if (gap > 2000) {
                rxChunksCount++;
            }
        }
        rxLastByteUs = byteTimeUs;

        // Buffer boundary protection
        if (rxBytesCount >= DIAG_BUFFER_SIZE) {
            reportMalformedFrame(0);
            resetRxParser();
            continue;
        }

        switch (rxParserState) {
            case DiagParserState::WAIT_MAGIC0:
                if (b == DIAG_MAGIC_0) {
                    rxBuffer[0] = b;
                    rxBytesCount = 1;
                    rxFirstByteUs = byteTimeUs;
                    rxMaxInterByteGapUs = 0;
                    rxChunksCount = 1;
                    rxParserState = DiagParserState::WAIT_MAGIC1;
                }
                break;

            case DiagParserState::WAIT_MAGIC1:
                if (b == DIAG_MAGIC_1) {
                    rxBuffer[1] = b;
                    rxBytesCount = 2;
                    rxParserState = DiagParserState::READ_HEADER;
                } else {
                    resetRxParser();
                }
                break;

            case DiagParserState::READ_HEADER:
                if (rxBytesCount < DIAG_BUFFER_SIZE) {
                    rxBuffer[rxBytesCount++] = b;
                } else {
                    reportMalformedFrame(0);
                    resetRxParser();
                    break;
                }
                if (rxBytesCount == DIAG_HEADER_SIZE) {
                    RawDiagHeader* hdr = (RawDiagHeader*)rxBuffer;
                    // Validate payload_len before computing expected total
                    if (!diag_is_valid_frame_len(hdr->payload_len)) {
                        reportMalformedFrame(hdr->payload_len);
                        resetRxParser();
                    } else {
                        size_t expTotal = diag_total_frame_len(hdr->payload_len);
                        if (expTotal > DIAG_BUFFER_SIZE) {
                            reportMalformedFrame(hdr->payload_len);
                            resetRxParser();
                        } else {
                            activePayloadLen = hdr->payload_len;
                            rxExpectedTotal = expTotal;
                            rxParserState = DiagParserState::READ_PAYLOAD_AND_CRC;
                        }
                    }
                }
                break;

            case DiagParserState::READ_PAYLOAD_AND_CRC:
                if (rxBytesCount < DIAG_BUFFER_SIZE) {
                    rxBuffer[rxBytesCount++] = b;
                } else {
                    reportMalformedFrame(0);
                    resetRxParser();
                    break;
                }
                if (rxBytesCount == rxExpectedTotal) {
                    uint32_t tCompleteUs = byteTimeUs;
                    uint32_t rxDurationUs = tCompleteUs - rxFirstByteUs;

                    RawDiagHeader* hdr = (RawDiagHeader*)rxBuffer;
                    size_t dataLen = DIAG_HEADER_SIZE + hdr->payload_len;

                    // Verify CRC
                    uint16_t computedCrc = diag_crc16(rxBuffer, dataLen);
                    uint16_t receivedCrc = (uint16_t)rxBuffer[dataLen] | ((uint16_t)rxBuffer[dataLen + 1] << 8);
                    bool crcOk = (computedCrc == receivedCrc);

                    // Verify deterministic payload pattern
                    bool patternOk = false;
                    if (crcOk) {
                        patternOk = diag_verify_payload(rxBuffer + DIAG_HEADER_SIZE, hdr->payload_len, hdr->seq_num);
                    }

                    // Sequence classification: 0=IN_ORDER, 1=GAP, 2=DUP, 3=OUT_OF_ORDER
                    uint8_t seqStatus = 0;
                    if (hasReceivedSeq) {
                        uint16_t expectedSeq = lastReceivedSeq + 1;
                        if (hdr->seq_num == expectedSeq) {
                            seqStatus = 0;
                        } else if (hdr->seq_num > expectedSeq) {
                            seqStatus = 1;
                            statRxGaps += (hdr->seq_num - expectedSeq);
                        } else if (hdr->seq_num == lastReceivedSeq) {
                            seqStatus = 2;
                            statRxDuplicates++;
                        } else {
                            seqStatus = 3;
                        }
                    }
                    lastReceivedSeq = hdr->seq_num;
                    hasReceivedSeq = true;

                    if (crcOk && patternOk) {
                        statRxValidFrames++;
                    } else if (crcOk && !patternOk) {
                        statRxPatternErrors++;
                    } else {
                        statRxCrcErrors++;
                    }

                    // Enqueue event (zero blocking USB delay during receive)
                    LogEvent ev = {};
                    ev.type = EventType::RX_FRAME;
                    ev.test_id = hdr->test_id;
                    ev.dir = hdr->direction;
                    ev.seq = hdr->seq_num;
                    ev.payload_len = hdr->payload_len;
                    ev.frame_len = (uint8_t)rxExpectedTotal;
                    ev.crc_ok = crcOk;
                    ev.pattern_ok = patternOk;
                    ev.seq_status = seqStatus;
                    ev.rx_duration_us = rxDurationUs;
                    ev.max_inter_byte_gap_us = rxMaxInterByteGapUs;
                    ev.chunks = rxChunksCount;
                    enqueueEvent(ev);

                    resetRxParser();
                }
                break;
        }
    }
}

// =============================================================================
// MODE C MASTER STEP (Ground sends Ping, awaits Pong, computes RTT)
// =============================================================================
static uint32_t modeCPingStartUs = 0;
static bool     modeCWaitingPong = false;

static void runModeCMasterStep() {
    if (framesSent >= targetFrameCount && !modeCWaitingPong) {
        Serial.print(F("{\"event\":\"MODE_C_COMPLETE\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.print(F("\",\"total_pings\":"));
        Serial.print(framesSent);
        Serial.print(F(",\"valid_pongs\":"));
        Serial.print(statRxValidFrames);
        Serial.println(F("}"));

        currentMode = DiagMode::IDLE;
        return;
    }

    uint32_t nowUs = micros();

    // Check timeout if waiting for Pong
    if (modeCWaitingPong) {
        if (nowUs - modeCPingStartUs > (pacingIntervalMs * 1000UL)) {
            LogEvent ev = {};
            ev.type = EventType::MODE_C_TIMEOUT;
            ev.seq = framesSent;
            ev.payload_len = activePayloadLen;
            ev.t1_us = pacingIntervalMs;
            enqueueEvent(ev);

            resetRxParser();
            modeCWaitingPong = false;
            framesSent++;
            return;
        }

        // Receive Pong
        while (radio.available()) {
            uint8_t b = (uint8_t)radio.read();
            if (rxBytesCount == 0 && b != DIAG_MAGIC_0) continue;
            if (rxBytesCount == 1 && b != DIAG_MAGIC_1) { resetRxParser(); continue; }

            // Buffer boundary protection
            if (rxBytesCount >= DIAG_BUFFER_SIZE) {
                reportMalformedFrame(0);
                resetRxParser();
                continue;
            }

            if (rxBytesCount < DIAG_BUFFER_SIZE) {
                rxBuffer[rxBytesCount++] = b;
            } else {
                reportMalformedFrame(0);
                resetRxParser();
                continue;
            }

            if (rxBytesCount == DIAG_HEADER_SIZE) {
                RawDiagHeader* hdr = (RawDiagHeader*)rxBuffer;
                // Validate payload_len before computing expected total
                if (!diag_is_valid_frame_len(hdr->payload_len)) {
                    reportMalformedFrame(hdr->payload_len);
                    resetRxParser();
                    continue;
                }
                size_t expTotal = diag_total_frame_len(hdr->payload_len);
                if (expTotal > DIAG_BUFFER_SIZE) {
                    reportMalformedFrame(hdr->payload_len);
                    resetRxParser();
                    continue;
                }
                rxExpectedTotal = expTotal;
            }

            if (rxExpectedTotal > 0 && rxBytesCount == rxExpectedTotal) {
                uint32_t t4Us = micros();
                RawDiagHeader* hdr = (RawDiagHeader*)rxBuffer;
                size_t dataLen = DIAG_HEADER_SIZE + hdr->payload_len;

                uint16_t computedCrc = diag_crc16(rxBuffer, dataLen);
                uint16_t receivedCrc = (uint16_t)rxBuffer[dataLen] | ((uint16_t)rxBuffer[dataLen + 1] << 8);

                // Strictly validate Mode C test ID, sequence, direction, payload length, CRC, and payload pattern
                bool crcMatch = (computedCrc == receivedCrc);
                bool dirMatch = (hdr->direction == DIAG_DIR_PONG);
                bool seqMatch = (hdr->seq_num == framesSent);
                bool idMatch  = (hdr->test_id == activeTestId);
                bool lenMatch = (hdr->payload_len == activePayloadLen);

                bool patternOk = false;
                if (crcMatch && dirMatch && seqMatch && idMatch && lenMatch) {
                    patternOk = diag_verify_payload(rxBuffer + DIAG_HEADER_SIZE, hdr->payload_len, hdr->seq_num);
                }

                // Acceptance condition must include patternOk
                bool validPong = crcMatch && dirMatch && seqMatch && idMatch && lenMatch && patternOk;

                if (validPong) {
                    uint32_t turnaroundUs = (uint32_t)hdr->turnaround_or_time;
                    uint32_t totalElapsed = t4Us - modeCPingStartUs;
                    uint32_t rttUs = (totalElapsed > turnaroundUs) ? (totalElapsed - turnaroundUs) : totalElapsed;
                    uint32_t approxOneWayUs = rttUs / 2;

                    statRxValidFrames++;

                    LogEvent ev = {};
                    ev.type = EventType::MODE_C_RTT;
                    ev.seq = hdr->seq_num;
                    ev.payload_len = hdr->payload_len;
                    ev.crc_ok = true;
                    ev.pattern_ok = true;
                    ev.t1_us = modeCPingStartUs;
                    ev.t4_us = t4Us;
                    ev.turnaround_us = turnaroundUs;
                    ev.rtt_us = rttUs;
                    ev.approx_one_way_us = approxOneWayUs;
                    enqueueEvent(ev);
                } else if (crcMatch && dirMatch && seqMatch && idMatch && lenMatch && !patternOk) {
                    statRxPatternErrors++;
                    LogEvent ev = {};
                    ev.type = EventType::MODE_C_INVALID_PATTERN;
                    ev.seq = hdr->seq_num;
                    ev.payload_len = hdr->payload_len;
                    enqueueEvent(ev);
                } else {
                    statRxCrcErrors++;
                }

                resetRxParser();
                modeCWaitingPong = false;
                framesSent++;
                break;
            }
        }
        return;
    }

    // Ready to send next Ping
    radio.waitForReady(100);

    if (!diag_is_valid_frame_len(activePayloadLen)) {
        return;
    }

    uint8_t txFrame[DIAG_BUFFER_SIZE];
    RawDiagHeader* hdr = (RawDiagHeader*)txFrame;
    hdr->magic0 = DIAG_MAGIC_0;
    hdr->magic1 = DIAG_MAGIC_1;
    hdr->test_id = activeTestId;
    hdr->direction = DIAG_DIR_PING;
    hdr->seq_num = framesSent;
    hdr->payload_len = activePayloadLen;
    hdr->turnaround_or_time = (uint16_t)(millis() & 0xFFFF);

    // Keep payload pattern 100% valid
    uint8_t* payloadPtr = txFrame + DIAG_HEADER_SIZE;
    diag_fill_payload(payloadPtr, activePayloadLen, framesSent);

    size_t dataLen = DIAG_HEADER_SIZE + activePayloadLen;
    uint16_t crc = diag_crc16(txFrame, dataLen);
    txFrame[dataLen]     = (uint8_t)(crc & 0xFF);
    txFrame[dataLen + 1] = (uint8_t)((crc >> 8) & 0xFF);
    size_t totalFrameLen = dataLen + DIAG_CRC_SIZE;

    modeCPingStartUs = micros();
    radio.write(txFrame, totalFrameLen);
    radio.flush();

    rxBytesCount = 0;
    rxExpectedTotal = 0;
    modeCWaitingPong = true;
}

// =============================================================================
// MODE C SLAVE STEP (Air awaits Ping, replies with Pong + turnaround in header)
// =============================================================================
static void runModeCSlaveStep() {
    while (radio.available()) {
        uint8_t b = (uint8_t)radio.read();
        if (rxBytesCount == 0 && b != DIAG_MAGIC_0) continue;
        if (rxBytesCount == 1 && b != DIAG_MAGIC_1) { resetRxParser(); continue; }

        // Buffer boundary protection
        if (rxBytesCount >= DIAG_BUFFER_SIZE) {
            reportMalformedFrame(0);
            resetRxParser();
            continue;
        }

        if (rxBytesCount < DIAG_BUFFER_SIZE) {
            rxBuffer[rxBytesCount++] = b;
        } else {
            reportMalformedFrame(0);
            resetRxParser();
            continue;
        }

        if (rxBytesCount == DIAG_HEADER_SIZE) {
            RawDiagHeader* hdr = (RawDiagHeader*)rxBuffer;
            // Validate payload_len before computing expected total
            if (!diag_is_valid_frame_len(hdr->payload_len)) {
                reportMalformedFrame(hdr->payload_len);
                resetRxParser();
                continue;
            }
            size_t expTotal = diag_total_frame_len(hdr->payload_len);
            if (expTotal > DIAG_BUFFER_SIZE) {
                reportMalformedFrame(hdr->payload_len);
                resetRxParser();
                continue;
            }
            rxExpectedTotal = expTotal;
        }

        if (rxExpectedTotal > 0 && rxBytesCount == rxExpectedTotal) {
            uint32_t t2Us = micros();
            RawDiagHeader* rxHdr = (RawDiagHeader*)rxBuffer;
            size_t dataLen = DIAG_HEADER_SIZE + rxHdr->payload_len;

            uint16_t computedCrc = diag_crc16(rxBuffer, dataLen);
            uint16_t receivedCrc = (uint16_t)rxBuffer[dataLen] | ((uint16_t)rxBuffer[dataLen + 1] << 8);

            bool patternOk = false;
            if (computedCrc == receivedCrc && rxHdr->direction == DIAG_DIR_PING) {
                patternOk = diag_verify_payload(rxBuffer + DIAG_HEADER_SIZE, rxHdr->payload_len, rxHdr->seq_num);
            }

            if (computedCrc == receivedCrc && rxHdr->direction == DIAG_DIR_PING && patternOk) {
                // Build and send Pong
                uint8_t txFrame[DIAG_BUFFER_SIZE];
                RawDiagHeader* txHdr = (RawDiagHeader*)txFrame;
                txHdr->magic0 = DIAG_MAGIC_0;
                txHdr->magic1 = DIAG_MAGIC_1;
                txHdr->test_id = rxHdr->test_id;
                txHdr->direction = DIAG_DIR_PONG;
                txHdr->seq_num = rxHdr->seq_num;
                txHdr->payload_len = rxHdr->payload_len;

                // Turnaround stored in dedicated 16-bit header field
                uint32_t t3Us = micros();
                uint32_t turnaroundUs = t3Us - t2Us;
                txHdr->turnaround_or_time = (uint16_t)(turnaroundUs > 65535 ? 65535 : turnaroundUs);

                // Preserve 100% deterministic pattern in payload (NEVER overwritten!)
                uint8_t* payloadPtr = txFrame + DIAG_HEADER_SIZE;
                diag_fill_payload(payloadPtr, rxHdr->payload_len, rxHdr->seq_num);

                size_t pongDataLen = DIAG_HEADER_SIZE + txHdr->payload_len;
                uint16_t pongCrc = diag_crc16(txFrame, pongDataLen);
                txFrame[pongDataLen]     = (uint8_t)(pongCrc & 0xFF);
                txFrame[pongDataLen + 1] = (uint8_t)((pongCrc >> 8) & 0xFF);

                radio.waitForReady(100);
                radio.write(txFrame, pongDataLen + DIAG_CRC_SIZE);
                radio.flush();

                LogEvent ev = {};
                ev.type = EventType::MODE_C_PONG_SENT;
                ev.seq = rxHdr->seq_num;
                ev.turnaround_us = turnaroundUs;
                enqueueEvent(ev);
            } else if (computedCrc == receivedCrc && rxHdr->direction == DIAG_DIR_PING && !patternOk) {
                statRxPatternErrors++;
            } else {
                statRxCrcErrors++;
            }

            resetRxParser();
            break;
        }
    }
}

// Validate requested TX parameters before assigning or changing mode
static bool validateTxCommandArgs(long plen, long count, long pace) {
    if (plen < 0 || plen > 255 || !diag_is_supported_payload_len((uint8_t)plen) || !diag_is_valid_frame_len((uint8_t)plen)) {
        Serial.print(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_PAYLOAD_LENGTH\",\"payload_len\":"));
        Serial.print(plen);
        Serial.println(F("}"));
        return false;
    }
    if (count <= 0 || count > 65535) {
        Serial.print(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_FRAME_COUNT\",\"count\":"));
        Serial.print(count);
        Serial.println(F("}"));
        return false;
    }
    if (pace <= 0 || pace > 60000) {
        Serial.print(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_PACING\",\"pacing_ms\":"));
        Serial.print(pace);
        Serial.println(F("}"));
        return false;
    }
    return true;
}

// =============================================================================
// PHASE 2 LIVE RC ADAPTER IMPLEMENTATION
// =============================================================================
#if defined(ESP32)
static HardwareSerial SerialCRSF(1);
static int crsfRxPin = 13;
static int crsfTxPin = 14;
static uint32_t crsfBaud = 420000;
static bool crsfInvert = false;
static bool crsfConfigLocked = false;
static uint32_t lastScanToggleMs = 0;
static uint8_t scanState = 0;

static volatile uint32_t edgeCount13 = 0;
static volatile uint32_t edgeCount14 = 0;
static void IRAM_ATTR isrEdge13() { edgeCount13++; }
static void IRAM_ATTR isrEdge14() { edgeCount14++; }

static uint32_t rawBytesTotal = 0;
static uint8_t rawSample[128];
static size_t rawSampleLen = 0;
#endif

static RcGroundAdapter rcGroundAdapter;
static uint16_t rcTransportSeq = 0;
static uint32_t lastRcTxMs = 0;
static uint32_t lastRcLogMs = 0;

static void startRcGround() {
#if defined(ESP32)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    edgeCount13 = 0;
    edgeCount14 = 0;
    attachInterrupt(digitalPinToInterrupt(13), isrEdge13, CHANGE);
    attachInterrupt(digitalPinToInterrupt(14), isrEdge14, CHANGE);

    rawBytesTotal = 0;
    rawSampleLen = 0;
    crsfConfigLocked = false;
    scanState = 0;
    crsfRxPin = 13;
    crsfTxPin = 14;
    crsfBaud = 400000;
    crsfInvert = true;
    SerialCRSF.begin(crsfBaud, SERIAL_8N1, crsfRxPin, crsfTxPin, crsfInvert);
    lastScanToggleMs = millis();
#endif
    currentMode = DiagMode::MODE_RC_GROUND;
    Serial.println(F("{\"event\":\"STARTED_RC_GROUND\",\"baud\":400000,\"rx_pin\":13,\"tx_pin\":14,\"invert\":true}"));
}

static void runRcGroundStep() {
#if defined(ESP32)
    while (SerialCRSF.available()) {
        uint8_t b = (uint8_t)SerialCRSF.read();
        rawBytesTotal++;
        if (rawSampleLen < sizeof(rawSample)) {
            rawSample[rawSampleLen++] = b;
        }
        rcGroundAdapter.feed_byte(b);
    }

    // If valid frames start arriving, LOCK configuration immediately!
    if (rcGroundAdapter.get_stats().crsf_frames_in > 0) {
        if (!crsfConfigLocked) {
            crsfConfigLocked = true;
            Serial.print(F("{\"event\":\"CRSF_LOCKED\",\"rx_pin\":"));
            Serial.print(crsfRxPin);
            Serial.print(F(",\"baud\":"));
            Serial.print(crsfBaud);
            Serial.print(F(",\"invert\":"));
            Serial.print(crsfInvert ? F("true") : F("false"));
            Serial.println(F("}"));
        }
    } else if (!crsfConfigLocked) {
        // Targeted scan table prioritized for active pin (Pin 13) with inverted first
        uint32_t nowScan = millis();
        if (nowScan - lastScanToggleMs > 1200) {
            lastScanToggleMs = nowScan;
            static const uint32_t SCAN_BAUDS[] = {400000, 420000, 115200, 400000, 420000, 115200};
            static const bool     SCAN_INVS[]  = {true,   true,   true,   false,  false,  false};
            scanState = (scanState + 1) % 6;
            crsfRxPin = 13;
            crsfTxPin = 14;
            crsfBaud = SCAN_BAUDS[scanState];
            crsfInvert = SCAN_INVS[scanState];

            SerialCRSF.end();
            SerialCRSF.begin(crsfBaud, SERIAL_8N1, crsfRxPin, crsfTxPin, crsfInvert);

            Serial.print(F("{\"event\":\"CRSF_SCAN_TRY\",\"rx_pin\":"));
            Serial.print(crsfRxPin);
            Serial.print(F(",\"baud\":"));
            Serial.print(crsfBaud);
            Serial.print(F(",\"invert\":"));
            Serial.print(crsfInvert ? F("true") : F("false"));
            Serial.println(F("}"));
        }
    }
#endif

    uint32_t now = millis();
    // Transmit RF frame every 20ms (~50 Hz) or whenever a new frame arrives
    if ((now - lastRcTxMs >= 20 || rcGroundAdapter.has_new_frame()) && (now - lastRcTxMs >= 10)) {
        lastRcTxMs = now;
        uint8_t wire_buf[64];
        size_t len = rcGroundAdapter.get_uplink_frame(wire_buf, rcTransportSeq++);
        if (len > 0) {
            radio.waitForReady(10);
            radio.write(wire_buf, len);
        }
    }

    // Log live channel updates and raw sniffer stats over USB Serial every 100ms (10 Hz)
    if (now - lastRcLogMs >= 100) {
        lastRcLogMs = now;
#if defined(ESP32)
        uint32_t e13 = edgeCount13; edgeCount13 = 0;
        uint32_t e14 = edgeCount14; edgeCount14 = 0;
        uint8_t p13_lvl = digitalRead(13);
        uint8_t p14_lvl = digitalRead(14);
#else
        uint32_t e13 = 0, e14 = 0;
        uint8_t p13_lvl = 0, p14_lvl = 0;
        uint32_t rawBytesTotal = 0;
        size_t rawSampleLen = 0;
        uint8_t* rawSample = nullptr;
        int crsfRxPin = 0;
        uint32_t crsfBaud = 0;
        bool crsfInvert = false;
#endif
        const uint16_t* ch = rcGroundAdapter.get_channels();

        Serial.print(F("{\"event\":\"RC_GROUND_IN\",\"frames\":"));
        Serial.print(rcGroundAdapter.get_stats().crsf_frames_in);
        Serial.print(F(",\"rf_tx\":"));
        Serial.print(rcGroundAdapter.get_stats().rf_frames_sent);
        Serial.print(F(",\"crc_err\":"));
        Serial.print(rcGroundAdapter.get_stats().crsf_crc_errors);
        Serial.print(F(",\"raw_bytes\":"));
        Serial.print(rawBytesTotal);
        Serial.print(F(",\"p13_lvl\":"));
        Serial.print(p13_lvl);
        Serial.print(F(",\"p13_edges\":"));
        Serial.print(e13);
        Serial.print(F(",\"p14_lvl\":"));
        Serial.print(p14_lvl);
        Serial.print(F(",\"p14_edges\":"));
        Serial.print(e14);
        Serial.print(F(",\"rx_pin\":"));
        Serial.print(crsfRxPin);
        Serial.print(F(",\"baud\":"));
        Serial.print(crsfBaud);
        Serial.print(F(",\"invert\":"));
        Serial.print(crsfInvert ? F("true") : F("false"));
        Serial.print(F(",\"hex\":\""));
#if defined(ESP32)
        for (size_t i = 0; i < rawSampleLen; i++) {
            if (rawSample[i] < 0x10) Serial.print('0');
            Serial.print(rawSample[i], HEX);
            if (i + 1 < rawSampleLen) Serial.print(' ');
        }
        rawSampleLen = 0;
#endif
        Serial.print(F("\",\"ch\":["));
        for (int i = 0; i < 16; ++i) {
            Serial.print(ch[i]);
            if (i < 15) Serial.print(F(","));
        }
        Serial.println(F("]}"));
    }
}

// -----------------------------------------------------------------------------
// AIR RC IMPLEMENTATION
// -----------------------------------------------------------------------------
static RcAirAdapter rcAirAdapter;
static TransportParser airRcParser(TransportNodeRole::AIR);
static uint32_t lastAirRcLogMs = 0;

static void startRcAir() {
    currentMode = DiagMode::MODE_RC_AIR;
    Serial.println(F("{\"event\":\"STARTED_RC_AIR\"}"));
}

static void runRcAirStep() {
    uint32_t now = millis();

    // Read incoming RF bytes from E22 radio
    while (radio.available()) {
        uint8_t b = (uint8_t)radio.read();
        if (airRcParser.feed_byte(b, now)) {
            const TransportHeader& hdr = airRcParser.get_header();
            if (hdr.channel == (uint8_t)TransportChannel::RC_CONTROL &&
                hdr.payload_length == sizeof(TransportPackedRc)) {
                const TransportPackedRc* prc = reinterpret_cast<const TransportPackedRc*>(airRcParser.get_payload());
                rcAirAdapter.ingest_rc_frame(*prc, now);
            }
        }
    }

    rcAirAdapter.update(now);

    // Log live channel telemetry over USB CDC every 50ms (20 Hz)
    if (now - lastAirRcLogMs >= 50) {
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
}

// =============================================================================
// HOST COMMAND PROCESSOR
// =============================================================================
static void processHostCommand(const char* cmd) {
    if (strncmp(cmd, "CMD:PING", 8) == 0) {
        Serial.print(F("{\"event\":\"PONG\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.print(F("\",\"modem_usable\":"));
        Serial.print(modemUsable ? F("true") : F("false"));
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:REG_READ", 12) == 0) {
        uint8_t regBuf[16] = {0};
        size_t readLen = 0;
        E22RegReadStatus st = radio.readRegistersReadOnly(regBuf, sizeof(regBuf), readLen);

        Serial.print(F("{\"event\":\"REG_READ\",\"role\":\""));
        Serial.print(ROLE_STR);

        if (st == E22RegReadStatus::OK) {
            Serial.print(F("\",\"ok\":true,\"status\":\"OK\",\"len\":"));
            Serial.print(readLen);
            Serial.print(F(",\"bytes_hex\":\""));
            for (size_t i = 0; i < readLen; i++) {
                if (regBuf[i] < 0x10) Serial.print('0');
                Serial.print(regBuf[i], HEX);
            }
            Serial.print(F("\""));
            if (readLen >= 10) {
                Serial.print(F(",\"addh\":\"0x"));
                if (regBuf[3] < 0x10) Serial.print('0');
                Serial.print(regBuf[3], HEX);
                Serial.print(F("\",\"addl\":\"0x"));
                if (regBuf[4] < 0x10) Serial.print('0');
                Serial.print(regBuf[4], HEX);
                Serial.print(F("\",\"netid\":\"0x"));
                if (regBuf[5] < 0x10) Serial.print('0');
                Serial.print(regBuf[5], HEX);
                Serial.print(F("\",\"reg0\":\"0x"));
                if (regBuf[6] < 0x10) Serial.print('0');
                Serial.print(regBuf[6], HEX);
                Serial.print(F("\",\"reg1\":\"0x"));
                if (regBuf[7] < 0x10) Serial.print('0');
                Serial.print(regBuf[7], HEX);
                Serial.print(F("\",\"reg2\":\"0x"));
                if (regBuf[8] < 0x10) Serial.print('0');
                Serial.print(regBuf[8], HEX);
                Serial.print(F("\",\"reg3\":\"0x"));
                if (regBuf[9] < 0x10) Serial.print('0');
                Serial.print(regBuf[9], HEX);
                Serial.print(F("\""));
            }
        } else if (st == E22RegReadStatus::REGISTERS_UNVERIFIED) {
            Serial.print(F("\",\"ok\":false,\"status\":\"REGISTERS_UNVERIFIED\",\"len\":"));
            Serial.print(readLen);
            Serial.print(F(",\"bytes_hex\":\""));
            for (size_t i = 0; i < readLen; i++) {
                if (regBuf[i] < 0x10) Serial.print('0');
                Serial.print(regBuf[i], HEX);
            }
            Serial.print(F("\""));
        } else if (st == E22RegReadStatus::MODE_RESTORE_FAILED) {
            modemUsable = false;
            Serial.print(F("\",\"ok\":false,\"status\":\"MODE_RESTORE_FAILED\",\"modem_usable\":false"));
        } else if (st == E22RegReadStatus::UART_RESTORE_FAILED) {
            modemUsable = false;
            Serial.print(F("\",\"ok\":false,\"status\":\"UART_RESTORE_FAILED\",\"modem_usable\":false"));
        } else {
            Serial.print(F("\",\"ok\":false,\"status\":\"REGISTER_READ_FAILED\""));
        }
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:MODE_A_TX", 13) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        long plen;
        long count;
        long pace;
        if (sscanf(cmd + 13, "%ld %ld %ld", &plen, &count, &pace) != 3) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_COMMAND_ARGUMENTS\"}"));
            return;
        }
        if (!validateTxCommandArgs(plen, count, pace)) {
            return;
        }
        activePayloadLen = (uint8_t)plen;
        targetFrameCount = (uint16_t)count;
        pacingIntervalMs = (uint32_t)pace;
        framesSent = 0;
        statTxAttempts = 0;
        statTxSuccess = 0;
        statTxDeferred = 0;
        statOverruns = 0;
        activeTestId++;
        currentMode = DiagMode::MODE_A_TX;

        Serial.print(F("{\"event\":\"STARTED_MODE_A_TX\",\"payload_len\":"));
        Serial.print(activePayloadLen);
        Serial.print(F(",\"count\":"));
        Serial.print(targetFrameCount);
        Serial.print(F(",\"pacing_ms\":"));
        Serial.print(pacingIntervalMs);
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:MODE_A_RX", 13) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        resetRxParser();
        statRxValidFrames = 0;
        statRxCrcErrors = 0;
        statRxPatternErrors = 0;
        statRxMalformedFrames = 0;
        statRxTimeouts = 0;
        statRxGaps = 0;
        hasReceivedSeq = false;
        currentMode = DiagMode::MODE_A_RX;

        Serial.println(F("{\"event\":\"STARTED_MODE_A_RX\",\"note\":\"Firmware silent, strictly receiving\"}"));
    }
    else if (strncmp(cmd, "CMD:MODE_B_TX", 13) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        long plen;
        long count;
        long pace;
        if (sscanf(cmd + 13, "%ld %ld %ld", &plen, &count, &pace) != 3) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_COMMAND_ARGUMENTS\"}"));
            return;
        }
        if (!validateTxCommandArgs(plen, count, pace)) {
            return;
        }
        activePayloadLen = (uint8_t)plen;
        targetFrameCount = (uint16_t)count;
        pacingIntervalMs = (uint32_t)pace;
        framesSent = 0;
        statTxAttempts = 0;
        statTxSuccess = 0;
        statTxDeferred = 0;
        statOverruns = 0;
        activeTestId++;
        currentMode = DiagMode::MODE_B_TX;

        Serial.print(F("{\"event\":\"STARTED_MODE_B_TX\",\"payload_len\":"));
        Serial.print(activePayloadLen);
        Serial.print(F(",\"count\":"));
        Serial.print(targetFrameCount);
        Serial.print(F(",\"pacing_ms\":"));
        Serial.print(pacingIntervalMs);
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:MODE_B_RX", 13) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        resetRxParser();
        statRxValidFrames = 0;
        statRxCrcErrors = 0;
        statRxPatternErrors = 0;
        statRxMalformedFrames = 0;
        statRxTimeouts = 0;
        statRxGaps = 0;
        hasReceivedSeq = false;
        currentMode = DiagMode::MODE_B_RX;

        Serial.println(F("{\"event\":\"STARTED_MODE_B_RX\",\"note\":\"Firmware silent, strictly receiving\"}"));
    }
    else if (strncmp(cmd, "CMD:MODE_C_MASTER", 17) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        long plen;
        long count;
        long pace;
        if (sscanf(cmd + 17, "%ld %ld %ld", &plen, &count, &pace) != 3) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"INVALID_COMMAND_ARGUMENTS\"}"));
            return;
        }
        if (!validateTxCommandArgs(plen, count, pace)) {
            return;
        }
        activePayloadLen = (uint8_t)plen;
        targetFrameCount = (uint16_t)count;
        pacingIntervalMs = (uint32_t)pace;
        framesSent = 0;
        statRxValidFrames = 0;
        statRxCrcErrors = 0;
        statRxPatternErrors = 0;
        statRxMalformedFrames = 0;
        modeCWaitingPong = false;
        activeTestId++;
        currentMode = DiagMode::MODE_C_MASTER;

        Serial.print(F("{\"event\":\"STARTED_MODE_C_MASTER\",\"payload_len\":"));
        Serial.print(activePayloadLen);
        Serial.print(F(",\"count\":"));
        Serial.print(targetFrameCount);
        Serial.print(F(",\"pacing_ms\":"));
        Serial.print(pacingIntervalMs);
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:MODE_C_SLAVE", 16) == 0) {
        if (!modemUsable) {
            Serial.println(F("{\"event\":\"ERROR\",\"reason\":\"MODEM_UNUSABLE\"}"));
            return;
        }
        resetRxParser();
        currentMode = DiagMode::MODE_C_SLAVE;
        Serial.println(F("{\"event\":\"STARTED_MODE_C_SLAVE\"}"));
    }
    else if (strncmp(cmd, "CMD:RC_START_GROUND", 19) == 0) {
        startRcGround();
    }
    else if (strncmp(cmd, "CMD:RC_START_AIR", 16) == 0) {
        startRcAir();
    }
    else if (strncmp(cmd, "CMD:RC_START", 12) == 0) {
        if (LOCAL_ROLE_ID == 1) {
            startRcGround();
        } else {
            startRcAir();
        }
    }
    else if (strncmp(cmd, "CMD:RC_STATS", 12) == 0) {
        if (LOCAL_ROLE_ID == 1) {
            const RcAdapterStats& s = rcGroundAdapter.get_stats();
            Serial.print(F("{\"event\":\"RC_STATS\",\"role\":\"GROUND\",\"crsf_in\":"));
            Serial.print(s.crsf_frames_in);
            Serial.print(F(",\"rf_tx\":"));
            Serial.print(s.rf_frames_sent);
            Serial.print(F(",\"crc_err\":"));
            Serial.print(s.crsf_crc_errors);
            Serial.println(F("}"));
        } else {
            const RcAdapterStats& s = rcAirAdapter.get_stats();
            Serial.print(F("{\"event\":\"RC_STATS\",\"role\":\"AIR\",\"rf_rx\":"));
            Serial.print(s.rf_frames_received);
            Serial.print(F(",\"failsafe\":"));
            Serial.print(rcAirAdapter.is_failsafe_active() ? F("true") : F("false"));
            Serial.print(F(",\"fs_events\":"));
            Serial.print(s.failsafe_events);
            Serial.print(F(",\"restore_events\":"));
            Serial.print(s.restore_events);
            Serial.println(F("}"));
        }
    }
    else if (strncmp(cmd, "CMD:SET_CRSF_PIN", 16) == 0) {
#if defined(ESP32)
        int pin = 13;
        if (sscanf(cmd + 16, "%d", &pin) == 1 && (pin == 13 || pin == 14)) {
            crsfRxPin = pin;
            crsfTxPin = (pin == 13) ? 14 : 13;
            crsfConfigLocked = true;
            SerialCRSF.end();
            SerialCRSF.begin(crsfBaud, SERIAL_8N1, crsfRxPin, crsfTxPin, crsfInvert);
            Serial.print(F("{\"event\":\"CRSF_CONFIG_SET\",\"rx_pin\":"));
            Serial.print(crsfRxPin);
            Serial.print(F(",\"tx_pin\":"));
            Serial.print(crsfTxPin);
            Serial.println(F("}"));
        }
#endif
    }
    else if (strncmp(cmd, "CMD:SET_CRSF_BAUD", 17) == 0) {
#if defined(ESP32)
        long b = 400000;
        if (sscanf(cmd + 17, "%ld", &b) == 1 && (b >= 9600 && b <= 1000000)) {
            crsfBaud = (uint32_t)b;
            crsfConfigLocked = true;
            SerialCRSF.end();
            SerialCRSF.begin(crsfBaud, SERIAL_8N1, crsfRxPin, crsfTxPin, crsfInvert);
            Serial.print(F("{\"event\":\"CRSF_CONFIG_SET\",\"baud\":"));
            Serial.print(crsfBaud);
            Serial.println(F("}"));
        }
#endif
    }
    else if (strncmp(cmd, "CMD:SET_CRSF_INVERT", 19) == 0) {
#if defined(ESP32)
        int inv = 0;
        if (sscanf(cmd + 19, "%d", &inv) == 1) {
            crsfInvert = (inv != 0);
            crsfConfigLocked = true;
            SerialCRSF.end();
            SerialCRSF.begin(crsfBaud, SERIAL_8N1, crsfRxPin, crsfTxPin, crsfInvert);
            Serial.print(F("{\"event\":\"CRSF_CONFIG_SET\",\"invert\":"));
            Serial.print(crsfInvert ? F("true") : F("false"));
            Serial.println(F("}"));
        }
#endif
    }
    else if (strncmp(cmd, "CMD:MEASURE_PULSES", 18) == 0) {
#if defined(ESP32)
        SerialCRSF.end();
        pinMode(13, INPUT_PULLUP);
        pinMode(14, INPUT_PULLUP);
        int activePin = 13;
        if (digitalRead(14) == LOW || edgeCount14 > edgeCount13) {
            activePin = 14;
        }
        uint32_t pulses[32];
        size_t count = 0;
        int lastLevel = digitalRead(activePin);
        uint32_t lastChangeUs = micros();
        uint32_t tStart = millis();
        while (count < 32 && (millis() - tStart < 1000)) {
            int lvl = digitalRead(activePin);
            if (lvl != lastLevel) {
                uint32_t nowUs = micros();
                pulses[count++] = nowUs - lastChangeUs;
                lastChangeUs = nowUs;
                lastLevel = lvl;
            }
        }
        Serial.print(F("{\"event\":\"PULSE_MEASURE\",\"pin\":"));
        Serial.print(activePin);
        Serial.print(F(",\"count\":"));
        Serial.print(count);
        Serial.print(F(",\"durations_us\":["));
        for (size_t i = 0; i < count; i++) {
            Serial.print(pulses[i]);
            if (i + 1 < count) Serial.print(F(","));
        }
        Serial.println(F("]}"));
#endif
    }
    else if (strncmp(cmd, "CMD:CAPTURE_CRSF_FRAME", 22) == 0) {
#if defined(ESP32)
        SerialCRSF.end();
        SerialCRSF.begin(420000, SERIAL_8N1, 13, 14, false);
        while (SerialCRSF.available()) SerialCRSF.read();

        // Wait for an idle gap of at least 1500us
        uint32_t tWait = millis();
        while (millis() - tWait < 500) {
            if (SerialCRSF.available()) {
                SerialCRSF.read();
                tWait = millis();
            }
        }

        // Wait for the first byte of a new burst
        uint32_t tWaitByte = millis();
        while (!SerialCRSF.available() && (millis() - tWaitByte < 1000)) {
            delayMicroseconds(50);
        }

        uint8_t captured[64];
        size_t capCount = 0;
        uint32_t tReadStart = millis();
        while (capCount < 64 && (millis() - tReadStart < 20)) {
            if (SerialCRSF.available()) {
                captured[capCount++] = (uint8_t)SerialCRSF.read();
            } else {
                delayMicroseconds(50);
            }
        }

        Serial.print(F("{\"event\":\"CAPTURED_FRAME\",\"count\":"));
        Serial.print(capCount);
        Serial.print(F(",\"bytes_hex\":\""));
        for (size_t i = 0; i < capCount; i++) {
            if (captured[i] < 0x10) Serial.print('0');
            Serial.print(captured[i], HEX);
            if (i + 1 < capCount) Serial.print(' ');
        }
        Serial.print(F("\""));
        if (capCount >= 4) {
            uint8_t addr = captured[0];
            uint8_t len = captured[1];
            uint8_t type = captured[2];
            if (len >= 2 && (size_t)(len + 2) <= capCount) {
                uint8_t computedCrc = crsf_crc8(&captured[2], len - 1);
                uint8_t rxCrc = captured[1 + len];
                Serial.print(F(",\"addr\":\"0x"));
                Serial.print(addr, HEX);
                Serial.print(F("\",\"len\":"));
                Serial.print(len);
                Serial.print(F(",\"type\":\"0x"));
                Serial.print(type, HEX);
                Serial.print(F("\",\"computed_crc\":\"0x"));
                Serial.print(computedCrc, HEX);
                Serial.print(F("\",\"rx_crc\":\"0x"));
                Serial.print(rxCrc, HEX);
                Serial.print(F("\",\"crc_match\":"));
                Serial.print((computedCrc == rxCrc) ? F("true") : F("false"));
            }
        }
        Serial.println(F("}"));
#endif
    }
    else if (strncmp(cmd, "CMD:STOP", 8) == 0 || strncmp(cmd, "CMD:IDLE", 8) == 0 || strncmp(cmd, "CMD:RC_STOP", 11) == 0) {
#if defined(ESP32)
        if (currentMode == DiagMode::MODE_RC_GROUND) {
            SerialCRSF.end();
            detachInterrupt(digitalPinToInterrupt(13));
            detachInterrupt(digitalPinToInterrupt(14));
        }
#endif
        currentMode = DiagMode::IDLE;
        digitalWrite(PIN_LED_BUILTIN, LED_PIN_OFF);
        Serial.println(F("{\"event\":\"STOPPED\",\"state\":\"IDLE\"}"));
    }
    else if (strncmp(cmd, "CMD:SET_TIMEOUT", 15) == 0) {
        int timeoutMs = 30;
        sscanf(cmd + 15, "%d", &timeoutMs);
        parserTimeoutUs = (uint32_t)timeoutMs * 1000UL;
        Serial.print(F("{\"event\":\"TIMEOUT_UPDATED\",\"parser_timeout_ms\":"));
        Serial.print(timeoutMs);
        Serial.println(F("}"));
    }
    else if (strncmp(cmd, "CMD:STATS", 9) == 0) {
        Serial.print(F("{\"event\":\"STATS\",\"role\":\""));
        Serial.print(ROLE_STR);
        Serial.print(F("\",\"tx_attempts\":"));
        Serial.print(statTxAttempts);
        Serial.print(F(",\"tx_success\":"));
        Serial.print(statTxSuccess);
        Serial.print(F(",\"tx_deferred\":"));
        Serial.print(statTxDeferred);
        Serial.print(F(",\"overruns\":"));
        Serial.print(statOverruns);
        Serial.print(F(",\"rx_valid\":"));
        Serial.print(statRxValidFrames);
        Serial.print(F(",\"rx_crc_err\":"));
        Serial.print(statRxCrcErrors);
        Serial.print(F(",\"rx_pattern_err\":"));
        Serial.print(statRxPatternErrors);
        Serial.print(F(",\"rx_malformed\":"));
        Serial.print(statRxMalformedFrames);
        Serial.print(F(",\"rx_timeouts\":"));
        Serial.print(statRxTimeouts);
        Serial.print(F(",\"rx_gaps\":"));
        Serial.print(statRxGaps);
        Serial.println(F("}"));
    }
    else {
        Serial.print(F("{\"event\":\"UNKNOWN_CMD\",\"cmd\":\""));
        Serial.print(cmd);
        Serial.println(F("\"}"));
    }
}

// =============================================================================
// MANUAL ONBOARD BUTTON TRIGGER (Optional Standalone Bench Testing)
// =============================================================================
static uint32_t lastButtonPressMs = 0;

static void handleButtonPress() {
    if (!modemUsable) return;

    if (digitalRead(PIN_USER_KEY) == LOW) {
        if (millis() - lastButtonPressMs > 500) {
            lastButtonPressMs = millis();
            if (currentMode == DiagMode::IDLE) {
                if (LOCAL_ROLE_ID == 0) {
                    // Air default: Mode A TX (53B, 1000 frames, 100ms conservative pacing)
                    activePayloadLen = 53;
                    targetFrameCount = 1000;
                    pacingIntervalMs = 100;
                    framesSent = 0;
                    currentMode = DiagMode::MODE_A_TX;
                    Serial.println(F("{\"event\":\"BUTTON_TRIGGER\",\"mode\":\"MODE_A_TX\",\"plen\":53,\"count\":1000,\"pacing_ms\":100}"));
                } else {
                    // Ground default: Mode A RX
                    resetRxParser();
                    currentMode = DiagMode::MODE_A_RX;
                    Serial.println(F("{\"event\":\"BUTTON_TRIGGER\",\"mode\":\"MODE_A_RX\"}"));
                }
            } else {
                currentMode = DiagMode::IDLE;
                digitalWrite(PIN_LED_BUILTIN, LED_PIN_OFF);
                Serial.println(F("{\"event\":\"BUTTON_TRIGGER\",\"mode\":\"IDLE\"}"));
            }
        }
    }
}
