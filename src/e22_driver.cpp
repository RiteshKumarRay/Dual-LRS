#include "e22_driver.h"

E22Driver::E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux)
    : _serial(serialPort), _pinM0(pinM0), _pinM1(pinM1), _pinAux(pinAux), _currentMode(E22Mode::NORMAL) {}

void E22Driver::begin(uint32_t baudRate) {
    pinMode(_pinM0, OUTPUT);
    pinMode(_pinM1, OUTPUT);
    pinMode(_pinAux, INPUT_PULLUP);

    // Auto-configure the Ebyte E22 hardware registers:
    // Sets UART to 115200 baud, Air Data Rate to 62.5 kbps, and RF Power to 21 dBm (125 mW)
    configureRadio(E22_ACTIVE_TX_POWER, 0x17);

    // Default to Normal Operating Mode
    setMode(E22Mode::NORMAL);

    _serial.end();
#if defined(ESP32)
    _serial.begin(baudRate, SERIAL_8N1, PIN_RADIO_RX, PIN_RADIO_TX);
#else
    _serial.begin(baudRate);
#endif
    waitForReady(200);
}

bool E22Driver::beginPassive(uint32_t baudRate) {
    pinMode(_pinM0, OUTPUT);
    pinMode(_pinM1, OUTPUT);
    pinMode(_pinAux, INPUT_PULLUP);

    // Passive init: Put radio into Normal Operating Mode (M0=0, M1=0)
    // NEVER writes to EEPROM, NEVER calls configureRadio(), NEVER sends 0xC0.
    digitalWrite(_pinM0, LOW);
    digitalWrite(_pinM1, LOW);
    _currentMode = E22Mode::NORMAL;

    // Datasheet mode settling time (>= 2ms)
    delay(5);
    if (!waitForReady(200)) {
        return false;
    }

    _serial.end();
#if defined(ESP32)
    _serial.begin(baudRate, SERIAL_8N1, PIN_RADIO_RX, PIN_RADIO_TX);
#else
    _serial.begin(baudRate);
#endif

    if (!waitForReady(200)) {
        return false;
    }

    return true;
}

void E22Driver::setMode(E22Mode mode) {
    // Wait until radio is done with pending tasks before switching modes
    waitForReady(100);

    switch (mode) {
        case E22Mode::NORMAL:
            digitalWrite(_pinM0, LOW);
            digitalWrite(_pinM1, LOW);
            break;
        case E22Mode::WOR_TRANSMIT:
            digitalWrite(_pinM0, HIGH);
            digitalWrite(_pinM1, LOW);
            break;
        case E22Mode::CONFIG:
            digitalWrite(_pinM0, LOW);
            digitalWrite(_pinM1, HIGH);
            break;
        case E22Mode::SLEEP:
            digitalWrite(_pinM0, HIGH);
            digitalWrite(_pinM1, HIGH);
            break;
    }

    _currentMode = mode;
    // Ebyte datasheet specifies >= 2ms delay after mode switch
    delay(5);
    waitForReady(100);
}

bool E22Driver::configureRadio(uint8_t powerLevel, uint8_t channel) {
    // Switch to Configuration Mode (M0=LOW, M1=HIGH)
    setMode(E22Mode::CONFIG);
    delay(40);
    waitForReady(200);

    // In config mode, E22 communicates at 9600 baud 8N1
    _serial.end();
#if defined(ESP32)
    _serial.begin(9600, SERIAL_8N1, PIN_RADIO_RX, PIN_RADIO_TX);
#else
    _serial.begin(9600);
#endif
    delay(40);

    while (_serial.available()) _serial.read();

    // Configuration packet:
    // C0: Write and save to EEPROM
    // 00: Starting address
    // 07: Length 7 bytes (covers ADDH, ADDL, NETID, REG0, REG1, REG2, REG3)
    // ADDH: 0x00, ADDL: 0x00, NETID: 0x00
    // REG0: 0xE7 (115200 baud, 8N1, 62.5k air data rate)
    // REG1: 0x80 | (powerLevel & 0x03) (64 byte sub-packet, 03 = 21dBm bench power, 00 = 30dBm flight power)
    // REG2: channel (default 0x17 = channel 23)
    // REG3: 0x00 (bit 7: RSSI byte disabled [0], bit 6: transparent mode [0], bit 4: LBT disabled [0])
    uint8_t cfgCmd[] = {
        0xC0, 0x00, 0x07,
        0x00, 0x00, 0x00,
        0xE7,
        (uint8_t)(0x80 | (powerLevel & 0x03)),
        channel,
        0x00
    };

    waitForReady(100);
    _serial.write(cfgCmd, sizeof(cfgCmd));
    _serial.flush();
    delay(50);

    uint8_t resp[16];
    size_t respLen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 300 && respLen < sizeof(resp)) {
        if (_serial.available()) {
            resp[respLen++] = (uint8_t)_serial.read();
        } else {
            delay(5);
        }
    }

    bool success = (respLen >= 4 && resp[0] == 0xC1);
    _configured = success;
    _lastRespLen = respLen;
    if (respLen > 0) {
        memcpy(_lastResp, resp, (respLen < sizeof(_lastResp) ? respLen : sizeof(_lastResp)));
    }

    _serial.end();

    // Switch back to Normal operating mode
    setMode(E22Mode::NORMAL);
    delay(40);

    return success;
}

E22RegReadStatus E22Driver::readRegistersReadOnly(uint8_t* dest, size_t maxLen, size_t& readLen) {
    readLen = 0;
    if (dest == nullptr || maxLen < 10) {
        return E22RegReadStatus::READ_FAILED;
    }

    // 1. Wait for AUX ready before switching mode
    if (!waitForReady(200)) {
        return E22RegReadStatus::READ_FAILED;
    }

    // 2. Switch to Configuration Mode (M0=LOW, M1=HIGH)
    setMode(E22Mode::CONFIG);
    delay(40);
    if (!waitForReady(200)) {
        setMode(E22Mode::NORMAL);
        return E22RegReadStatus::READ_FAILED;
    }

    // 3. In configuration mode, E22 communicates at 9600 baud 8N1
    _serial.end();
#if defined(ESP32)
    _serial.begin(9600, SERIAL_8N1, PIN_RADIO_RX, PIN_RADIO_TX);
#else
    _serial.begin(9600);
#endif
    delay(40);

    // 4. Drain any stale UART input
    while (_serial.available()) _serial.read();

    // 5. Send verified read command from E22-900T30D Section 7.1:
    // 0xC1: Read register command
    // 0x00: Starting address
    // 0x07: Length 7 bytes (ADDH, ADDL, NETID, REG0, REG1, REG2, REG3)
    const uint8_t readCmd[] = {0xC1, 0x00, 0x07};
    waitForReady(100);
    _serial.write(readCmd, sizeof(readCmd));
    _serial.flush();
    delay(50);

    // 6. Capture response (expected 10 bytes: 0xC1 0x00 0x07 + 7 registers)
    uint8_t resp[16] = {0};
    size_t respLen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 300 && respLen < sizeof(resp)) {
        if (_serial.available()) {
            resp[respLen++] = (uint8_t)_serial.read();
        } else {
            delay(5);
        }
    }

    // Determine read/verification status
    E22RegReadStatus readStatus = E22RegReadStatus::OK;
    if (respLen == 0) {
        readStatus = E22RegReadStatus::READ_FAILED;
    } else if (respLen < 10 || resp[0] != 0xC1 || resp[1] != 0x00 || resp[2] != 0x07) {
        readStatus = E22RegReadStatus::REGISTERS_UNVERIFIED;
        readLen = (respLen <= maxLen ? respLen : maxLen);
        memcpy(dest, resp, readLen);
    } else {
        readStatus = E22RegReadStatus::OK;
        readLen = (respLen <= maxLen ? respLen : maxLen);
        memcpy(dest, resp, readLen);
    }

    // 7. Safely restore Normal Mode
    _serial.end();
    setMode(E22Mode::NORMAL);
    delay(40);
    if (!waitForReady(200)) {
        return E22RegReadStatus::MODE_RESTORE_FAILED;
    }

    // 8. Safely restore standard modem baud
#if defined(ESP32)
    _serial.begin(RADIO_UART_BAUD, SERIAL_8N1, PIN_RADIO_RX, PIN_RADIO_TX);
#else
    _serial.begin(RADIO_UART_BAUD);
#endif
    delay(20);
    while (_serial.available()) _serial.read();

    if (!waitForReady(200)) {
        return E22RegReadStatus::UART_RESTORE_FAILED;
    }

    return readStatus;
}

bool E22Driver::isBusy() const {
    // Ebyte E22 AUX pin: LOW = Busy (transmitting or receiving over RF), HIGH = Ready
    return (digitalRead(_pinAux) == LOW);
}

bool E22Driver::waitForReady(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (isBusy()) {
        if (millis() - start >= timeoutMs) {
            return false; // Timed out waiting for AUX
        }
        delayMicroseconds(100);
    }
    return true;
}

size_t E22Driver::write(const uint8_t* data, size_t length) {
    if (length == 0 || data == nullptr) return 0;
    return _serial.write(data, length);
}

int E22Driver::available() {
    return _serial.available();
}

int E22Driver::read() {
    return _serial.read();
}

size_t E22Driver::readBytes(uint8_t* buffer, size_t length) {
    return _serial.readBytes(buffer, length);
}

void E22Driver::flush() {
    _serial.flush();
}
