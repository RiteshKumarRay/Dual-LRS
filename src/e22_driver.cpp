#include "e22_driver.h"

E22Driver::E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux)
    : _serial(serialPort), _pinM0(pinM0), _pinM1(pinM1), _pinAux(pinAux), _currentMode(E22Mode::NORMAL) {}

void E22Driver::begin(uint32_t baudRate) {
    pinMode(_pinM0, OUTPUT);
    pinMode(_pinM1, OUTPUT);
    pinMode(_pinAux, INPUT_PULLUP);

    // Default to Normal Operating Mode
    setMode(E22Mode::NORMAL);

    _serial.begin(baudRate);
    waitForReady(200);
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
        case E22Mode::WOR_RECEIVE:
            digitalWrite(_pinM0, LOW);
            digitalWrite(_pinM1, HIGH);
            break;
        case E22Mode::SLEEP_CONFIG:
            digitalWrite(_pinM0, HIGH);
            digitalWrite(_pinM1, HIGH);
            break;
    }

    _currentMode = mode;
    // Ebyte datasheet specifies >= 2ms delay after mode switch
    delay(5);
    waitForReady(100);
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
