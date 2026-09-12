#pragma once
#include <Arduino.h>
#include "config.h"

enum class E22Mode : uint8_t {
    NORMAL       = 0, // M0=0, M1=0 : RF and UART operational (Transparent)
    WOR_TRANSMIT = 1, // M0=1, M1=0 : WOR Transmit
    WOR_RECEIVE  = 2, // M0=0, M1=1 : WOR Receive
    SLEEP_CONFIG = 3  // M0=1, M1=1 : Sleep & Configuration mode (9600 8N1)
};

class E22Driver {
public:
    E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux);

    void begin(uint32_t baudRate = RADIO_UART_BAUD);
    void setMode(E22Mode mode);
    bool isBusy() const;
    bool waitForReady(uint32_t timeoutMs = 100);

    size_t write(const uint8_t* data, size_t length);
    int available();
    int read();
    size_t readBytes(uint8_t* buffer, size_t length);
    void flush();

private:
    HardwareSerial& _serial;
    uint8_t _pinM0;
    uint8_t _pinM1;
    uint8_t _pinAux;
    E22Mode _currentMode;
};
