#pragma once
#include <Arduino.h>
#include "config.h"

enum class E22Mode : uint8_t {
    NORMAL       = 0, // M0=0, M1=0 : RF and UART operational (Transparent)
    WOR_TRANSMIT = 1, // M0=1, M1=0 : WOR Transmit
    CONFIG       = 2, // M0=0, M1=1 : Configuration mode (9600 8N1)
    SLEEP        = 3  // M0=1, M1=1 : Deep sleep
};

class E22Driver {
public:
    E22Driver(HardwareSerial& serialPort, uint8_t pinM0, uint8_t pinM1, uint8_t pinAux);

    void begin(uint32_t baudRate = RADIO_UART_BAUD);
    void setMode(E22Mode mode);
    bool configureRadio(uint8_t powerLevel = E22_ACTIVE_TX_POWER, uint8_t channel = 0x17);
    bool isBusy() const;
    bool waitForReady(uint32_t timeoutMs = 100);

    size_t write(const uint8_t* data, size_t length);
    int available();
    int read();
    size_t readBytes(uint8_t* buffer, size_t length);
    void flush();
    bool isConfigured() const { return _configured; }
    const uint8_t* getConfigResponse() const { return _lastResp; }
    size_t getConfigResponseLen() const { return _lastRespLen; }

private:
    HardwareSerial& _serial;
    uint8_t _pinM0;
    uint8_t _pinM1;
    uint8_t _pinAux;
    E22Mode _currentMode;
    bool _configured = false;
    uint8_t _lastResp[16] = {0};
    size_t _lastRespLen = 0;
};
