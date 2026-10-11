#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>
#include <vector>

using std::min;
using std::max;

#define HIGH 1
#define LOW 0

#define PC13 13
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA9 9
#define PA10 10
#define PB0 20
#define PB1 21
#define PB10 22

extern uint32_t g_mock_micros;
extern uint32_t g_mock_millis;

inline uint32_t micros() { return g_mock_micros; }
inline uint32_t millis() { return g_mock_millis; }

class Stream {
public:
    virtual ~Stream() = default;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(const uint8_t* data, size_t length) = 0;
};

class HardwareSerial : public Stream {
public:
    void begin(uint32_t) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(const uint8_t*, size_t length) override { return length; }
};
