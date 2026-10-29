#ifndef PIN_TEST_ARDUINO_H
#define PIN_TEST_ARDUINO_H
#include <stddef.h>
#include <stdint.h>

uint32_t millis();
uint32_t micros();

class Print
{
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t *data, size_t length)
    {
        size_t written = 0;
        while (written < length && write(data[written])) ++written;
        return written;
    }
};

class Stream : public Print
{
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
};
#endif
