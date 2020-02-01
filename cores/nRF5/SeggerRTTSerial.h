/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 by SimpleHacks, Henry Gabryjelski
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef SeggerRTTSerial_h
#define SeggerRTTSerial_h

//#include "Arduino.h"
#include <stdint.h>
#include <Stream.h>
#include <SEGGER_RTT.h>
#include <SEGGER_Extensions.h>

class Segger_RTT_Serial_t : public Stream {
public:
    virtual size_t write(uint8_t b) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    virtual int availableForWrite() override;
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;
    // Additional non-virtual functions to emulate `Serial`
    operator bool();
    void begin(uint32_t baud);
    void begin(uint32_t baud, uint8_t config);
    void end(void);
};

// Segger RTT allows for multiple virtual terminals
// via channel 0 ... but note that this will add
// 4 bytes RTT buffer overhead to each call to write().
template <unsigned char TerminalId>
class Segger_RTT_Terminal_t : public Segger_RTT_Serial_t {
private:
    static_assert(TerminalId < SEGGER_RTT_NUMBER_OF_TERMINALS, "TerminalID must be less than SEGGER_RTT_NUMBER_OF_TERMINALS" );
public:
    virtual size_t write(uint8_t b) override {
        return SEGGER_RTT_TerminalOutBuffer(TerminalId, &b, 1);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) override {
        return SEGGER_RTT_TerminalOutBuffer(TerminalId, buffer, size);
    }
    virtual int availableForWrite() override {
        int result = SEGGER_RTT_GetAvailWriteSpace(0);
        if (result > SEGGER_RTT_TERMINAL_OUT_OVERHEAD) {
            result -= SEGGER_RTT_TERMINAL_OUT_OVERHEAD;
        } else if (result > 0) {
            result = 0;
        }
        return result;
    }
};

#endif // #ifndef SeggerRTTSerial_h
