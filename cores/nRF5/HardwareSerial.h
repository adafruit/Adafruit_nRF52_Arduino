/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include <nrf.h>

#include "Stream.h"

// below configs are not supported by nRF52
// #define SERIAL_5N1
// #define SERIAL_6N1
// #define SERIAL_7N1
// #define SERIAL_5N2
// #define SERIAL_6N2
// #define SERIAL_7N2
// #define SERIAL_5E1
// #define SERIAL_6E1
// #define SERIAL_7E1
// #define SERIAL_5E2
// #define SERIAL_6E2
// #define SERIAL_7E2
// #define SERIAL_5O1
// #define SERIAL_6O1
// #define SERIAL_7O1
// #define SERIAL_8O1
// #define SERIAL_5O2
// #define SERIAL_6O2
// #define SERIAL_7O2
// #define SERIAL_8O2

#ifdef NRF52832_XXAA
  #define SERIAL_8N1	(UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
  #define SERIAL_8E1	(UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos)
#elif defined(NRF52840_XXAA)
  #define SERIAL_8N1	((UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos) | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos))
  #define SERIAL_8N2	((UARTE_CONFIG_STOP_Two << UARTE_CONFIG_STOP_Pos) | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos))
  #define SERIAL_8E1	((UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos) | (UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos))
  #define SERIAL_8E2	((UARTE_CONFIG_STOP_Two << UARTE_CONFIG_STOP_Pos) | (UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos))
#else
  #error Unsupported MCU
#endif

class HardwareSerial : public Stream
{
  public:
    virtual void begin(unsigned long);
    virtual void begin(unsigned long baudrate, uint16_t config);
    virtual void end();
    virtual int available(void) = 0;
    virtual int peek(void) = 0;
    virtual int read(void) = 0;
    virtual void flush(void) = 0;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    using Print::write; // pull in write(str) from Print
    virtual operator bool() = 0;
};

extern void serialEventRun(void) __attribute__((weak));
extern void serialEvent() __attribute__((weak));

#ifndef NRF52832_XXAA // 832 only has 1 UART for Serial
extern void serialEvent1() __attribute__((weak));
extern void serialEvent2() __attribute__((weak));
#endif

#endif
