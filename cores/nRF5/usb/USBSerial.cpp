/**************************************************************************/
/*!
    @file     USBSerial.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifdef NRF52840_XXAA

#include "USBSerial.h"
#include "Arduino.h"
#include "tusb.h"


USBSerial Serial;

USBSerial::USBSerial(void)
{

}

// Baud and config is ignore in CDC
void USBSerial::begin(uint32_t baud) { }
void USBSerial::begin(uint32_t baud, uint8_t config) { }

void USBSerial::end(void)
{
  // nothing to do
}

USBSerial::operator bool()
{
  return tud_cdc_connected();
}

int USBSerial::available(void)
{
  return tud_cdc_available();
}

int USBSerial::peek(void)
{
  return tud_cdc_peek(0);
}

int USBSerial::read(void)
{
  return (int) tud_cdc_read_char();
}

size_t USBSerial::readBytes(char *buffer, size_t length)
{
  return tud_cdc_read(buffer, length);
}

void USBSerial::flush(void)
{
  tud_cdc_write_flush();
}

size_t USBSerial::write(uint8_t ch)
{
  return tud_cdc_write_char((char) ch);
}

size_t USBSerial::write(const uint8_t *buffer, size_t size)
{
  return tud_cdc_write(buffer, size);
}

#endif // NRF52840_XXAA
