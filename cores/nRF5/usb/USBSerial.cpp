/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach for Adafruit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifdef NRF52840_XXAA

#include "USBSerial.h"
#include "Arduino.h"
#include "tusb.h"

#define EPOUT   0x00
#define EPIN    0x80

USBSerial Serial;

USBSerial::USBSerial(void)
{

}

uint16_t USBSerial::getDescriptor(uint8_t* buf, uint16_t bufsize)
{
  // CDC is mostly always existed for DFU
  uint8_t desc[] = { TUD_CDC_DESCRIPTOR(0, 0, EPIN, 8, EPOUT, EPIN, 64) };
  uint16_t const len = sizeof(desc);

  if ( bufsize < len ) return 0;

  memcpy(buf, desc, len);
  return len;
}

// Baud and config is ignore in CDC
void USBSerial::begin (uint32_t baud)
{
}
void USBSerial::begin (uint32_t baud, uint8_t config)
{
}

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
  size_t remain = size;
  while ( remain && tud_cdc_connected() )
  {
    size_t wrcount = tud_cdc_write(buffer, remain);
    remain -= wrcount;
    buffer += wrcount;

    // Write FIFO is full, flush and re-try
    if ( remain )
    {
      tud_cdc_write_flush();
    }
  }

  return size - remain;
}

#endif // NRF52840_XXAA
