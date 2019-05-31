/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
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

#ifdef USE_TINYUSB

#include "Arduino.h"
#include "Adafruit_USBD_CDC.h"

#define EPOUT   0x00
#define EPIN    0x80

Adafruit_USBD_CDC Serial;

Adafruit_USBD_CDC::Adafruit_USBD_CDC(void)
{

}

uint16_t Adafruit_USBD_CDC::getDescriptor(uint8_t* buf, uint16_t bufsize)
{
  // CDC is mostly always existed for DFU
  uint8_t desc[] = { TUD_CDC_DESCRIPTOR(0, 0, EPIN, 8, EPOUT, EPIN, 64) };
  uint16_t const len = sizeof(desc);

  if ( bufsize < len ) return 0;

  memcpy(buf, desc, len);
  return len;
}

// Baud and config is ignore in CDC
void Adafruit_USBD_CDC::begin (uint32_t baud)
{
}
void Adafruit_USBD_CDC::begin (uint32_t baud, uint8_t config)
{
}

void Adafruit_USBD_CDC::end(void)
{
  // nothing to do
}

Adafruit_USBD_CDC::operator bool()
{
  return tud_cdc_connected();
}

int Adafruit_USBD_CDC::available(void)
{
  return tud_cdc_available();
}

int Adafruit_USBD_CDC::peek(void)
{
  return tud_cdc_peek(0);
}

int Adafruit_USBD_CDC::read(void)
{
  return (int) tud_cdc_read_char();
}

void Adafruit_USBD_CDC::flush(void)
{
  tud_cdc_write_flush();
}

size_t Adafruit_USBD_CDC::write(uint8_t ch)
{
  return tud_cdc_write_char((char) ch);
}

size_t Adafruit_USBD_CDC::write(const uint8_t *buffer, size_t size)
{
  size_t remain = size;
  while ( remain && tud_cdc_connected() )
  {
    size_t wrcount = tud_cdc_write(buffer, remain);
    remain -= wrcount;
    buffer += wrcount;

    // Write FIFO is full, run usb background to flush
    if ( remain ) yield();
  }

  return size - remain;
}

extern "C"
{

// Invoked when cdc when line state changed e.g connected/disconnected
// Use to reset to DFU when disconnect with 1200 bps
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;  // interface ID, not used

  // DTR = false is counted as disconnected
  if ( !dtr )
  {
    cdc_line_coding_t coding;
    tud_cdc_get_line_coding(&coding);

    if ( coding.bit_rate == 1200 ) Adafruit_TinyUSB_Core_touch1200();
  }
}

}

#endif // USE_TINYUSB
