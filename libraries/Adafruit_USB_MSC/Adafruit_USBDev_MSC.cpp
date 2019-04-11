/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 hathach for Adafruit Industries
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

#include "Adafruit_USBDev_MSC.h"

Adafruit_USBDev_MSC::Adafruit_USBDev_MSC(uint8_t epout, uint8_t epin, uint16_t epsize)
{
  _epout = epout;
  _epin = epin;
  _epsize = epsize;
}

uint16_t Adafruit_USBDev_MSC::getDescriptor(uint8_t* buf, uint16_t bufsize)
{
  uint8_t desc[] = { TUD_MSC_DESCRIPTOR(0, 0, _epout, _epin, _epsize) };
  uint16_t const len = sizeof(desc);

  if ( bufsize < len ) return 0;
  memcpy(buf, desc, len);
  return len;
}

void Adafruit_USBDev_MSC::begin(void)
{
  USBDevice.addInterface(*this);
}

