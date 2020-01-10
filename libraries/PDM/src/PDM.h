/*
  Copyright (c) 2019 Arduino LLC.  All right reserved.

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

#ifndef _PDM_H_INCLUDED
#define _PDM_H_INCLUDED

#include <Arduino.h>

#if !defined(ARDUINO_NRF52_ADAFRUIT)
#error "This library targets only Adafruit NRF52840 boards"
#endif

#include "utility/PDMDoubleBuffer.h"

class PDMClass
{
public:
  PDMClass(int dataPin, int clkPin, int pwrPin);
  virtual ~PDMClass();

  void setPins(int dataPin, int clkPin, int pwrPin);
  int begin(int channels, long sampleRate);
  void end();

  virtual int available();
  virtual int read(void* buffer, size_t size);

  void onReceive(void(*)(void));

  void setGain(int gain);
  void setBufferSize(int bufferSize);

// private:
  void IrqHandler();

private:
  int _dinPin;
  int _clkPin;
  int _pwrPin;

  int _channels;
  
  PDMDoubleBuffer _doubleBuffer;
  
  void (*_onReceive)(void);
};

extern PDMClass PDM;

#endif
