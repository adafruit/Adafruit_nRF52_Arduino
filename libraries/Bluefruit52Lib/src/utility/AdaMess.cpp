/**************************************************************************/
/*!
    @file     AdaMess.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include "AdaMess.h"

AdaMess::AdaMess(void)
{
  _dynamic = true;
  _sem     = NULL;
  _buffer  = NULL;
  _bufsize = _rxlen = 0;
}

// dynamic mean semaphore is malloced and freed only when in action
void AdaMess::begin(bool dynamic)
{
  _dynamic = dynamic;
  if ( !_dynamic ) _sem = xSemaphoreCreateBinary();
}

uint16_t AdaMess::waitForData(void* buffer, uint16_t bufsize, uint32_t ms)
{
  _buffer  = (uint8_t*) buffer;
  _bufsize = bufsize;
  _rxlen   = 0;

  if (_dynamic)
  {
    _sem = xSemaphoreCreateBinary();
    VERIFY(_sem, 0);
  }

  uint16_t result = xSemaphoreTake(_sem, ms2tick(ms) ) ? _rxlen : 0;

  if (_dynamic)
  {
    vSemaphoreDelete(_sem);
    _sem = NULL;
  }

  return result;
}

uint16_t AdaMess::feed(void* data, uint16_t len)
{
  len = min16(len, _bufsize);

  memcpy(_buffer, data, len);

  _buffer  += len;
  _bufsize -= len;
  _rxlen   += len;

  return len;
}

void AdaMess::complete(void)
{
   xSemaphoreGive(_sem);
}

