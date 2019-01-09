/**************************************************************************/
/*!
    @file     AdaMsg.cpp
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

#include "AdaMsg.h"

void AdaMsg::_init(void)
{
  _dynamic = true;
  _waiting = false;
  _sem     = NULL;

  buffer  = NULL;
  remaining = xferlen = 0;
}

AdaMsg::AdaMsg(void)
{
  _init();
}

// dynamic mean semaphore is malloced and freed only when in action
void AdaMsg::begin(bool dynamic)
{
  _dynamic = dynamic;
  if ( !_dynamic )
  {
    _sem = xSemaphoreCreateCounting(10, 0);
  }
}

void AdaMsg::stop(void)
{
  if (!_dynamic) vSemaphoreDelete(_sem);
  _init();
}

void AdaMsg::prepare(void* buf, uint16_t bufsize)
{
  buffer    = (uint8_t*) buf;
  remaining = bufsize;
  xferlen   = 0;
}

/**
 *
 * @param ms
 * @return -1 if timeout
 */
int32_t AdaMsg::waitUntilComplete(uint32_t ms)
{
  if (_dynamic)
  {
    _sem = xSemaphoreCreateBinary();
    VERIFY(_sem, -1);
  }

  int result = -1;

  _waiting = true;
  if ( xSemaphoreTake(_sem, ms2tick(ms) ) )
  {
    result = xferlen;
  }
  _waiting = false;

  if (_dynamic)
  {
    vSemaphoreDelete(_sem);
    _sem = NULL;
  }

  return result;
}

bool AdaMsg::isWaiting(void)
{
  return _waiting;
}

uint16_t AdaMsg::feed(void* data, uint16_t len)
{
  len = min16(len, remaining);

  // pass NULL to skip copy
  if ( data ) memcpy(buffer, data, len);

  buffer    += len;
  remaining -= len;
  xferlen   += len;

  return len;
}

void AdaMsg::complete(void)
{
  if(_sem) xSemaphoreGive(_sem);
}

