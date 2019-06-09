/**************************************************************************/
/*!
    @file     SwRotaryEncoder.cpp
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

#include "SwRotaryEncoder.h"

void _pina_irq(uint8_t id);

#define _pina_irqn(n) \
  void _pina_irqn##_##n(void) { \
    _pina_irq(n);\
  }

extern "C"
{
  _pina_irqn(0)
  _pina_irqn(1)
  _pina_irqn(2)
  _pina_irqn(3)
//  _pina_irqn(4)
//  _pina_irqn(5)
//  _pina_irqn(6)
//  _pina_irqn(7)
  // add more to match SW_ROTARY_ENCODER_MAX_INSTANCE
}

static uint8_t _encoder_count = 0;
static SwRotaryEncoder* _encoder_ptr[SW_ROTARY_ENCODER_MAX_INSTANCE] = { NULL };

static voidFuncPtr _pina_irq_ptr[SW_ROTARY_ENCODER_MAX_INSTANCE] =
{
    _pina_irqn_0, _pina_irqn_1, _pina_irqn_2, _pina_irqn_3,
//    _pina_irqn_4, _pina_irqn_5, _pina_irqn_6, _pina_irqn_7
};

bool SwRotaryEncoder::begin(uint8_t pina, uint8_t pinb)
{
  // Add to pointer array
  VERIFY ( _encoder_count < SW_ROTARY_ENCODER_MAX_INSTANCE);
  _encoder_ptr[_encoder_count] = this;

  _pina = pina;
  _pinb = pinb;

  pinMode(_pina, INPUT_PULLUP);
  pinMode(_pinb, INPUT_PULLUP);

  _a_last = digitalRead(_pina);

  attachInterrupt(_pina, _pina_irq_ptr[_encoder_count], CHANGE);

  _encoder_count++;

  return true;
}

void SwRotaryEncoder::setCallback(callback_t fp)
{
  _cb = fp;
}

void SwRotaryEncoder::stop(void)
{
  detachInterrupt(_pina);
}

int32_t SwRotaryEncoder::read(void)
{
  int32_t ret = _abs - _last_read;
  _last_read = _abs;
  return ret;
}

int32_t SwRotaryEncoder::readAbs(void)
{
  return _abs;
}

void SwRotaryEncoder::writeAbs(int32_t value)
{
  _abs = value;
}

void SwRotaryEncoder::clearAbs(void)
{
  _abs = 0;
}

void SwRotaryEncoder::_irq_handler(void)
{
  uint32_t val = NRF_GPIO->IN;
  uint8_t bita = bitRead(val, _pina);

  if ( bita != _a_last)
  {
    int32_t step;

    if ( bita != bitRead(val, _pinb) )
    {
      step = 1;
    }else
    {
      step = -1;
    }

    _abs += step;
    _a_last = bita;

    if (_cb) ada_callback(NULL, 0, _cb, step);
  }
}

void _pina_irq(uint8_t id)
{
  _encoder_ptr[id]->_irq_handler();
}
