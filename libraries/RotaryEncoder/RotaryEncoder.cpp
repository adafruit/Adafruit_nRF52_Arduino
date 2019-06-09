/**************************************************************************/
/*!
    @file     HardwareEncoder.cpp
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

#include "RotaryEncoder.h"

class HwRotaryEncoder RotaryEncoder;

extern "C"
{
  void QDEC_IRQHandler(void);
}

/**
 * Initialize Hardware Encoder
 */
void HwRotaryEncoder::begin(uint8_t pina, uint8_t pinb, int8_t pinled)
{
  // default sample period
  NRF_QDEC->SAMPLEPER = QDEC_SAMPLEPER_SAMPLEPER_128us;

  pinMode(pina, INPUT_PULLUP);
  pinMode(pinb, INPUT_PULLUP);

  NRF_QDEC->PSEL.A = g_ADigitalPinMap[pina];
  NRF_QDEC->PSEL.B = g_ADigitalPinMap[pinb];

  if ( pinled >= 0 )
  {
    pinMode(pinled, INPUT);

    NRF_QDEC->PSEL.LED = g_ADigitalPinMap[pinled];
  }

  // Disable debounce by default
  NRF_QDEC->DBFEN = 0;

  // Reporter is disabled by default

}

void HwRotaryEncoder::start(void)
{
  // Enable IRQ
  if ( NRF_QDEC->INTENSET )
  {
    NVIC_ClearPendingIRQ(QDEC_IRQn);
    NVIC_EnableIRQ(QDEC_IRQn);
  }

  // Enable
  NRF_QDEC->ENABLE = 1;

  // Start
  NRF_QDEC->TASKS_START = 1;
}

void HwRotaryEncoder::stop(void)
{
  if ( NRF_QDEC->INTENSET )
  {
    NVIC_DisableIRQ(QDEC_IRQn);
  }

  // Stop
  NRF_QDEC->TASKS_STOP = 1;

  // Disable
  NRF_QDEC->ENABLE = 0;
}


/**
 * * @param sample_period
 * @param period value is QDEC_SAMPLEPER_SAMPLEPER_xxx in nrf52_bitfields.h
 */
void HwRotaryEncoder::setSampler(uint8_t period)
{
  NRF_QDEC->SAMPLEPER = period;
}

void HwRotaryEncoder::setDebounce(bool enable)
{
  NRF_QDEC->DBFEN = enable;
}

void HwRotaryEncoder::setReporter(int8_t sample_num)
{
  // Disable
  if (sample_num < 0)
  {
    NRF_QDEC->INTENCLR = QDEC_INTENCLR_REPORTRDY_Msk;
    NRF_QDEC->SHORTS  &= ~QDEC_SHORTS_REPORTRDY_READCLRACC_Msk;
  }else
  {
    NRF_QDEC->REPORTPER = sample_num;
    NRF_QDEC->INTENSET  = QDEC_INTENSET_REPORTRDY_Msk;

    // shortcut
    NRF_QDEC->SHORTS |= QDEC_SHORTS_REPORTRDY_READCLRACC_Msk;
  }
}

void HwRotaryEncoder::setCallback(callback_t fp)
{
  _cb = fp;

  if (fp)
  {
    NRF_QDEC->INTENSET = QDEC_INTENSET_SAMPLERDY_Msk;
  }else
  {
    NRF_QDEC->INTENCLR = QDEC_INTENCLR_SAMPLERDY_Msk;
  }
}

int32_t HwRotaryEncoder::read(void)
{
  // Trigger READ CLR ACC
  NRF_QDEC->TASKS_RDCLRACC = 1;

  // Nordic QDEC CW is negative, CCW is positive --> invert
  int32_t val = -(NRF_QDEC->ACCREAD);

  // Add to absolute value and buffered step
  _abs += val;

  int32_t diff = (_abs - _last) / 2;

  if ( diff ) _last = _abs;
  return diff;
}

int32_t HwRotaryEncoder::readDebug(void)
{
  return _abs;
}

int32_t HwRotaryEncoder::readAbs(void)
{
  // first update the abs value
  read();

  return _abs/2;
}

void HwRotaryEncoder::writeAbs(int32_t value)
{
  _abs = value;
}

void HwRotaryEncoder::clearAbs(void)
{
  _abs = 0;
}

void HwRotaryEncoder::_irq_handler(void)
{
  if ( (NRF_QDEC->INTENSET & QDEC_INTENSET_SAMPLERDY_Msk) && NRF_QDEC->EVENTS_SAMPLERDY )
  {
    NRF_QDEC->EVENTS_SAMPLERDY = 0;

    if ( _cb )
    {
      int32_t step = read();
      if ( step ) ada_callback(NULL, 0, _cb, step);
    }
  }
}

void QDEC_IRQHandler(void)
{
  RotaryEncoder._irq_handler();
}
