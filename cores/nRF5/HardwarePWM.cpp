/**************************************************************************/
/*!
    @file     HardwarePWM.cpp
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

#include "Arduino.h"
#include "HardwarePWM.h"

HardwarePWM HwPWM0(NRF_PWM0);
HardwarePWM HwPWM1(NRF_PWM1);
HardwarePWM HwPWM2(NRF_PWM2);

#ifdef NRF_PWM3
HardwarePWM HwPWM3(NRF_PWM3);
#endif

HardwarePWM* HwPWMx[] =
{
  &HwPWM0, &HwPWM1, &HwPWM2
#ifdef NRF_PWM3
  ,&HwPWM3
#endif
};

// returns true ONLY when (1) no PWM channel has a pin, and (2) the owner token is nullptr
bool HardwarePWM::takeOwnership(void const * token)
{
  if (this->_owner_token       != 0) return false; // doesn't matter if it's actually a match ... it's not legal to take ownership twice
  if (this->usedChannelCount() != 0) return false; // at least one channel is already in use
  if (this->enabled()              ) return false; // if it's enabled, do not allow new ownership, even with no pins in use
  // use gcc built-in intrinsic to ensure atomicity
  // See https://gcc.gnu.org/onlinedocs/gcc/_005f_005fsync-Builtins.html
  return __sync_bool_compare_and_swap(&(this->_owner_token), 0, token);
}
// returns true ONLY when (1) no PWM channel has a pin attached, and (2) the owner token matches
bool HardwarePWM::releaseOwnership(void const * token)
{
  if (!this->isOwner(token)         ) return false; // don't even look at peripheral
  if ( this->usedChannelCount() != 0) return false; // fail if any channels still have pins
  if ( this->enabled()              ) return false; // if it's enabled, do not allow ownership to be released, even with no pins in use
  // use gcc built-in intrinsic to ensure atomicity
  // See https://gcc.gnu.org/onlinedocs/gcc/_005f_005fsync-Builtins.html
  return __sync_bool_compare_and_swap(&(this->_owner_token), token, 0);
}

HardwarePWM::HardwarePWM(NRF_PWM_Type* pwm)
{
  _pwm = pwm;
  arrclr(_seq0);

  _max_value = 255;
  _clock_div = PWM_PRESCALER_PRESCALER_DIV_1; // 16 Mhz

  // FIXME workaround to fix bootloader 0.2.6 does not clean up PSEL[1] of PWM0
  _pwm->PSEL.OUT[1] = 0xFFFFFFFFUL;
}

void HardwarePWM::setResolution(uint8_t bitnum)
{
  setMaxValue( bit(min8(bitnum, 15)) -1 );
}

void HardwarePWM::setMaxValue(uint16_t value)
{
  _max_value = value;
  _pwm->COUNTERTOP = value;
}

void HardwarePWM::setClockDiv(uint8_t div)
{
  _clock_div = min8(div, PWM_PRESCALER_PRESCALER_DIV_128);
  _pwm->PRESCALER = _clock_div;
}

/**
 * Add pin to this group.
 * @param pin Pin to add
 * @return true if add succeeded, or pin is already added
 */
bool HardwarePWM::addPin(uint8_t pin)
{
  // succeed if pin is already configured
  if ( pin2channel(pin) >= 0 ) return true;

  int ch = -1;

  // find free slot which is not connected
  for(int i=0; i<MAX_CHANNELS; i++)
  {
    if ( _pwm->PSEL.OUT[i] & PWM_PSEL_OUT_CONNECT_Msk )
    {
      ch = i;
      break;
    }
  }
  VERIFY ( ch >= 0 );

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  // Must disable before changing PSEL
  if ( enabled() )
  {
    _pwm->ENABLE = 0;
    _pwm->PSEL.OUT[ch] = g_ADigitalPinMap[pin];
    _pwm->ENABLE = 1;
    _start();
  }else
  {
    _pwm->PSEL.OUT[ch] = g_ADigitalPinMap[pin];
  }

  return true;
}

bool HardwarePWM::removePin(uint8_t pin)
{
  int ch = pin2channel(pin);
  VERIFY( ch >= 0 );

  bool const en = enabled();

  // Must disable before changing PSEL
  if ( en ) _pwm->ENABLE = 0;

  _pwm->PSEL.OUT[ch] = 0xFFFFFFFFUL;
  _seq0[ch] = 0;

  if ( en ) _pwm->ENABLE = 1;

  return true;
}

bool HardwarePWM::enabled (void)
{
  return _pwm->ENABLE;
}

void HardwarePWM::begin(void)
{
  // Initialize Registers
  _pwm->MODE            = PWM_MODE_UPDOWN_Up;
  _pwm->COUNTERTOP      = _max_value; // default is 255 (8 bit), can be configured before begin()
  _pwm->PRESCALER       = _clock_div;
  _pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  _pwm->LOOP            = 0;

  _pwm->SEQ[0].PTR      = (uint32_t) _seq0;
  _pwm->SEQ[0].CNT      = MAX_CHANNELS; // default mode is Individual --> count must be 4
  _pwm->SEQ[0].REFRESH  = 0;
  _pwm->SEQ[0].ENDDELAY = 0;

  _pwm->SEQ[1].PTR      = 0;
  _pwm->SEQ[1].CNT      = 0;
  _pwm->SEQ[1].REFRESH  = 0;
  _pwm->SEQ[1].ENDDELAY = 0;

  _pwm->ENABLE = 1;
}

void HardwarePWM::_start(void)
{
  // update sequence count (depending on mode)
  //  _pwm->SEQ[0].CNT = MAX_CHANNELS;

  // start sequence
  _pwm->TASKS_SEQSTART[0] = 1;
}

void HardwarePWM::stop(void)
{
  _pwm->ENABLE = 0;
}

bool HardwarePWM::writeChannel(uint8_t ch, uint16_t value, bool inverted )
{
  VERIFY( ch < MAX_CHANNELS );

  _seq0[ch] = value | (inverted ? 0 : bit(15));

  // Start PWM if not already
  if ( !enabled() ) begin();

  _start();

  return true;
}

bool HardwarePWM::writePin(uint8_t pin, uint16_t value, bool inverted)
{
  int ch = pin2channel(pin);
  VERIFY( ch >= 0 );

  return writeChannel(ch, value, inverted);
}

uint16_t HardwarePWM::readPin(uint8_t pin)
{
  int ch = pin2channel(pin);
  VERIFY( ch >= 0, 0);

  return readChannel(ch);
}

uint16_t HardwarePWM::readChannel(uint8_t ch)
{
  // remove inverted bit
  return (_seq0[ch] & 0x7FFF);
}

uint8_t HardwarePWM::usedChannelCount(void)
{
  uint8_t usedChannels = 0;
  for(int i=0; i<MAX_CHANNELS; i++)
  {
    if ( (_pwm->PSEL.OUT[i] & PWM_PSEL_OUT_CONNECT_Msk) != (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos) )
    {
      usedChannels++;
    }
  }
  return usedChannels;
}

uint8_t HardwarePWM::freeChannelCount(void)
{
  return MAX_CHANNELS - usedChannelCount();
}

