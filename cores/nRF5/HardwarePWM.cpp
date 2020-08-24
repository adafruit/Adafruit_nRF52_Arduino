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

extern uint8_t LastUsedNrfPinAsAnalog[NUMBER_OF_PINS]; // defined in wiring_digital.c

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

#if CFG_DEBUG
bool can_stringify_token(uintptr_t token)
{
  uint8_t * t = (uint8_t *)&token;
  for (size_t i = 0; i < sizeof(uintptr_t); ++i, ++t)
  {
    uint8_t x = *t;
    if ((x < 0x20) || (x > 0x7E)) return false;
  }
  return true;
}

void HardwarePWM::DebugOutput(Stream& logger)
{
  const size_t count = arrcount(HwPWMx);
  logger.printf("HwPWM Debug:");
  for (size_t i = 0; i < count; i++) {
    HardwarePWM const * pwm = HwPWMx[i];
    uintptr_t token = pwm->_owner_token;
    logger.printf(" || %d:", i);
    if (can_stringify_token(token)) {
      uint8_t * t = (uint8_t*)(&token);
      static_assert(sizeof(uintptr_t) == 4);
      logger.printf("   \"%c%c%c%c\"", t[0], t[1], t[2], t[3] );
    } else {
      static_assert(sizeof(uintptr_t) == 4);
      logger.printf(" %08x", token);
    }
    for (size_t j = 0; j < MAX_CHANNELS; j++) {
      uint32_t r = pwm->_pwm->PSEL.OUT[j]; // only read it once
      if ( (r & PWM_PSEL_OUT_CONNECT_Msk) != (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos) ) {
        logger.printf(" %02x", r & 0x1F);
      } else {
        logger.printf(" xx");
      }
    }
  }
  logger.printf("\n");
}
#else
void HardwarePWM::DebugOutput(Stream& logger) {}
#endif // CFG_DEBUG

// returns true ONLY when (1) no PWM channel has a pin, and (2) the owner token is nullptr
bool HardwarePWM::takeOwnership(uintptr_t token)
{
  bool notInIsr = !isInISR();
  if (token == 0) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "zero / nullptr is not a valid ownership token (attempted use in takeOwnership)");
    }
    return false; // cannot take ownership with nullptr
  }
  if (token == this->_owner_token) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "failing to acquire ownership because already owned by requesting token (cannot take ownership twice)");
    }
  }
  if (this->_owner_token != 0) {
    return false;
  }
  if (this->usedChannelCount() != 0) {
    return false;
  }
  if (this->enabled()) {
    return false;
  }
  // TODO: warn, but do not fail, if taking ownership with IRQs already enabled
  // NVIC_GetActive

  // Use C++11 atomic CAS operation
  uintptr_t newValue = 0U;
  return this->_owner_token.compare_exchange_strong(newValue, token);
}
// returns true ONLY when (1) no PWM channel has a pin attached, and (2) the owner token matches
bool HardwarePWM::releaseOwnership(uintptr_t token)
{
  bool notInIsr = !isInISR();
  if (token == 0) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "zero / nullptr is not a valid ownership token (attempted use in releaseOwnership)");
    }
    return false;
  }
  if (!this->isOwner(token)) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "attempt to release ownership when not the current owner");
    }
    return false;
  }
  if (this->usedChannelCount() != 0) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "attempt to release ownership when at least on channel is still connected");
    }
    return false;
  }
  if (this->enabled()) {
    if (notInIsr) {
      LOG_LV1("HwPWM", "attempt to release ownership when PWM peripheral is still enabled");
    }
    return false; // if it's enabled, do not allow ownership to be released, even with no pins in use
  }
  // TODO: warn, but do not fail, if releasing ownership with IRQs enabled
  // NVIC_GetActive

  // Use C++11 atomic CAS operation
  bool result = this->_owner_token.compare_exchange_strong(token, 0U);
  if (!result) {
    LOG_LV1("HwPWM", "race condition resulted in failure to acquire ownership");
  }
  return result;
}

HardwarePWM::HardwarePWM(NRF_PWM_Type* pwm) :
  _pwm(pwm)
{
  _owner_token = 0U;
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

  // for automatic removal if later call digitalWrite()...
  uint32_t nrfPin = g_ADigitalPinMap[pin];
  LastUsedNrfPinAsAnalog[nrfPin] = 1;

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

  // for automatic removal if later call digitalWrite()...
  uint32_t nrfPin = g_ADigitalPinMap[pin];
  LastUsedNrfPinAsAnalog[nrfPin] = 0;

  bool const en = enabled();

  // Must disable before changing PSEL
  if ( en ) _pwm->ENABLE = 0;

  _pwm->PSEL.OUT[ch] = 0xFFFFFFFFUL;
  _seq0[ch] = 0;

  if ( en ) _pwm->ENABLE = 1;

  return true;
}

extern "C" {
  bool removePinFromAllHwPWMInstances(uint8_t pin) // directly declared in wiring_digital also
  {
    bool result = false;
    for (int i = 0; i < HWPWM_MODULE_NUM; i++) {
      if (HwPWMx[i]->removePin(pin)) {
        result = true;
      }
    }
    return result;
  }
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

uint16_t HardwarePWM::readPin(uint8_t pin) const
{
  int ch = pin2channel(pin);
  VERIFY( ch >= 0, 0);

  return readChannel(ch);
}

uint16_t HardwarePWM::readChannel(uint8_t ch) const
{
  // remove inverted bit
  return (_seq0[ch] & 0x7FFF);
}

uint8_t HardwarePWM::usedChannelCount(void) const
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

uint8_t HardwarePWM::freeChannelCount(void) const
{
  return MAX_CHANNELS - usedChannelCount();
}

