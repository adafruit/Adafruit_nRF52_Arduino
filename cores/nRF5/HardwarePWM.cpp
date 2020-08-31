/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach for Adafruit Industries
 * Copyright (c) 2020 Henry Gabryjelski
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

#if CFG_DEBUG
bool can_stringify_token(uint32_t token)
{
  uint8_t * t = (uint8_t *)&token;
  for (size_t i = 0; i < sizeof(uint32_t); ++i, ++t)
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
    uint32_t token = pwm->_owner_token;
    logger.printf(" || %d:", i);
    if (can_stringify_token(token)) {
      uint8_t * t = (uint8_t*)(&token);
      logger.printf("   \"%c%c%c%c\"", t[0], t[1], t[2], t[3] );
    } else {
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

void HardwarePWM::stop(void)
{
  _pwm->ENABLE = 0;
}

bool HardwarePWM::enabled (void)
{
  return _pwm->ENABLE;
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

void HardwarePWM::_set_psel(int ch, uint32_t value)
{
  // Must disable before changing PSEL
  if ( enabled() )
  {
    _pwm->ENABLE = 0;
    _pwm->PSEL.OUT[ch] = value;
    _seq0[ch] = 0;
    _pwm->ENABLE = 1;

    // re-start sequence
    if ( usedChannelCount() ) _pwm->TASKS_SEQSTART[0] = 1;
  }else
  {
    _pwm->PSEL.OUT[ch] = value;
    _seq0[ch] = 0;
  }
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

  _set_psel(ch, g_ADigitalPinMap[pin]);

  return true;
}

bool HardwarePWM::removePin(uint8_t pin)
{
  int ch = pin2channel(pin);
  VERIFY( ch >= 0 );

  _set_psel(ch, 0xFFFFFFFFUL);
  return true;
}

bool HardwarePWM::writeChannel(uint8_t ch, uint16_t value, bool inverted)
{
  VERIFY( ch < MAX_CHANNELS );

  _seq0[ch] = value | (inverted ? 0 : bit(15));

  // Initialize PWM if not already
  if ( !enabled() ) begin();

  // start sequence
  _pwm->TASKS_SEQSTART[0] = 1;

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

// returns true ONLY when (1) no PWM channel has a pin, and (2) the owner token is nullptr
bool HardwarePWM::takeOwnership(uint32_t token)
{
  bool const thread_mode = !isInISR();

  if (token == 0) {
    if (thread_mode) LOG_LV1("HwPWM", "zero is not a valid ownership token (attempted use in takeOwnership)");
    return false;
  }

  if (token == this->_owner_token) {
    if (thread_mode) LOG_LV1("HwPWM", "failing to acquire ownership because already owned by requesting token (cannot take ownership twice)");
    return false;
  }

  if ( this->_owner_token != 0 ) return false;
  if ( this->usedChannelCount() != 0 ) return false;
  if ( this->enabled() ) return false;

  // Use C++11 atomic CAS operation
  uint32_t expectedValue = 0U;
  return this->_owner_token.compare_exchange_strong(expectedValue, token);
}

// returns true ONLY when (1) no PWM channel has a pin attached, and (2) the owner token matches
bool HardwarePWM::releaseOwnership(uint32_t token)
{
  bool const thread_mode = !isInISR();

  if (token == 0) {
    if (thread_mode) LOG_LV1("HwPWM", "zero is not a valid ownership token (attempted use in releaseOwnership)");
    return false;
  }

  if (!this->isOwner(token)) {
    if (thread_mode) LOG_LV1("HwPWM", "attempt to release ownership when not the current owner");
    return false;
  }

  if (this->usedChannelCount() != 0) {
    if (thread_mode) LOG_LV1("HwPWM", "attempt to release ownership when at least on channel is still connected");
    return false;
  }

  if (this->enabled()) {
    if (thread_mode) LOG_LV1("HwPWM", "attempt to release ownership when PWM peripheral is still enabled");
    return false; // if it's enabled, do not allow ownership to be released, even with no pins in use
  }

  // Use C++11 atomic CAS operation
  return this->_owner_token.compare_exchange_strong(token, 0U);
}
