/**************************************************************************/
/*!
    @file     wiring_analog.cpp
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


/* Implement analogWrite() and analogWriteResolution() using
 * HardwarePWM.
 *
 * Note: Arduino's core implement this in C linker. So do we
 */

extern "C"
{

enum
{
  ANALOG_TOKEN = 0x676f6c41 // 'A' 'l' 'o' 'g'
};

static uint8_t _analogResolution = 8; // default is 256 levels

/**
 * This will apply to all PWM Hardware currently used by analogWrite(),
 * and automatically apply to future calls to analogWrite().
 */
void analogWriteResolution( uint8_t res )
{
  // save the resolution for when adding a new instance
  _analogResolution = res;
  for (int i = 0; i<HWPWM_MODULE_NUM; i++)
  {
    if (HwPWMx[i]->isOwner(ANALOG_TOKEN))
    {
      HwPWMx[i]->setResolution(res);
    }
  }
}

/**
 * Generate PWM without pre-configured. this function will
 * configure pin to available HardwarePWM and start it if not started
 *
 * @param pin
 * @param value
 */
void analogWrite( uint32_t pin, uint32_t value )
{
  // first, handle the case where the pin is already in use by analogWrite()
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if (HwPWMx[i]->isOwner(ANALOG_TOKEN))
    {
      int const ch = HwPWMx[i]->pin2channel(pin);
      if (ch >= 0)
      {
        HwPWMx[i]->writeChannel(ch, value);
        return;
      }
    }
  }

  // Next, handle the case where can add the pin to a PWM instance already owned by analogWrite()
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if ( HwPWMx[i]->isOwner(ANALOG_TOKEN) && HwPWMx[i]->addPin(pin) )
    {
      // successfully added the pin, so write the value also
      HwPWMx[i]->writePin(pin, value);
      LOG_LV2("Analog", "Added pin %" PRIu32 " to already-owned PWM %d", pin, i);
      return;
    }
  }

  // Attempt to acquire a new HwPWMx instance ... but only where
  // 1. it's not one already used for analog, and
  // 2. it currently has no pins in use.
  for(int i=0; i<HWPWM_MODULE_NUM; i++)
  {
    if (HwPWMx[i]->takeOwnership(ANALOG_TOKEN))
    {
      // apply the cached analog resolution to newly owned instances
      HwPWMx[i]->setResolution(_analogResolution);
      HwPWMx[i]->addPin(pin);
      HwPWMx[i]->writePin(pin, value);
      LOG_LV2("Analog", "took ownership of, and added pin %" PRIu32 " to, PWM %d", pin, i);
      return;
    }
  }

  LOG_LV1("Analog", "Unable to find a free PWM peripheral");
  return;
}

} // end extern "C"
