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
