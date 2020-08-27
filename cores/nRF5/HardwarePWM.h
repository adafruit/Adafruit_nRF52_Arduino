/**************************************************************************/
/*!
    @file     HardwarePWM.h
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
#ifndef HARDWAREPWM_H_
#define HARDWAREPWM_H_

#include "common_inc.h"
#include "nrf.h"
#include <atomic>

#ifdef NRF52840_XXAA
#define HWPWM_MODULE_NUM    4
#else
#define HWPWM_MODULE_NUM    3
#endif

class HardwarePWM
{
  private:
    enum { MAX_CHANNELS = 4 }; // Max channel per group
    NRF_PWM_Type * const _pwm;
    std::atomic_uint32_t _owner_token;

    uint16_t _seq0[MAX_CHANNELS];

    uint16_t  _max_value;
    uint8_t  _clock_div;

  public:
    HardwarePWM(NRF_PWM_Type* pwm);

    // Configure
    void setResolution(uint8_t bitnum); // set max value by 2^bitnum - 1
    void setMaxValue(uint16_t value);   // set max value

    void setClockDiv(uint8_t div);      // value is PWM_PRESCALER_PRESCALER_DIV_x, DIV1 is 16Mhz

    // Cooperative ownership sharing

    // returns true ONLY when (1) no PWM channel has a pin, and (2) the owner token is nullptr
    bool takeOwnership   (uint32_t token);

    // returns true ONLY when (1) no PWM channel has a pin attached, and (2) the owner token matches
    bool releaseOwnership(uint32_t token);

    // allows caller to verify that they own the peripheral
    bool isOwner(uint32_t token) const
    {
      return this->_owner_token == token;
    }

    // Add a pin to PWM module
    bool addPin     (uint8_t pin);

    // Remove a pin from PWM module
    bool removePin  (uint8_t pin);

    // Get the mapped channel of a pin
    int  pin2channel(uint8_t pin) const
    {
      pin = g_ADigitalPinMap[pin];
      for(int i=0; i<MAX_CHANNELS; i++)
      {
        if ( _pwm->PSEL.OUT[i] == pin ) return i;
      }
      return (-1);
    }

    // Check if pin is controlled by PWM
    bool checkPin(uint8_t pin) const
    {
      return pin2channel(pin) >= 0;
    }

    void begin (void);
    bool enabled (void);
    void stop  (void);

    // Generate PWM
    bool writePin    (uint8_t pin, uint16_t value, bool inverted = false);
    bool writeChannel(uint8_t ch , uint16_t value, bool inverted = false);

    // Read current set value
    uint16_t readPin     (uint8_t pin) const;
    uint16_t readChannel (uint8_t ch) const;

    // Get count of used / free channels
    uint8_t usedChannelCount(void) const;
    uint8_t freeChannelCount(void) const;

    static void DebugOutput(Stream& logger);

  private:
    void _set_psel(int ch, uint32_t value);
};

extern HardwarePWM HwPWM0;
extern HardwarePWM HwPWM1;
extern HardwarePWM HwPWM2;

#ifdef NRF_PWM3
extern HardwarePWM HwPWM3;
#endif

extern HardwarePWM* HwPWMx[];

#endif /* HARDWAREPWM_H_ */
