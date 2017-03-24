/**************************************************************************/
/*!
    @file     HardwarePWM.h
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
#ifndef HARDWAREPWM_H_
#define HARDWAREPWM_H_

#include "common_inc.h"
#include "nrf.h"

class HardwarePWM
{
  private:
    enum { MAX_CHANNELS = 4 }; // Max channel per group
    NRF_PWM_Type* _pwm;

    uint8_t  _count;
    uint16_t _seq0[MAX_CHANNELS];

    void _start(void);

  public:
    HardwarePWM(NRF_PWM_Type* pwm);

    // Enable & Disable module
    void disable(void);

    // Configure
    void setResolution(uint8_t bitnum);
    void setClockDiv(uint8_t div); // value is PWM_PRESCALER_PRESCALER_DIV_x, DIV1 is 16Mhz

    bool addPin     (uint8_t pin);
    int  pin2channel(uint8_t pin) ATTR_ALWAYS_INLINE
    {
      for(int i=0; i<_count; i++)
      {
        if ( _pwm->PSEL.OUT[i] == pin ) return i;
      }
      return (-1);
    }

    bool checkPin(uint8_t pin) ATTR_ALWAYS_INLINE
    {
      return pin2channel(pin) >= 0;
    }

    // Generate PWM
    void begin (void);
    bool begun (void);
    void stop  (void);

    bool writePin    (uint8_t pin, uint16_t value, bool inverted = false);
    bool writeChannel(uint8_t ch , uint16_t value, bool inverted = false);

    void printInfo(void);
};

extern HardwarePWM PWM0;
extern HardwarePWM PWM1;
extern HardwarePWM PWM2;

extern HardwarePWM* PWMx[3];

#endif /* HARDWAREPWM_H_ */
