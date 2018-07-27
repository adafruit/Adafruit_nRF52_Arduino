/**************************************************************************/
/*!
    @file     HarwareEncoder.h
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
#ifndef ROTARYENCODER_H_
#define ROTARYENCODER_H_

#include "Arduino.h"
#include "SwRotaryEncoder.h"

class HwRotaryEncoder
{
  public:
    typedef void (*callback_t) (int step);

    HwRotaryEncoder(void)
    {
      _abs = _last = 0;
      _cb  = NULL;
    }

    void begin(uint8_t pina, uint8_t pinb, int8_t pinled = -1);

    void setSampler(uint8_t period);
    void setDebounce(bool enable);
    void setReporter(int8_t sample_num);
    void setCallback(callback_t fp);

    void start(void);
    void stop(void);

    int32_t read(void);
    int32_t readAbs(void);
    void    writeAbs(int32_t value);
    void    clearAbs(void);


    int32_t readDebug(void);

    // Internal API
    void _irq_handler(void);

  private:
    // Note For each turn, encoder generate 2 transitions
    int32_t _abs;   // Absolute position
    int32_t _last;

    callback_t _cb;
};

extern class HwRotaryEncoder RotaryEncoder;


#endif /* ROTARYENCODER_H_ */
