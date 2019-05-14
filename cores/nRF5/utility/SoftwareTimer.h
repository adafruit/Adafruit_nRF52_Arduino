/**************************************************************************/
/*!
    @file     SoftwareTimer.h
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
#ifndef SOFTWARETIMER_H_
#define SOFTWARETIMER_H_

#include "Arduino.h"

class SoftwareTimer
{
  private:
    TimerHandle_t _handle;

  public:
    SoftwareTimer()          { _handle = NULL; }
    virtual ~SoftwareTimer() { if(_handle != NULL) xTimerDelete(_handle, 0); }

    void begin(uint32_t ms, TimerCallbackFunction_t callback, bool repeating = true)
    {
      _handle = xTimerCreate(NULL, ms2tick(ms), repeating, NULL, callback);
    }

    TimerHandle_t getHandle(void)
    {
      return _handle;
    }

    void start(void) { xTimerStart(_handle, 0); }
    void stop (void) { xTimerStop (_handle, 0); }
    void reset(void) { xTimerReset(_handle, 0); }

    bool startFromISR(void) {
      BaseType_t ret, xHigherPriorityTaskWoken = pdFALSE;
      ret = xTimerStartFromISR(_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      return (ret == pdPASS);
    }

    bool stopFromISR(void) {
      BaseType_t ret, xHigherPriorityTaskWoken = pdFALSE;
      ret = xTimerStopFromISR(_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      return (ret == pdPASS);
    }

    bool resetFromISR(void) {
      BaseType_t ret, xHigherPriorityTaskWoken = pdFALSE;
      ret = xTimerResetFromISR(_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      return (ret == pdPASS);
    }

    void setPeriod(uint32_t ms)
    {
      BaseType_t active = xTimerIsTimerActive(_handle);
      xTimerChangePeriod(_handle, ms2tick(ms), 0);

      // Change period of inactive timer will also start it !!
      if ( !active ) xTimerStop(_handle, 0);
    }
};

#endif /* SOFTWARETIMER_H_ */
