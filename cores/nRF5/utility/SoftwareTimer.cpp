/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org) for Adafruit Industries
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

#include "SoftwareTimer.h"

SoftwareTimer::SoftwareTimer()
{
  _handle = NULL;
}

SoftwareTimer::~SoftwareTimer ()
{
  if ( _handle != NULL ) xTimerDelete(_handle, 0);
}

void SoftwareTimer::begin(uint32_t ms, TimerCallbackFunction_t callback, void* timerID, bool repeating)
{
  _handle = xTimerCreate(NULL, ms2tick(ms), repeating, timerID, callback);
}

void SoftwareTimer::setID(void* id)
{
  vTimerSetTimerID(_handle, id);
}

void* SoftwareTimer::getID(void)
{
  return pvTimerGetTimerID(_handle);
}

bool SoftwareTimer::start(void)
{
  bool ret = true;
  if ( isInISR() )
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ret = (pdPASS == xTimerStartFromISR(_handle, &xHigherPriorityTaskWoken));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }else
  {
    xTimerStart(_handle, 0);
  }
  return ret;
}

bool SoftwareTimer::stop (void)
{
  bool ret = true;
  if ( isInISR() )
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ret = (pdPASS == xTimerStopFromISR(_handle, &xHigherPriorityTaskWoken));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    xTimerStop (_handle, 0);
  }

  return ret;
}

bool SoftwareTimer::reset (void)
{
  bool ret = true;
  if ( isInISR() )
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ret = (pdPASS == xTimerResetFromISR(_handle, &xHigherPriorityTaskWoken));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    xTimerReset(_handle, 0);
  }

  return ret;
}

bool SoftwareTimer::setPeriod(uint32_t ms)
{
  BaseType_t active = xTimerIsTimerActive(_handle);
  bool ret = true;

  if ( isInISR() )
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    ret = (pdPASS == xTimerChangePeriodFromISR(_handle, ms2tick(ms), &xHigherPriorityTaskWoken));
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    xTimerChangePeriod(_handle, ms2tick(ms), 0);
  }

  // Change period of inactive timer will also start it !!
  if ( !active ) stop();

  return ret;
}
