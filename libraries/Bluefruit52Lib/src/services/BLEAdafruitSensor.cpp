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

#include "bluefruit.h"
#include "BLEAdafruitSensor.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

BLEAdafruitSensor::BLEAdafruitSensor(BLEUuid bleuuid)
  : BLEService(bleuuid), Period(UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD)
{
  _measure_cb = NULL;
}

void BLEAdafruitSensor::setMeasureCallback(measure_callback_t fp)
{
  _measure_cb = fp;
}

err_t BLEAdafruitSensor::begin(int32_t ms)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  Period.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Period.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Period.setFixedLen(4);
  VERIFY_STATUS( Period.begin() );
  Period.write32(ms);

  Period.setWriteCallback(sensor_period_write_cb, true);

  // setup timer
  _timer.begin(ms, sensor_timer_cb, this, true);

  return ERROR_NONE;
}

void BLEAdafruitSensor::startMeasuring(void)
{
  _timer.start();
}

void BLEAdafruitSensor::stopMeasuring(void)
{
  _timer.stop();
}

void BLEAdafruitSensor::sensor_timer_cb(TimerHandle_t xTimer)
{
  BLEAdafruitSensor* svc = (BLEAdafruitSensor*) pvTimerGetTimerID(xTimer);

  // invoke measure callback
  if (svc->_measure_cb) svc->_measure_cb();
}

// Client update period, adjust timer accordingly
void BLEAdafruitSensor::sensor_period_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAdafruitSensor& svc = (BLEAdafruitSensor&) chr->parentService();

  int32_t ms = 0;
  memcpy(&ms, data, len);

  // TODO handle period = 0 which notify on changes ASAP
  if ( ms < 0 )
  {
    svc._timer.stop();
  }else
  {
    svc._timer.setPeriod(ms);
  }
}

