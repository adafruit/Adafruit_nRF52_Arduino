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

BLEAdafruitSensor::BLEAdafruitSensor(BLEUuid service_uuid, BLEUuid data_uuid)
  : BLEService(service_uuid), _measurement(data_uuid), _period(UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD)
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

  _measurement.setCccdWriteCallback(sensor_data_cccd_cb, true);

  VERIFY_STATUS( _measurement.begin() );

  _period.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  _period.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  _period.setFixedLen(4);
  VERIFY_STATUS( _period.begin() );
  _period.write32(ms);

  _period.setWriteCallback(sensor_period_write_cb, true);

  // setup timer
  _timer.begin(ms, sensor_timer_cb, this, true);

  return ERROR_NONE;
}

void BLEAdafruitSensor::setPeriod(int32_t period_ms)
{
  _period.write32(period_ms);
  _update_timer(period_ms);
}


void BLEAdafruitSensor::startMeasuring(void)
{
  _timer.start();
}

void BLEAdafruitSensor::stopMeasuring(void)
{
  _timer.stop();
}


void BLEAdafruitSensor::_update_timer(int32_t ms)
{
  // TODO handle period = 0 which notify on changes ASAP
  if ( ms < 0 )
  {
    _timer.stop();
  }else if ( ms > 0)
  {
    _timer.setPeriod(ms);
  }else
  {
    // Period = 0: keeping the current interval, but report on changes only
  }
}

void BLEAdafruitSensor::_timer_callback(void)
{
  if (_measure_cb)
  {
    uint8_t buf[_measurement.getMaxLen()];
    uint16_t len = _measure_cb(buf, sizeof(buf));
    len = min(len, sizeof(buf));

    // Period = 0, compare with old data, only update on changes
    if ( 0 == _period.read32() )
    {
      uint8_t prev_buf[_measurement.getMaxLen()];
      _measurement.read(prev_buf, sizeof(prev_buf));

      // skip notify if there is no changes
      if ( 0 == memcmp(prev_buf, buf, len) ) return;
    }

    _measurement.notify(buf, len);
  }
}

//--------------------------------------------------------------------+
// Static Callbacks
//--------------------------------------------------------------------+

void BLEAdafruitSensor::sensor_timer_cb(TimerHandle_t xTimer)
{
  BLEAdafruitSensor* svc = (BLEAdafruitSensor*) pvTimerGetTimerID(xTimer);
  svc->_timer_callback();
}

// Client update period, adjust timer accordingly
void BLEAdafruitSensor::sensor_period_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAdafruitSensor& svc = (BLEAdafruitSensor&) chr->parentService();

  int32_t ms = 0;
  memcpy(&ms, data, len);

  svc._update_timer(ms);
}

void BLEAdafruitSensor::sensor_data_cccd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value)
{
  BLEAdafruitSensor& svc = (BLEAdafruitSensor&) chr->parentService();

  // notify enabled
  if (value & BLE_GATT_HVX_NOTIFICATION)
  {
    svc._timer.start();
  }else
  {
    svc._timer.stop();
  }
}

