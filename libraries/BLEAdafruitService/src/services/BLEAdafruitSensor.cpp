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

#include "BLEAdafruitService.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

BLEAdafruitSensor::BLEAdafruitSensor(BLEUuid service_uuid, BLEUuid data_uuid)
  : BLEService(service_uuid), _measurement(data_uuid), _period(UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD)
{
  _sensor = NULL;
  _measure_cb = NULL;
  _notify_cb = NULL;
}

err_t BLEAdafruitSensor::_begin(int ms)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  _measurement.setCccdWriteCallback(sensor_data_cccd_cb, true);
  VERIFY_STATUS( _measurement.begin() );
  _measurement.write32(0); // zero 4 bytes could help to init some services

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

err_t BLEAdafruitSensor::begin(measure_callback_t fp, int ms)
{
  _measure_cb = fp;
  return _begin(ms);
}

err_t BLEAdafruitSensor::begin(Adafruit_Sensor* sensor, int ms)
{
  _sensor = sensor;
  return _begin(ms);
}

void BLEAdafruitSensor::setPeriod(int period_ms)
{
  _period.write32(period_ms);
  _update_timer(period_ms);
}

void BLEAdafruitSensor::setNotifyCallback(notify_callback_t fp)
{
  _notify_cb = fp;
}

//--------------------------------------------------------------------+
// Internal API
//--------------------------------------------------------------------+
void BLEAdafruitSensor::_notify_handler(uint16_t conn_hdl, uint16_t value)
{
  // notify enabled
  if (value & BLE_GATT_HVX_NOTIFICATION)
  {
    _timer.start();
  }else
  {
    _timer.stop();
  }

  // invoke callback if any
  if (_notify_cb) _notify_cb(conn_hdl, value);

  // send initial notification if period = 0
  //  if ( 0 == svc._period.read32() )
  //  {
  //    svc._measurement.notify();
  //  }
}

void BLEAdafruitSensor::_update_timer(int32_t ms)
{
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

void BLEAdafruitSensor::_measure_handler(void)
{
  uint16_t len = _measurement.getMaxLen();
  uint8_t buf[len];

  // Use unified sensor API if available, only with fixed length sensor
  if (_sensor && _measurement.isFixedLen())
  {
    sensors_event_t event;
    _sensor->getEvent(&event);

    memcpy(buf, event.data, len);
  }
  // Else use callback
  else if (_measure_cb)
  {
    len = _measure_cb(buf, sizeof(buf));
    len = min(len, sizeof(buf));
  }
  else
  {
    return; // nothing to measure
  }

  // no data to notify
  if (!len) return;

  // Period = 0, compare with old data, only update on changes
  if ( 0 == _period.read32() )
  {
    uint8_t prev_buf[_measurement.getMaxLen()];
    _measurement.read(prev_buf, sizeof(prev_buf));

    // skip notify if there is no changes
    if ( 0 == memcmp(prev_buf, buf, len) ) return;
  }

  // TODO multiple connections
  _measurement.notify(buf, len);
}

//--------------------------------------------------------------------+
// Static Callbacks
//--------------------------------------------------------------------+

void BLEAdafruitSensor::sensor_timer_cb(TimerHandle_t xTimer)
{
  BLEAdafruitSensor* svc = (BLEAdafruitSensor*) pvTimerGetTimerID(xTimer);
  svc->_measure_handler();
}

// Client update period, adjust timer accordingly
void BLEAdafruitSensor::sensor_period_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;
  BLEAdafruitSensor* svc = (BLEAdafruitSensor*) &chr->parentService();

  int32_t ms = 0;
  memcpy(&ms, data, len);

  svc->_update_timer(ms);
}

void BLEAdafruitSensor::sensor_data_cccd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value)
{
  BLEAdafruitSensor* svc = (BLEAdafruitSensor*) &chr->parentService();

  svc->_notify_handler(conn_hdl, value);
}

