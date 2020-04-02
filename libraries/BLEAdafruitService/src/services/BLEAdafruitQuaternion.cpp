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
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+


/* All Adafruit Service/Characteristic UUID128 share the same Base UUID:
 *    ADAFxxx-C332-42A8-93BD-25E905756CB8
 *
 * Shared Characteristics
 *  - Measurement Period  0001 | int32_t | Read + Write |
 *    ms between measurements, -1: stop reading, 0: update when changes
 *
 * Quaternion service  0D00
 *  - Quater            #include <Adafruit>  0D01 | float[4] | Read + Notify | qw, qx, qy, qz
 *  - Measurement Period  0001
 */

const uint8_t BLEAdafruitQuaternion::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x00, 0x0D, 0xAF, 0xAD
};

const uint8_t BLEAdafruitQuaternion::UUID128_CHR_DATA[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x0D, 0xAF, 0xAD
};

// Constructor
BLEAdafruitQuaternion::BLEAdafruitQuaternion(void)
  : BLEAdafruitSensor(UUID128_SERVICE, UUID128_CHR_DATA)
{
  _accel = _gyro = _mag = NULL;
  _filter = NULL;
  _calib = NULL;

  // Setup Measurement Characteristic
  _measurement.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  _measurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _measurement.setFixedLen(4*4);
}

err_t BLEAdafruitQuaternion::begin(Adafruit_AHRS_FusionInterface* filter, Adafruit_Sensor* accel, Adafruit_Sensor* gyro, Adafruit_Sensor* mag)
{
  _filter = filter;
  _accel = accel;
  _gyro  = gyro;
  _mag   = mag;

  int32_t const period_ms = 100;

  // Invoke base class begin(), this will add Service, Measurement and Period characteristics
  VERIFY_STATUS( BLEAdafruitSensor::_begin(period_ms) );

  // filter timer run faster than measurement timer, but not too fast
  int32_t const filter_ms = maxof(10, period_ms / FUSION_MEASURE_RATIO);

  _filter_timer.begin(filter_ms, quaternion_filter_timer_cb, this, true);
  _filter->begin(1000.0f/filter_ms);

  return ERROR_NONE;
}

void BLEAdafruitQuaternion::setCalibration(Adafruit_Sensor_Calibration* calib)
{
  _calib = calib;
}

void BLEAdafruitQuaternion::_notify_handler(uint16_t conn_hdl, uint16_t value)
{
  // Start/Stop filter timer
  if (value & BLE_GATT_HVX_NOTIFICATION)
  {
    _filter_timer.start();
  }else
  {
    _filter_timer.stop();
  }

  // Call SuperClass function
  BLEAdafruitSensor::_notify_handler(conn_hdl, value);
}

void BLEAdafruitQuaternion::_update_timer(int32_t ms)
{
  int32_t const filter_ms = maxof(10, ms / FUSION_MEASURE_RATIO);

  if ( filter_ms < 0 )
  {
    _filter_timer.stop();
  }else if ( filter_ms > 0)
  {
    _filter_timer.setPeriod(filter_ms);
    _filter->begin(1000.0f/filter_ms); // need to also change the sample frequency
  }else
  {
    // Period = 0: keeping the current interval, but report on changes only
    // Not applicable for this service
  }

  // Call SuperClass function
  BLEAdafruitSensor::_update_timer(ms);
}

#if CFG_DEBUG
static void print_quaternion(float quater[4])
{
  // prepare for ability to change output, based on compile-time flags
  Print& logger = Serial;
  logger.printf("Quaternion: %.04f, %.04f, %.04f, %.04f\n", quater[0], quater[1], quater[2], quater[3]);
}
#endif

// Invoked by period timer in Base class
// Note invoked in RTOS Timer thread
void BLEAdafruitQuaternion::_measure_handler(void)
{
  float quater[4]; // w, x, y, z

  _filter->getQuaternion(&quater[0], &quater[1], &quater[2], &quater[3]);

  // TODO multiple connections
  _measurement.notify(quater, sizeof(quater));

#if CFG_DEBUG
  ada_callback(quater, sizeof(quater), print_quaternion, quater);
#endif
}

// Fusion Filter update
// This function take ~ 1-2 ms to get all sensor data
void BLEAdafruitQuaternion::_fitler_update(void)
{
  // get sensor events
  sensors_event_t accel_evt, gyro_evt, mag_evt;

//  int start_ms = millis();

  _accel->getEvent(&accel_evt);
  _gyro->getEvent(&gyro_evt);
  _mag->getEvent(&mag_evt);

  // calibrate sensor if available
  if (_calib)
  {
    _calib->calibrate(accel_evt);
    _calib->calibrate(gyro_evt);
    _calib->calibrate(mag_evt);
  }

  // Convert gyro from Rad/s to Degree/s
  gyro_evt.gyro.x *= SENSORS_RADS_TO_DPS;
  gyro_evt.gyro.y *= SENSORS_RADS_TO_DPS;
  gyro_evt.gyro.z *= SENSORS_RADS_TO_DPS;

  // apply filter
  _filter->update(gyro_evt.gyro.x , gyro_evt.gyro.y, gyro_evt.gyro.z,
                  accel_evt.acceleration.x, accel_evt.acceleration.y, accel_evt.acceleration.z,
                  mag_evt.magnetic.x, mag_evt.magnetic.y, mag_evt.magnetic.z);

//  start_ms = millis() - start_ms;
//  PRINT_INT(start_ms);
}

//--------------------------------------------------------------------+
// Static Methods
//--------------------------------------------------------------------+

void BLEAdafruitQuaternion::quaternion_filter_timer_cb(TimerHandle_t xTimer)
{
  BLEAdafruitQuaternion* svc = (BLEAdafruitQuaternion*) pvTimerGetTimerID(xTimer);
  svc->_fitler_update();
}

