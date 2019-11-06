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

#include "BLEAdafruitTemperature.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

/* Adafruit Temperature Service
 * - Service: 20E6-0001-C332-42A8-93BD-25E905756CB8
 * - Temperature Celsius  : 0x2A6E
 * - Measurement Interval : 0x2A21
 *
 * https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.temperature.xml
 * https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.measurement_interval.xml
 */

const uint8_t BLEAdafruitTemperature::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x00, 0xE6, 0x20
};

// Constructor
BLEAdafruitTemperature::BLEAdafruitTemperature(void)
  : BLEService(UUID128_SERVICE), Temperature(UUID16_CHR_TEMPERATURE), MeasurementInterval(UUID16_CHR_MEASUREMENT_INTERVAL)
{

}

err_t BLEAdafruitTemperature::begin (void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add Temperature Characteristic
  Temperature.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  Temperature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  Temperature.setFixedLen(2);
  Temperature.setUserDescriptor("Temperature");
  VERIFY_STATUS( Temperature.begin() );

  // Add Measurement Interval Characteristic
  MeasurementInterval.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  MeasurementInterval.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  MeasurementInterval.setFixedLen(2);
  MeasurementInterval.setUserDescriptor("Measurement Interval");
  VERIFY_STATUS( MeasurementInterval.begin() );

  MeasurementInterval.write16(30); // measure every 30 seconds

  return ERROR_NONE;
}
