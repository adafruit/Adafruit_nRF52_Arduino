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
 * Temperature service    0100
 *  - Temperature         0101 | float   | Read + Notify | degree in Celsius
 *  - Measurement Period  0001
 */

const uint8_t BLEAdafruitTemperature::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x00, 0x01, 0xAF, 0xAD
};

const uint8_t BLEAdafruitTemperature::UUID128_CHR_TEMPERATURE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x01, 0xAF, 0xAD
};

// Constructor
BLEAdafruitTemperature::BLEAdafruitTemperature(void)
  : BLEService(UUID128_SERVICE), Temperature(UUID128_CHR_TEMPERATURE), Period(UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD)
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
  VERIFY_STATUS( Temperature.begin() );

  // Add Measurement Interval Characteristic
  Period.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Period.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Period.setFixedLen(4);
  VERIFY_STATUS( Period.begin() );

  Period.write32(30); // measure every 30 seconds

  return ERROR_NONE;
}
