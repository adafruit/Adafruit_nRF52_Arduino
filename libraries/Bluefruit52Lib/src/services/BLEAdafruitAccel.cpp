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


/* Adafruit Accelerometer Service
 * using micro:bit Accelerometer Service definition
 * https://lancaster-university.github.io/microbit-docs/resources/bluetooth/bluetooth_profile.html
 *
 * - Service: E95D-0753-251D-470A-A062-FA1922DFA9A8
 *    - Data   : E95D-CA4B-251D-470A-A062-FA1922DFA9A8
 *    - Period : E95D-FB24-251D-470A-A062-FA1922DFA9A8
 */
const uint8_t BLEAdafruitAccel::UUID128_SERVICE[16] =
{
  0xA8, 0xA9, 0xDF, 0x22, 0x19, 0xFA, 0x62, 0xA0,
  0x0A, 0x47, 0x1D, 0x25, 0x53, 0x07, 0x5D, 0xE9
};

const uint8_t BLEAdafruitAccel::UUID128_CHR_DATA[16] =
{
  0xA8, 0xA9, 0xDF, 0x22, 0x19, 0xFA, 0x62, 0xA0,
  0x0A, 0x47, 0x1D, 0x25, 0x4B, 0xCA, 0x5D, 0xE9
};

const uint8_t BLEAdafruitAccel::UUID128_CHR_PERIOD[16] =
{
  0xA8, 0xA9, 0xDF, 0x22, 0x19, 0xFA, 0x62, 0xA0,
  0x0A, 0x47, 0x1D, 0x25, 0x24, 0xFB, 0x5D, 0xE9
};

// Constructor
BLEAdafruitAccel::BLEAdafruitAccel(void)
  : BLEService(UUID128_SERVICE), Data(UUID128_CHR_DATA), Period(UUID128_CHR_PERIOD)
{

}

err_t BLEAdafruitAccel::begin (void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add Characteristic
  Data.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  Data.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  Data.setFixedLen(6);
  Data.setUserDescriptor("Data");
  VERIFY_STATUS( Data.begin() );

  // Add Characteristic
  Period.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Period.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Period.setFixedLen(2);
  Period.setUserDescriptor("Period");
  VERIFY_STATUS( Period.begin() );
  Period.write16(10);

  return ERROR_NONE;
}
