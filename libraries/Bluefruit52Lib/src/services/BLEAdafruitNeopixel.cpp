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


//------------- IMPLEMENTATION -------------//
/* Adafruit NeoPixel Service
 * - Service: ADAF-0002-C332-42A8-93BD-25E905756CB8
 *    - Count : ADAF-0003-C332-42A8-93BD-25E905756CB8
 *    - Type  : ADAF-0004-C332-42A8-93BD-25E905756CB8
 *    - Data  : ADAF-0005-C332-42A8-93BD-25E905756CB8
 */

const uint8_t BLEAdafruitNeopixel::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x02, 0x00, 0xAF, 0xAD
};

const uint8_t BLEAdafruitNeopixel::UUID128_CHR_COUNT[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x03, 0x00, 0xAF, 0xAD
};

const uint8_t BLEAdafruitNeopixel::UUID128_CHR_TYPE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x04, 0x00, 0xAF, 0xAD
};

const uint8_t BLEAdafruitNeopixel::UUID128_CHR_DATA[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x05, 0x00, 0xAF, 0xAD
};

// Constructor
BLEAdafruitNeopixel::BLEAdafruitNeopixel(void)
  : BLEService(UUID128_SERVICE), Count(UUID128_CHR_COUNT), Type(UUID128_CHR_TYPE), Data(UUID128_CHR_DATA)
{

}

err_t BLEAdafruitNeopixel::begin (void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add Characteristic
  Count.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Count.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Count.setFixedLen(2);
  Count.setUserDescriptor("Count");
  VERIFY_STATUS( Count.begin() );
  Count.write16(0);

  // Add Characteristic
  Type.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Type.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Type.setFixedLen(2);
  Type.setUserDescriptor("Type");
  VERIFY_STATUS( Type.begin() );
  Type.write16(0);

  // Add Characteristic
  Data.setProperties(CHR_PROPS_WRITE);
  Data.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  Data.setMaxLen(Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH));
  Data.setUserDescriptor("Data");
  VERIFY_STATUS( Data.begin() );

  return ERROR_NONE;
}
