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
 * Adafruit NeoPixel Service  0900
 *  - Pin   0901  | uint8_t | Read + Write |
 *  - Type  0902  | uint8_t | Read + Write | 0: neopixel, 1: dotstar
 *  - Data  0903  | { uint16_t start, uint8_t flags, data }
 *  - Count
 */

const uint8_t BLEAdafruitRgbPixel::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x00, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitRgbPixel::UUID128_CHR_PIN[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitRgbPixel::UUID128_CHR_TYPE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x02, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitRgbPixel::UUID128_CHR_DATA[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x03, 0x09, 0xAF, 0xAD
};

// Constructor
BLEAdafruitRgbPixel::BLEAdafruitRgbPixel(void)
  : BLEService(UUID128_SERVICE), Pin(UUID128_CHR_PIN), Type(UUID128_CHR_TYPE), Data(UUID128_CHR_DATA)
{

}

err_t BLEAdafruitRgbPixel::begin (Adafruit_NeoPixel* neo_pixel)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add Characteristic
  Pin.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Pin.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Pin.setFixedLen(1);
  VERIFY_STATUS( Pin.begin() );
  Pin.write8(0);

  // Add Characteristic
  Type.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  Type.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  Type.setFixedLen(1);
  VERIFY_STATUS( Type.begin() );
  Type.write16(0);

  // Add Characteristic
  Data.setProperties(CHR_PROPS_WRITE);
  Data.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  // Change to use VLOC STACK to USER due to lack of memroy
  // Data.setMaxLen(Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH));
  Data.setMaxLen(20);
  VERIFY_STATUS( Data.begin() );

  return ERROR_NONE;
}
