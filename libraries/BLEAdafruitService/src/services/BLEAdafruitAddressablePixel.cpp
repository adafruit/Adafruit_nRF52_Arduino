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

#include "bluefruit_common.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

#include "BLEAdafruitAddressablePixel.h"

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

const uint8_t BLEAdafruitAddressablePixel::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x00, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitAddressablePixel::UUID128_CHR_PIN[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitAddressablePixel::UUID128_CHR_TYPE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x02, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitAddressablePixel::UUID128_CHR_DATA[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x03, 0x09, 0xAF, 0xAD
};

const uint8_t BLEAdafruitAddressablePixel::UUID128_CHR_BUFSIZE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x04, 0x09, 0xAF, 0xAD
};

// Constructor
BLEAdafruitAddressablePixel::BLEAdafruitAddressablePixel(void)
  : BLEService(UUID128_SERVICE), _pin(UUID128_CHR_PIN), _type(UUID128_CHR_TYPE),
    _data(UUID128_CHR_DATA), _bufsize(UUID128_CHR_BUFSIZE)
{
  _neo = NULL;
}

err_t BLEAdafruitAddressablePixel::begin(Adafruit_NeoPixel* neo_pixel)
{
  _neo = neo_pixel;
  uint16_t bufsize = _neo->numPixels()*3; // TODO actual bufsize for 3 RGB or 4 RGBW later
  return begin((uint8_t) _neo->getPin(), 0, bufsize); // TODO dotstart support
}

err_t BLEAdafruitAddressablePixel::begin(uint8_t pin, uint8_t type, uint16_t bufsize)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add Characteristic
  _pin.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  _pin.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  _pin.setFixedLen(1);
  VERIFY_STATUS( _pin.begin() );
  _pin.write8(pin);

  // Add Characteristic
  _type.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  _type.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  _type.setFixedLen(1);
  VERIFY_STATUS( _type.begin() );
  _type.write8(type);

  // Add Characteristic
  _data.setProperties(CHR_PROPS_WRITE);
  _data.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  // start (2 byte) + flags (1 byte)
  // TODO: for backward compatible with current CPB app, force to at least 10 pixels
  _data.setMaxLen(3 + 30 /* bufsize */);
  VERIFY_STATUS( _data.begin() );

  // Add Characteristic
  _bufsize.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  _bufsize.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  _bufsize.setFixedLen(2);
  VERIFY_STATUS( _bufsize.begin() );
  _bufsize.write16(bufsize);

  _data.setWriteCallback(pixel_data_write_cb, true);

  return ERROR_NONE;
}

void BLEAdafruitAddressablePixel::_pixel_write_handler(uint16_t conn_hdl, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;

  if (len < 3) return;

  uint16_t index;
  memcpy(&index, data, 2);
  uint8_t flag = data[2];

  // pixel staring buffer
  uint8_t* pixbuf = _neo->getPixels() + index;

  // limit copied bytes up to strip's boundary
  uint16_t copied_count = max16(_bufsize.read16(), index) - index;
  copied_count = min16(len-3, copied_count);

  if (copied_count) memcpy(pixbuf, data+3, copied_count);

  // show flag
  if ( flag & 0x01 ) _neo->show();
}

//--------------------------------------------------------------------+
// Static callbacks
//--------------------------------------------------------------------+
void BLEAdafruitAddressablePixel::pixel_data_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAdafruitAddressablePixel& svc = (BLEAdafruitAddressablePixel&) chr->parentService();
  svc._pixel_write_handler(conn_hdl, data, len);
}
