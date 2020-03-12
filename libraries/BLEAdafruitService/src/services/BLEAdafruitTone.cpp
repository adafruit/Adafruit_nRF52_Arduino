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

const uint8_t BLEAdafruitTone::UUID128_SERVICE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x00, 0x0C, 0xAF, 0xAD
};

const uint8_t BLEAdafruitTone::UUID128_CHR_TONE[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x0C, 0xAF, 0xAD
};

BLEAdafruitTone::BLEAdafruitTone(void)
  : BLEService(UUID128_SERVICE), _tone(UUID128_CHR_TONE)
{
  _pin = -1;
}

err_t BLEAdafruitTone::begin(int pin)
{
  _pin = pin;

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Setup Characteristic
  _tone.setProperties(CHR_PROPS_WRITE);
  _tone.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  _tone.setFixedLen(6); // uint16_t freq, uint32_t duration
  _tone.setWriteCallback(tone_write_cb, true);
  VERIFY_STATUS( _tone.begin() );

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);

  return ERROR_NONE;
}

void BLEAdafruitTone::tone_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEAdafruitTone& svc = (BLEAdafruitTone&) chr->parentService();

  struct ATTR_PACKED {
      uint16_t freq;
      uint32_t duration;
  } tone_data;

  // invalid length
  if (len != sizeof(tone_data)) return;

  // extra data
  memcpy(&tone_data, data, len);

  // frequency = 0 means no tone
  int pin = svc._pin;

  if (tone_data.freq == 0)
  {
    noTone(pin);
    digitalWrite(pin, LOW);
  }else
  {
    tone(pin, tone_data.freq, tone_data.duration);
  }
}
