/**************************************************************************/
/*!
    @file     BLEBeacon.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "bluefruit.h"

void BLEBeacon::_init(void)
{
  _manufacturer_id = UUID16_COMPANY_ID_APPLE; // default is Apple
  _uuid128 = NULL;

  _major_be = _minor_be = 0;
  _rssi_at_1m = -54;
}

BLEBeacon::BLEBeacon(void)
{
  _init();
}

BLEBeacon::BLEBeacon(uint8_t const uuid128[16])
{
  _init();
  _uuid128 = uuid128;
}

BLEBeacon::BLEBeacon(uint8_t const uuid128[16], uint16_t major, uint16_t minor, int8_t rssi)
{
  _init();
  _uuid128 = uuid128;
  _major_be = __swap16(major);
  _minor_be = __swap16(minor);
  _rssi_at_1m = rssi;
}

void BLEBeacon::setManufacturer(uint16_t manfacturer)
{
  _manufacturer_id = manfacturer;
}

void BLEBeacon::setUuid(uint8_t const uuid128[16])
{
  _uuid128 = uuid128;
}

void BLEBeacon::setMajorMinor(uint16_t major, uint16_t minor)
{
  _major_be = __swap16(major);
  _minor_be = __swap16(minor);
}

void BLEBeacon::setRssiAt1m(int8_t rssi)
{
  _rssi_at_1m = rssi;
}

bool BLEBeacon::start(void)
{
  return start(Bluefruit.Advertising);
}

bool BLEBeacon::start(BLEAdvertising& adv)
{
  adv.clearData();

  struct ATTR_PACKED
  {
    uint16_t manufacturer;

    uint8_t  beacon_type;
    uint8_t  beacon_len;

    uint8_t  uuid128[16];
    uint16_t major;
    uint16_t minor;
    int8_t   rssi_at_1m;
  } beacon_data =
  {
      .manufacturer = _manufacturer_id,
      .beacon_type = 0x02,
      .beacon_len  = sizeof(beacon_data) - 4, // len of uuid + major + minor + rssi
      .uuid128 = { 0 },
      .major = _major_be,
      .minor = _minor_be,
      .rssi_at_1m = _rssi_at_1m
  };

  VERIFY_STATIC(sizeof(beacon_data) == 25);

  memcpy(beacon_data.uuid128, _uuid128, 16);

  adv.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  return adv.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &beacon_data, sizeof(beacon_data));
}
