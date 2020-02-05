/**************************************************************************/
/*!
    @file     BLEUuid.cpp
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

#include "BLEUuid.h"

const uint8_t UUID128_CHR_ADAFRUIT_MEASUREMENT_PERIOD[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x01, 0x00, 0xAF, 0xAD
};

const uint8_t UUID128_CHR_ADAFRUIT_VERSION[16] =
{
  0xB8, 0x6c, 0x75, 0x05, 0xE9, 0x25, 0xBD, 0x93,
  0xA8, 0x42, 0x32, 0xC3, 0x02, 0x00, 0xAF, 0xAD
};

void BLEUuid::set(uint16_t uuid16)
{
  _uuid.type = BLE_UUID_TYPE_BLE;
  _uuid.uuid = uuid16;

  _uuid128   = NULL;
}

void BLEUuid::set(uint8_t const uuid128[16])
{
  _uuid.type = BLE_UUID_TYPE_UNKNOWN;
  _uuid.uuid = 0;

  _uuid128   = uuid128;
}

bool BLEUuid::get(uint16_t* uuid16 ) const
{
  *uuid16 = _uuid.uuid;

  return true;
}

bool BLEUuid::get(uint8_t uuid128[16])
{
  if (_uuid.type < BLE_UUID_TYPE_VENDOR_BEGIN ) return false;

  if ( _uuid128 )
  {
    memcpy(uuid128, _uuid128, 16);
  }else
  {
    uint8_t len = 16;
    VERIFY_STATUS( sd_ble_uuid_encode(&_uuid, &len, uuid128), false);
    VERIFY(len == 16);
  }

  return true;
}

/**
 * Get size of uuid in BIT
 * @return 16, 32 or 128
 */
size_t BLEUuid::size (void) const
{
  // uuid 16
  if (_uuid.type == BLE_UUID_TYPE_BLE ) return 16;
  if (_uuid128 != NULL || _uuid.type >= BLE_UUID_TYPE_VENDOR_BEGIN) return 128;

  // unknown
  return 0;
}

bool BLEUuid::begin(void)
{
  /* Add base uuid and decode to get uuid16
   * This should cover the already added base uuid128 previously
   */
  if (_uuid.type == BLE_UUID_TYPE_UNKNOWN && _uuid128 != NULL )
  {
    (void) sd_ble_uuid_vs_add( (ble_uuid128_t const*) _uuid128, &_uuid.type );
    VERIFY_STATUS( sd_ble_uuid_decode(16, _uuid128, &_uuid), false );
  }

  return true;
}

bool BLEUuid::operator== (const BLEUuid& uuid) const
{
  return (this->_uuid.type == uuid._uuid.type) && (this->_uuid.uuid == uuid._uuid.uuid);
}

bool BLEUuid::operator!= (const BLEUuid& uuid) const
{
  return !(*this == uuid);
}

bool BLEUuid::operator== (const ble_uuid_t uuid) const
{
  return (this->_uuid.type == uuid.type) && (this->_uuid.uuid == uuid.uuid);
}

bool BLEUuid::operator!= (const ble_uuid_t uuid) const
{
  return !(*this == uuid);
}

// Overload copy operator to allow initialization from other type
BLEUuid& BLEUuid::operator=(const uint16_t uuid)
{
  set(uuid);
  return *this;
}

BLEUuid& BLEUuid::operator=(uint8_t const uuid128[16])
{
  set(uuid128);
  return *this;
}

BLEUuid& BLEUuid::operator=(ble_uuid_t uuid)
{
  _uuid = uuid;
  return *this;
}
