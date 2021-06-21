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

//--------------------------------------------------------------------+
// Constructor
//--------------------------------------------------------------------+

BLEUuid::BLEUuid(void)
{
  _uuid.type = BLE_UUID_TYPE_UNKNOWN;
  _uuid.uuid = 0;

  _uuid128   = NULL;
  _str       = NULL;
}

BLEUuid::BLEUuid(uint16_t uuid16)
{
  set(uuid16);
}

BLEUuid::BLEUuid(const char *str)
{
  set(str);
}

BLEUuid::BLEUuid(uint8_t const uuid128[16])
{
  set(uuid128);
}

BLEUuid::BLEUuid(ble_uuid_t uuid)
{
  _uuid = uuid;

  _uuid128 = NULL;
  _str = NULL;
}

BLEUuid::~BLEUuid()
{
  if (_str && _uuid128)
  {
    // don't even free _uuid128, it could be pointer copyied to other
    // check again if it is a major memory leak
  }
}

// Get size of uuid in bit length. return 16, 32 or 128
size_t BLEUuid::size (void) const
{
  // uuid 16
  if (_uuid.type == BLE_UUID_TYPE_BLE ) return 16;
  if (_uuid128 != NULL || _str != NULL || _uuid.type >= BLE_UUID_TYPE_VENDOR_BEGIN) return 128;

  // unknown
  return 0;
}

uint8_t* parse_str2uuid128(const char* str)
{
  uint8_t* u128 = (uint8_t*) rtos_malloc(16);
  uint8_t len = 0;

  // str is input as big endian
  for(int i = strlen(str)-1; i >= 0 && len < 16; i -= 2 )
  {
    if (str[i] == '-' )
    {
      // skip dash
      i++;
    }else
    {
      char temp[3] = { 0 };
      temp[0] = str[i-1];
      temp[1] = str[i];

      u128[len++] = (uint8_t) strtoul(temp, NULL, 16);
    }
  }

  return u128;
}

bool BLEUuid::begin(void)
{
  // Add base uuid and decode to get uuid16
  // This should cover the already added base uuid128 previously
  if (_uuid.type == BLE_UUID_TYPE_UNKNOWN)
  {
    // allocate uuid128 and parse str (str for uuid16 already parsed at this point)
    if (_str)
    {
      _uuid128 = parse_str2uuid128(_str);
    }

    if (_uuid128 != NULL )
    {
      (void) sd_ble_uuid_vs_add( (ble_uuid128_t const*) _uuid128, &_uuid.type );
      VERIFY_STATUS( sd_ble_uuid_decode(16, _uuid128, &_uuid), false );
    }
  }

  return true;
}

String BLEUuid::toString(void) const
{
  if (_str) return _str;

  char result[38];
  if (this->size() == 16)
  {
    sprintf(result, "%02X%02X", highByte(_uuid.uuid), lowByte(_uuid.uuid));
  }else
  {
    // uuid is little endian
    sprintf(result, "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
            _uuid128[15], _uuid128[14], _uuid128[13], _uuid128[12], _uuid128[11], _uuid128[10], _uuid128[ 9], _uuid128[ 8],
            _uuid128[ 7], _uuid128[ 6], _uuid128[ 5], _uuid128[ 4], _uuid128[ 3], _uuid128[ 2], _uuid128[ 1], _uuid128[ 0]);
  }

  return result;
}

//--------------------------------------------------------------------+
// Set & Get
//--------------------------------------------------------------------+
void BLEUuid::set(uint16_t uuid16)
{
  _uuid.type = BLE_UUID_TYPE_BLE;
  _uuid.uuid = uuid16;

  _uuid128   = NULL;
  _str       = NULL;
}

void BLEUuid::set(uint8_t const uuid128[16])
{
  _uuid128   = uuid128;

  _uuid.type = BLE_UUID_TYPE_UNKNOWN;
  _uuid.uuid = 0;
  _str       = NULL;
}

void BLEUuid::set(const char* str)
{
  // Check if str is uuid16
  if (strlen(str) == 4)
  {
    uint16_t uuid16 = strtoul(str, NULL, 16);
    set(uuid16);
  }else
  {
    _str       = str;

    _uuid.type = BLE_UUID_TYPE_UNKNOWN;
    _uuid.uuid = 0;
    _uuid128   = NULL;
  }
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

//--------------------------------------------------------------------+
// Comparison
//--------------------------------------------------------------------+

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

//--------------------------------------------------------------------+
// Copy operator
//--------------------------------------------------------------------+

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

BLEUuid& BLEUuid::operator=(const char* str)
{
  set(str);
  return *this;
}


BLEUuid& BLEUuid::operator=(ble_uuid_t uuid)
{
  _uuid = uuid;
  return *this;
}
