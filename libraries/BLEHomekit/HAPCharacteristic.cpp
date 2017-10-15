/**************************************************************************/
/*!
    @file     HAPCharacteristic.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include <Arduino.h>
#include <bluefruit.h>
#include "BLEHomekit.h"

BLEUuid HAPCharacteristic::_g_uuid_cid(HAP_UUID_CHR_CHARACTERISTIC_ID);

err_t HAPCharacteristic::begin(void)
{
  VERIFY_STATUS( BLECharacteristic::begin() );
  VERIFY_STATUS( _addChrIdDescriptor() );
  VERIFY_STATUS( _addFormatDescriptor() );

  return ERROR_NONE;
}

/**
 * Add Characteristic Instance ID descriptor
 * @return status code
 */
err_t HAPCharacteristic::_addChrIdDescriptor(void)
{
  // Add Descriptor UUID if not yet added
  if (_g_uuid_cid._uuid.type == BLE_UUID_TYPE_UNKNOWN)
  {
    _g_uuid_cid.begin();
  }

  _cid = BLEHomekit::_gInstanceID++;

  ble_gatts_attr_md_t cid_md =
  {
      .read_perm  = BLE_SECMODE_OPEN,
      .write_perm = BLE_SECMODE_NO_ACCESS,
      .vlen       = 0,
      .vloc       = BLE_GATTS_VLOC_STACK
  };

  ble_gatts_attr_t cid_desc =
  {
      .p_uuid    = &_g_uuid_cid._uuid,
      .p_attr_md = &cid_md,
      .init_len  = 2,
      .init_offs = 0,
      .max_len   = 2,
      .p_value   = (uint8_t*) &_cid
  };

  uint16_t ref_hdl;
  VERIFY_STATUS ( sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &cid_desc, &ref_hdl) );

  return ERROR_NONE;
}

err_t HAPCharacteristic::_addFormatDescriptor(void)
{


  return ERROR_NONE;
}

