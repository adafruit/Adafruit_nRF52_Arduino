/**************************************************************************/
/*!
    @file     HAPAccessoryInfo.cpp
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

#include <bluefruit.h>
#include "HAPUuid.h"
#include "HAPAccessoryInfo.h"

HAPAccessoryInfo::HAPAccessoryInfo(void)
  : HAPService (HAP_UUID_SVC_ACCESSORY_INFO),
    _identify  (HAP_UUID_CHR_IDENTIFY      , BLE_GATT_CPF_FORMAT_UTF8S),
    _mfr       (HAP_UUID_CHR_MANUFACTURER  , BLE_GATT_CPF_FORMAT_UTF8S),
    _model     (HAP_UUID_CHR_MODEL         , BLE_GATT_CPF_FORMAT_UTF8S),
    _name      (HAP_UUID_CHR_NAME          , BLE_GATT_CPF_FORMAT_UTF8S),
    _serial    (HAP_UUID_CHR_SERIAL_NUMBER , BLE_GATT_CPF_FORMAT_UTF8S),
    _fw_rev    (HAP_UUID_CHR_FIRMWARE_REV  , BLE_GATT_CPF_FORMAT_UTF8S)
{

}

err_t HAPAccessoryInfo::begin(void)
{
  VERIFY_STATUS( HAPService::begin() ); // Invoke base class begin()

  // Identify
  _identify.setHapProperties(HAP_CHR_PROPS_SECURE_WRITE);
//  _identify.setFixedLen(1);
  VERIFY_STATUS( _identify.begin() );

  const char* strvals[] =
  {
      "Adafruit Industrial",
      "Adafruit Bluefruit nrf52",
      "Bluefruit52",
      getMcuUniqueID(),
      "0.9.0"
  };

  // Manufacturer
  _mfr.setHapProperties(HAP_CHR_PROPS_SECURE_READ);
  VERIFY_STATUS( _mfr.begin() );

  // Model
  _model.setHapProperties(HAP_CHR_PROPS_SECURE_READ);
  VERIFY_STATUS( _model.begin() );

  // Name
  _name.setHapProperties(HAP_CHR_PROPS_SECURE_READ);
  VERIFY_STATUS( _name.begin() );

  // Serial
  _serial.setHapProperties(HAP_CHR_PROPS_SECURE_READ);
  VERIFY_STATUS( _serial.begin() );

  // Firmware Revision
  _fw_rev.setHapProperties(HAP_CHR_PROPS_SECURE_READ);
  VERIFY_STATUS( _fw_rev.begin() );

  return ERROR_NONE;
}

