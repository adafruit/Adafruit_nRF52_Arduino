/**************************************************************************/
/*!
    @file     HAPPairing.cpp
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

#include <bluefruit.h>
#include "HAPUuid.h"
#include "HAPPairing.h"

HAPPairing::HAPPairing(void)
  : HAPService(HAP_UUID_SVC_PAIRING),
    _setup    (HAP_UUID_CHR_PAIR_SETUP   , BLE_GATT_CPF_FORMAT_STRUCT),
    _verify   (HAP_UUID_CHR_PAIR_VERIFY  , BLE_GATT_CPF_FORMAT_STRUCT),
    _features (HAP_UUID_SVC_PAIR_FEATURE , BLE_GATT_CPF_FORMAT_STRUCT),
    _pairing  (HAP_UUID_SVC_PAIR_PAIRING , BLE_GATT_CPF_FORMAT_STRUCT)
{

}

err_t HAPPairing::begin(void)
{
  VERIFY_STATUS( HAPService::begin() ); // Invoke base class begin()

  // TODO read, write using auth
  _setup.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _setup.setMaxLen(BLE_GATTS_VAR_ATTR_LEN_MAX);
  VERIFY_STATUS( _setup.begin() );

  _verify.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _verify.setMaxLen(100);
  VERIFY_STATUS( _verify.begin() );

  _features.setHapProperties(HAP_CHR_PROPS_READ);
  _features.setMaxLen(100);
  VERIFY_STATUS( _features.begin() );
  //_features.write( (uint8_t) 0x01); // support HAP pairing

  _pairing.setHapProperties(HAP_CHR_PROPS_SECURE_READ | HAP_CHR_PROPS_SECURE_WRITE);
  _pairing.setMaxLen(100);
  VERIFY_STATUS( _pairing.begin() );

  return ERROR_NONE;
}
