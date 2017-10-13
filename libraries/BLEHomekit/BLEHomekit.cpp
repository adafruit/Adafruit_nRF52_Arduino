/**************************************************************************/
/*!
    @file     BLEHomekit.cpp
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
#include "BLEHomekit.h"

#define HOMEKIT_PROTOCOL_VERION           2
#define HOMEKIT_PROTOCOL_VERSION_STR      "02.01.00"

BLEHomekit::BLEHomekit()
 : _protocol_info(HAP_UUID_SVC_PROTOCOL_INFO), AccessoryInfo(),
   _pairing(HAP_UUID_SVC_PAIRING), _pair_setup(HAP_UUID_CHR_PAIR_SETUP), _pair_verify(HAP_UUID_CHR_PAIR_VERIFY),
   _pair_features(HAP_UUID_SVC_PAIR_FEATURE), _pair_pairing(HAP_UUID_SVC_PAIR_PAIRING),
   _lightbulb(HAP_UUID_SVC_LIGHT_BULB), _on(HAP_UUID_CHR_ON)
{

}

err_t addServiceInstanceID(uint16_t id)
{
  BLECharacteristic chr(HAP_UUID_CHR_SERVICE_ID);
  chr.setTempMemory();

  chr.setProperties(CHR_PROPS_READ);
  chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chr.setFixedLen(2);
  VERIFY_STATUS( chr.begin() );

  chr.write( id );

  return ERROR_NONE;
}

//err_t addChrInstance()

err_t BLEHomekit::begin()
{
  /*------------- Accessory Info Service -------------*/
  VERIFY_STATUS ( AccessoryInfo.begin() );

  /*------------- Protocol Info Service -------------*/
  VERIFY_STATUS( _protocol_info.begin() );
  {
    BLECharacteristic chr(HAP_UUID_CHR_VERSION);

    chr.setTempMemory(); // ready-only, not included in Gatt list
    chr.setProperties(CHR_PROPS_READ);
    chr.setPermission(/*SECMODE_ENC_NO_MITM*/SECMODE_OPEN, SECMODE_NO_ACCESS);
    chr.setFixedLen(strlen(HOMEKIT_PROTOCOL_VERSION_STR));

    VERIFY_STATUS( chr.begin() );
    chr.write(HOMEKIT_PROTOCOL_VERSION_STR);
  }


  /*------------- Pairing Service -------------*/
  VERIFY_STATUS( _pairing.begin() );
  {
    addServiceInstanceID(3);

    // TODO read, write using auth
    _pair_setup.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    _pair_setup.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    _pair_setup.setMaxLen(100);
    VERIFY_STATUS( _pair_setup.begin() );

    _pair_verify.setProperties(CHR_PROPS_READ);
    _pair_verify.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _pair_verify.setMaxLen(100);
    VERIFY_STATUS( _pair_verify.begin() );

    _pair_features.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    _pair_features.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _pair_features.setFixedLen(1);
    VERIFY_STATUS( _pair_features.begin() );
    _pair_features.write( (uint8_t) 0x01); // support HAP pairing

    _pair_pairing.setProperties(CHR_PROPS_READ);
    _pair_pairing.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    _pair_pairing.setMaxLen(100);
    VERIFY_STATUS( _pair_pairing.begin() );
  }

  /*------------- LightBulb Service -------------*/
  VERIFY_STATUS( _lightbulb.begin() ) ;
  {
    addServiceInstanceID(4);

    // Name chr
    BLECharacteristic chr(HAP_UUID_CHR_VERSION);

    chr.setTempMemory(); // ready-only, not included in Gatt list
    chr.setProperties(CHR_PROPS_READ);
    chr.setPermission(/*SECMODE_ENC_NO_MITM*/SECMODE_OPEN, SECMODE_NO_ACCESS);
    chr.setFixedLen(strlen(HOMEKIT_PROTOCOL_VERSION_STR));

    VERIFY_STATUS( chr.begin() );
    chr.write(HOMEKIT_PROTOCOL_VERSION_STR);

    // ON char
    _on.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY );
    _on.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    _on.setFixedLen(1);
    VERIFY_STATUS( _on.begin() );
    _on.write( (uint8_t) 0x00 );
  }

  return ERROR_NONE;
}

bool BLEHomekit::setAdv(BLEAdvertisingData& adv_ref)
{
  VERIFY(&Bluefruit.Advertising == &adv_ref);
  BLEAdvertising& adv = (BLEAdvertising&) adv_ref;

  // Calculate AIL field
  uint16_t interval_ms = MS625TO1000( adv.getInterval() );
  uint8_t ail;

  if      ( interval_ms > 2500 ) ail = 0xE0;
  else if ( interval_ms > 1250 ) ail = 0xC0;

  else if ( interval_ms > 500  ) ail = 0xA0;
  else if ( interval_ms > 300  ) ail = 0x80;
  else if ( interval_ms > 100  ) ail = 0x60;
  else if ( interval_ms > 25   ) ail = 0x40;
  else                           ail = 0x20;

  ail |= 0x0D;

  struct ATTR_PACKED
  {
    uint16_t company_id;
    uint8_t  type;
    uint8_t  adv_int_len;
    uint8_t  status_flag;
    uint8_t  dev_id[6];
    uint16_t category;
    uint16_t gsn;
    uint8_t  config_num;
    uint8_t  compatible_verion;
  }data =
  {
      .company_id        = UUID16_COMPANY_ID_APPLE,
      .type              = 0x06,
      .adv_int_len       = ail,
      .status_flag       = 1, // bit0 = HAP Pairing Status Flag (1 not paired)
      .dev_id            = { 0x0a, 0xbc, 0x12, 0x33, 0x56, 0x98 },
      .category          = HAP_CAT_LIGHTBULB,
      .gsn               = 1,
      .config_num        = 1,
      .compatible_verion = HOMEKIT_PROTOCOL_VERION
  };

  VERIFY_STATIC( sizeof(data) == 17);

  adv.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  return adv.addManufacturerData(&data, sizeof(data));
}

