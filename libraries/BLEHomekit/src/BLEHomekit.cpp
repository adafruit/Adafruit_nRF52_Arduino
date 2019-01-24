/**************************************************************************/
/*!
    @file     BLEHomekit.cpp
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
#include "BLEHomekit.h"

uint16_t BLEHomekit::_gInstanceID = 1;

BLEHomekit::BLEHomekit()
 :  AccessoryInfo(), _protocol(), _pairing(),
   _lightbulb()
{

}

err_t BLEHomekit::begin(void)
{
  /*------------- Accessory Info Service -------------*/
  VERIFY_STATUS ( AccessoryInfo.begin() );

  /*------------- Protocol Info Service -------------*/
  VERIFY_STATUS( _protocol.begin() );

  /*------------- Pairing Service -------------*/
  VERIFY_STATUS ( _pairing.begin() );

  /*------------- LightBulb Service -------------*/
  VERIFY_STATUS( _lightbulb.begin() ) ;

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
    uint8_t  compatible_version;
  }data =
  {
      .company_id         = UUID16_COMPANY_ID_APPLE,
      .type               = 0x06,
      .adv_int_len        = ail,
      .status_flag        = 1, // bit0 = HAP Pairing Status Flag (1 not paired)
      .dev_id             = { 0x00},
      .category           = HAP_CAT_LIGHTBULB,
      .gsn                = 1,
      .config_num         = 1,
      .compatible_version = 2 // protocol version
  };

  Bluefruit.getAddr(data.dev_id);

  VERIFY_STATIC( sizeof(data) == 17);

  adv.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  adv.addManufacturerData(&data, sizeof(data));

  // Full name is not added, added full name to scan response
  if ( !adv.addName() )
  {
    Bluefruit.ScanResponse.clearData();
    Bluefruit.ScanResponse.addName();
  }

  return true;
}

