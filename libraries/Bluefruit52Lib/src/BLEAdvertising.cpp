/**************************************************************************/
/*!
    @file     BLEAdvertising.cpp
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

#include "bluefruit.h"

#define GAP_ADV_INTERVAL_MS              20
#define GAP_ADV_TIMEOUT_S                30

BLEAdvertising::BLEAdvertising(void)
{
  _count = 0;
  varclr(_data);
}

bool BLEAdvertising::start(uint8_t mode)
{
  (void) mode; // not used yet

  // Only allow to call start with Bluefruit.Advertising.start()
  VERIFY( this == &Bluefruit.Advertising );

  // Configure Data
  VERIFY_STATUS( sd_ble_gap_adv_data_set(_data, _count, Bluefruit.ScanResponse._data, Bluefruit.ScanResponse._count), false );

  // ADV Params
  ble_gap_adv_params_t adv_para =
  {
      .type        = BLE_GAP_ADV_TYPE_ADV_IND,
      .p_peer_addr = NULL                            , // Undirected advertisement
      .fp          = BLE_GAP_ADV_FP_ANY              ,
      .p_whitelist = NULL                            ,
      .interval    = MS1000TO625(GAP_ADV_INTERVAL_MS), // advertising interval (in units of 0.625 ms)
      .timeout     = GAP_ADV_TIMEOUT_S
  };

  VERIFY_STATUS( sd_ble_gap_adv_start(&adv_para), false );

  Bluefruit.startConnLed(); // start blinking

  return ERROR_NONE;
}

bool BLEAdvertising::stop(void)
{
  // Only allow to call start with Bluefruit.Advertising.start()
  VERIFY( this == &Bluefruit.Advertising );
  VERIFY_STATUS( sd_ble_gap_adv_stop(), false);

  Bluefruit.stopConnLed(); // stop blinking

  return true;
}


bool BLEAdvertising::addData(uint8_t type, const void* data, uint8_t len)
{
  VERIFY( _count + len + 2 <= BLE_GAP_ADV_MAX_SIZE );

  uint8_t* adv_data = &_data[_count];

  // len (1+data), type, data
  *adv_data++ = (len+1);
  *adv_data++ = type;
  memcpy(adv_data, data, len);

  _count = _count + len + 2;

  return true;
}

bool BLEAdvertising::addUuid(uint16_t uuid16)
{
  return addData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, &uuid16, 2);
}

bool BLEAdvertising::addUuid(uint8_t const  uuid128[])
{
  return addData(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, uuid128, 16);
}

bool BLEAdvertising::addService(BLEService& service)
{
  // UUID128
  if ( service.uuid._uuid128 )
  {
    return addUuid(service.uuid._uuid128);
  }else
  {
    return addUuid(service.uuid._uuid.uuid);
  }
}

// Add Name to Adv packet, use setName() to set
bool BLEAdvertising::addName(void)
{
  const char* name = Bluefruit.getName();

  uint8_t type = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
  uint8_t len  = strlen(name);

  // not enough for full name, chop it
  if (_count + len + 2 > BLE_GAP_ADV_MAX_SIZE)
  {
    type = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    len  = BLE_GAP_ADV_MAX_SIZE - (_count+2);
  }

  return addData(type, name, len);
}

// tx power is set by setTxPower
bool BLEAdvertising::addTxPower(void)
{
  int8_t tx_power = Bluefruit.getTxPower();
  return addData(BLE_GAP_AD_TYPE_TX_POWER_LEVEL, &tx_power, 1);
}

bool BLEAdvertising::addFlags(uint8_t flags)
{
  return addData(BLE_GAP_AD_TYPE_FLAGS, &flags, 1);
}

bool BLEAdvertising::addApperance(uint16_t appearance)
{
  return addData(BLE_GAP_AD_TYPE_APPEARANCE, &appearance, 2);
}

bool BLEAdvertising::setBeacon(BLEBeacon& beacon)
{
  return beacon.start(*this);
}

/*------------------------------------------------------------------*/
/* CUSTOM API
 *------------------------------------------------------------------*/
uint8_t BLEAdvertising::count(void)
{
  return _count;
}

char* BLEAdvertising::getData(void)
{
  return (char*) _data;
}

bool BLEAdvertising::setData(uint8_t const * data, uint8_t count)
{
  VERIFY( data && (count <= BLE_GAP_ADV_MAX_SIZE) );

  memcpy(_data, data, count);
  _count = count;

  return true;
}

void BLEAdvertising::clearData(void)
{
  _count = 0;
  varclr(_data);
}

