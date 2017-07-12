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
#include "utility/AdaCallback.h"

#define BLE_ADV_INTERVAL_FAST_DFLT       32  // 20    ms (in 0.625 ms unit)
#define BLE_ADV_INTERVAL_SLOW_DFLT       244 // 152.5 ms (in 0.625 ms unit)

#define BLE_ADV_TIMEOUT_DFLT             30  // in seconds

/*------------------------------------------------------------------*/
/* BLEAdvertisingData shared between ADV and ScanResponse
 *------------------------------------------------------------------*/
BLEAdvertisingData::BLEAdvertisingData(void)
{
  _count = 0;
  varclr(_data);
}

bool BLEAdvertisingData::addData(uint8_t type, const void* data, uint8_t len)
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

bool BLEAdvertisingData::addUuid(BLEUuid bleuuid)
{
  switch ( bleuuid.size() )
  {
    case 16:
      return addData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, &bleuuid._uuid.uuid, 2);
    break;

    case 128:
      return addData(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, bleuuid._uuid128, 16);
    break;

    default: break;
  }

  return false;
}

bool BLEAdvertisingData::addService(BLEService& service)
{
  return addUuid(service.uuid);
}

bool BLEAdvertisingData::addService(BLEClientService& service)
{
  // Central service is added to Solicitation UUID
  switch ( service.uuid.size() )
  {
    case 16:
      return addData(BLE_GAP_AD_TYPE_SOLICITED_SERVICE_UUIDS_16BIT, &service.uuid._uuid.uuid, 2);
    break;

    case 128:
      return addData(BLE_GAP_AD_TYPE_SOLICITED_SERVICE_UUIDS_128BIT, service.uuid._uuid128, 16);
    break;

    default: break;
  }

  return false;
}

// Add Name to Adv packet, use setName() to set
bool BLEAdvertisingData::addName(void)
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
bool BLEAdvertisingData::addTxPower(void)
{
  int8_t tx_power = Bluefruit.getTxPower();
  return addData(BLE_GAP_AD_TYPE_TX_POWER_LEVEL, &tx_power, 1);
}

bool BLEAdvertisingData::addFlags(uint8_t flags)
{
  return addData(BLE_GAP_AD_TYPE_FLAGS, &flags, 1);
}

bool BLEAdvertisingData::addAppearance(uint16_t appearance)
{
  return addData(BLE_GAP_AD_TYPE_APPEARANCE, &appearance, 2);
}

/*------------------------------------------------------------------*/
/* CUSTOM API
 *------------------------------------------------------------------*/
uint8_t BLEAdvertisingData::count(void)
{
  return _count;
}

uint8_t* BLEAdvertisingData::getData(void)
{
  return _data;
}

bool BLEAdvertisingData::setData(uint8_t const * data, uint8_t count)
{
  VERIFY( data && (count <= BLE_GAP_ADV_MAX_SIZE) );

  memcpy(_data, data, count);
  _count = count;

  return true;
}

void BLEAdvertisingData::clearData(void)
{
  _count = 0;
  varclr(_data);
}

/*------------------------------------------------------------------*/
/* BLEAdvertising only
 *------------------------------------------------------------------*/
BLEAdvertising::BLEAdvertising(void)
{
  _timeout_sec   = BLE_ADV_TIMEOUT_DFLT;
  _type          = BLE_GAP_ADV_TYPE_ADV_IND;

  _fast_interval = BLE_ADV_INTERVAL_FAST_DFLT;
  _slow_interval = BLE_ADV_INTERVAL_SLOW_DFLT;

  _started_sec   = 0;
  _stop_sec      = 0;
  _stop_cb       = NULL;
}

void BLEAdvertising::setTimeout(uint16_t sec)
{
  _timeout_sec = sec;
}

void BLEAdvertising::setType(uint8_t adv_type)
{
  _type = adv_type;
}

void BLEAdvertising::setInterval(uint16_t fast, uint16_t slow)
{
  _fast_interval = fast;
  _slow_interval = slow;
}

void BLEAdvertising::setIntervalMS(uint16_t fast, uint16_t slow)
{
  _fast_interval = MS1000TO625(fast);
  _slow_interval = MS1000TO625(slow);
}

void BLEAdvertising::setStopCallback(stop_callback_t fp)
{
  fp = _stop_cb;
}

bool BLEAdvertising::setBeacon(BLEBeacon& beacon)
{
  return beacon.start(*this);
}

bool BLEAdvertising::_start(uint16_t interval)
{
  // ADV Params
  ble_gap_adv_params_t adv_para =
  {
      .type        = _type              ,
      .p_peer_addr = NULL               , // Undirected advertisement
      .fp          = BLE_GAP_ADV_FP_ANY ,
      .p_whitelist = NULL               ,
      .interval    = interval           , // advertising interval (in units of 0.625 ms)
      .timeout     = _timeout_sec
  };

  VERIFY_STATUS( sd_ble_gap_adv_start(&adv_para), false );
  Bluefruit._startConnLed(); // start blinking

  return true;
}

bool BLEAdvertising::start(uint32_t stop_sec)
{
  // Configure Data
  VERIFY_STATUS( sd_ble_gap_adv_data_set(_data, _count, Bluefruit.ScanResponse.getData(), Bluefruit.ScanResponse.count()), false );
  VERIFY( _start(_fast_interval) );

  _started_sec = 0; // reset established time

  return true;
}

bool BLEAdvertising::stop(void)
{
  VERIFY_STATUS( sd_ble_gap_adv_stop(), false);

  Bluefruit._stopConnLed(); // stop blinking

  return true;
}


void BLEAdvertising::_eventHandler(ble_evt_t* evt)
{
  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;

      if ( para->role == BLE_GAP_ROLE_PERIPH)
      {
        // Turn on Conn LED
        Bluefruit._stopConnLed();
        Bluefruit._setConnLed(true);
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      if ( BLE_GAP_ROLE_PERIPH == Bluefruit.Gap.getRole(evt->evt.common_evt.conn_handle) )
      {
        // Turn off Conn LED
        Bluefruit._setConnLed(false);
      }
    break;

    case BLE_GAP_EVT_TIMEOUT:
      if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
      {
        _started_sec += _timeout_sec;

        // Continue to adv if stop is not configured or time not reach it yet
        if ( (_stop_sec == 0) || (_started_sec < _stop_sec) )
        {
          _start(_slow_interval);
        }else
        {
          Bluefruit._stopConnLed(); // stop blinking

          // invoke stop callback
          if (_stop_cb) ada_callback(NULL, _stop_cb);
        }
      }
    break;

    default: break;
  }
}

