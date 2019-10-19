/**************************************************************************/
/*!
    @file     BLEAdvertising.cpp
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
  VERIFY( _count + len + 2 <= BLE_GAP_ADV_SET_DATA_SIZE_MAX );

  uint8_t* adv_data = &_data[_count];

  // len (1+data), type, data
  *adv_data++ = (len+1);
  *adv_data++ = type;
  memcpy(adv_data, data, len);

  _count = _count + len + 2;

  return true;
}

/*------------------------------------------------------------------*/
/* Adding UUID
 *------------------------------------------------------------------*/
bool BLEAdvertisingData::addUuid(BLEUuid bleuuid)
{
  return addUuid(&bleuuid, 1);
}

bool BLEAdvertisingData::addUuid(BLEUuid bleuuid1, BLEUuid bleuuid2)
{
  BLEUuid bleuuid[] = { bleuuid1, bleuuid2 };
  return addUuid(bleuuid, 2);
}

bool BLEAdvertisingData::addUuid(BLEUuid bleuuid1, BLEUuid bleuuid2, BLEUuid bleuuid3)
{
  BLEUuid bleuuid[] = { bleuuid1, bleuuid2, bleuuid3};
  return addUuid(bleuuid, 3);
}

bool BLEAdvertisingData::addUuid(BLEUuid bleuuid1, BLEUuid bleuuid2, BLEUuid bleuuid3, BLEUuid bleuuid4)
{
  BLEUuid bleuuid[] = { bleuuid1, bleuuid2, bleuuid3, bleuuid4 };
  return addUuid(bleuuid, 4);
}

bool BLEAdvertisingData::addUuid(BLEUuid bleuuid[], uint8_t count)
{
  uint16_t uuid16_list[15];
  uint8_t  uuid16_count = 0;

  uint8_t const* uuid128 = NULL;

  for(uint8_t i=0; i<count; i++)
  {
    switch ( bleuuid[i].size() )
    {
      case 16:
        uuid16_list[uuid16_count++] = bleuuid[i]._uuid.uuid;
      break;

      case 128:
        // cannot fit more than one uuid128
        if ( uuid128 ) return false;
        uuid128 = bleuuid[i]._uuid128;
      break;

      default: break;
    }
  }

  if (uuid16_count)
  {
    VERIFY( addData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, uuid16_list, 2*uuid16_count) );
  }

  if (uuid128)
  {
    VERIFY( addData(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, uuid128, 16) );
  }

  return true;
}

/*------------------------------------------------------------------*/
/* Adding Service's UUID
 *------------------------------------------------------------------*/
bool BLEAdvertisingData::addService(BLEService& service)
{
  return addUuid(service.uuid);
}

bool BLEAdvertisingData::addService(BLEService& service1, BLEService& service2)
{
  return addUuid(service1.uuid, service2.uuid);
}

bool BLEAdvertisingData::addService(BLEService& service1, BLEService& service2, BLEService& service3)
{
  return addUuid(service1.uuid, service2.uuid, service3.uuid);
}

bool BLEAdvertisingData::addService(BLEService& service1, BLEService& service2, BLEService& service3, BLEService& service4)
{
  return addUuid(service1.uuid, service2.uuid, service3.uuid, service4.uuid);
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

/*------------------------------------------------------------------*/
/* Adding Others
 *------------------------------------------------------------------*/

/**
 * Add Name to Adv packet, use setName() to set
 * @return true if full name is added, false if shorten name or not enough data to add name
 */
bool BLEAdvertisingData::addName(void)
{
  char name[BLE_GAP_ADV_SET_DATA_SIZE_MAX+1];

  uint8_t type = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
  uint8_t len  = Bluefruit.getName(name, sizeof(name));

  // not enough for full name, chop it
  if (_count + len + 2 > BLE_GAP_ADV_SET_DATA_SIZE_MAX)
  {
    type = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    len  = BLE_GAP_ADV_SET_DATA_SIZE_MAX - (_count+2);
  }

  VERIFY( addData(type, name, len) );

  return type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
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

bool BLEAdvertisingData::addManufacturerData(const void* data, uint8_t count)
{
  return addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, data, count);
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
  VERIFY( data && (count <= BLE_GAP_ADV_SET_DATA_SIZE_MAX) );

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
  _hdl                 = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
  _type                = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
  _start_if_disconnect = true;
  _runnning            = false;

  _conn_mask           = 0;

  _fast_interval       = BLE_ADV_INTERVAL_FAST_DFLT;
  _slow_interval       = BLE_ADV_INTERVAL_SLOW_DFLT;
  _active_interval     = _fast_interval;

  _fast_timeout        = BLE_ADV_FAST_TIMEOUT_DFLT;
  _stop_timeout        = _left_timeout = 0;
  _stop_cb             = NULL;
  _slow_cb             = NULL;
}

void BLEAdvertising::setFastTimeout(uint16_t sec)
{
  _fast_timeout = sec;
}

void BLEAdvertising::setType(uint8_t adv_type)
{
  _type = adv_type;
}

/**
 * Set Interval in unit of 0.625 ms
 * @param fast  Interval that is used in the first n seconds (configurable)
 * @param slow  Interval that is used after fast timeout
 */
void BLEAdvertising::setInterval(uint16_t fast, uint16_t slow)
{
  _fast_interval   = fast;
  _slow_interval   = slow;

  // default is fast since it will be advertising first
  _active_interval = _fast_interval;
}

void BLEAdvertising::setIntervalMS(uint16_t fast, uint16_t slow)
{
  setInterval(MS1000TO625(fast), MS1000TO625(slow));
}

/**
 * Get current active interval
 * @return Either slow or fast interval in unit of 0.625 ms
 */
uint16_t BLEAdvertising::getInterval(void)
{
  return _active_interval;
}

void BLEAdvertising::setSlowCallback(slow_callback_t fp)
{
  _slow_cb = fp;
}

void BLEAdvertising::setStopCallback(stop_callback_t fp)
{
  _stop_cb = fp;
}

bool BLEAdvertising::isRunning(void)
{
  return _runnning;
}

bool BLEAdvertising::setBeacon(BLEBeacon& beacon)
{
  return beacon.start(*this);
}

bool BLEAdvertising::setBeacon(EddyStoneUrl& eddy_url)
{
  return eddy_url.start();
}

void BLEAdvertising::restartOnDisconnect(bool enable)
{
  _start_if_disconnect = enable;
}

bool BLEAdvertising::_start(uint16_t interval, uint16_t timeout)
{
  // ADV Params
  ble_gap_adv_params_t adv_para =
  {
    .properties    = { .type = _type, .anonymous  = 0 },
    .p_peer_addr   = NULL                     , // Undirected advertisement
    .interval      = interval                 , // advertising interval (in units of 0.625 ms)
    .duration      = (uint16_t) (timeout*100) , // in 10-ms unit

    .max_adv_evts  = 0                        , // TODO can be used for fast/slow mode
    .channel_mask  = { 0, 0, 0, 0, 0 }        , // 40 channel, set 1 to disable
    .filter_policy = BLE_GAP_ADV_FP_ANY       ,

    .primary_phy   = BLE_GAP_PHY_AUTO         , // 1 Mbps will be used
    .secondary_phy = BLE_GAP_PHY_AUTO         , // 1 Mbps will be used
      // , .set_id, .scan_req_notification
  };

  // gap_adv long-live is required by SD v6
  static ble_gap_adv_data_t gap_adv =
  {
      .adv_data      = { .p_data = _data, .len = _count },
      .scan_rsp_data = { .p_data = Bluefruit.ScanResponse.getData(), .len = Bluefruit.ScanResponse.count() }
  };
  VERIFY_STATUS( sd_ble_gap_adv_set_configure(&_hdl, &gap_adv, &adv_para), false );
  VERIFY_STATUS( sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, _hdl, Bluefruit.getTxPower() ), false );
  VERIFY_STATUS( sd_ble_gap_adv_start(_hdl, CONN_CFG_PERIPHERAL), false );

  Bluefruit._startConnLed(); // start blinking
  _runnning        = true;
  _active_interval = interval;

  _left_timeout -= min16(_left_timeout, timeout);

  return true;
}

bool BLEAdvertising::start(uint16_t timeout)
{
  _stop_timeout = _left_timeout = timeout;

  // Initially advertising in fast mode
  // Fast mode blink 2x than slow mode
  Bluefruit.setConnLedInterval(CFG_ADV_BLINKY_INTERVAL/2);
  VERIFY( _start(_fast_interval, _fast_timeout) );

  return true;
}

bool BLEAdvertising::stop(void)
{
  VERIFY_STATUS( sd_ble_gap_adv_stop(_hdl), false);

  _runnning = false;
  Bluefruit._stopConnLed(); // stop blinking

  return true;
}


void BLEAdvertising::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset for all events
  uint16_t const conn_hdl = evt->evt.common_evt.conn_handle;

  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;

      if ( para->role == BLE_GAP_ROLE_PERIPH )
      {
        bitSet(_conn_mask, conn_hdl);

        _runnning = false;
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      if ( bitRead(_conn_mask, conn_hdl) && (0 == Bluefruit.Periph.connected()) )
      {
        bitClear(_conn_mask, conn_hdl);

        // Auto start if enabled and not connected to any central
        if ( !_runnning && _start_if_disconnect ) start(_stop_timeout);
      }
    break;

    case BLE_GAP_EVT_ADV_SET_TERMINATED:
      if (evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
      {
        _runnning = false;

        // If still advertising, it is only in slow mode --> blink normal
        Bluefruit.setConnLedInterval(CFG_ADV_BLINKY_INTERVAL);

        if ( _stop_timeout == 0 )
        {
          // Call slow callback if available
          if (_slow_cb) _slow_cb();

          // if stop_timeout is 0 --> no timeout
          _start(_slow_interval, 0);
        }else
        {
          // Advertising if there is still time left, otherwise stop it
          if ( _left_timeout )
          {
            // Call slow callback if available
            if (_slow_cb) _slow_cb();

            _start(_slow_interval, _left_timeout);
          }else
          {
            // Stop advertising
            Bluefruit._stopConnLed(); // stop blinking

            // invoke stop callback
            if (_stop_cb) ada_callback(NULL, 0, _stop_cb);
          }
        }
      }
    break;

    default: break;
  }
}

