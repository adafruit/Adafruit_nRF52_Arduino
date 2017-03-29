/**************************************************************************/
/*!
    @file     BLECentral.cpp
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

#define BLE_CENTRAL_TIMEOUT     3000

/**
 * Constructor
 */
BLECentral::BLECentral(void)
{
  _conn_hdl    = BLE_CONN_HANDLE_INVALID;

  _txpacket_sem = NULL;

  _svc_count = 0;
  for(uint8_t i=0; i<BLE_CENTRAL_MAX_SERVICE; i++) _svc_list[i] = NULL;

  _chars_count = 0;
  for(uint8_t i=0; i<BLE_CENTRAL_MAX_CHARS; i++) _chars_list[i] = NULL;

  _scan_cb     = NULL;
  _scan_param  = (ble_gap_scan_params_t) {
    .active      = 1,
    .selective   = 0,
    .p_whitelist = NULL,
    .interval    = 0x00A0,
    .window      = 0x0050,
    .timeout     = 0, // no timeout
  };


  _connect_cb     = NULL;
  _disconnect_cb = NULL;
}

void BLECentral::begin(void)
{

}

bool BLECentral::_registerService(BLECentralService* svc)
{
  VERIFY( _svc_count < BLE_CENTRAL_MAX_SERVICE );
  _svc_list[ _svc_count++ ] = svc;

  return true;
}

bool BLECentral::_registerCharacteristic(BLECentralCharacteristic* chr)
{
  VERIFY( _chars_count < BLE_CENTRAL_MAX_CHARS );
  _chars_list[ _chars_count++ ] = chr;

  return true;
}

bool BLECentral::getTxPacket(uint32_t ms)
{
  VERIFY(_txpacket_sem != NULL);
  return xSemaphoreTake(_txpacket_sem, ms2tick(ms));
}

/*------------------------------------------------------------------*/
/* Scan  and Parser
 *------------------------------------------------------------------*/
void BLECentral::setScanCallback(scan_callback_t fp)
{
  _scan_cb = fp;
}

err_t BLECentral::startScanning(uint16_t timeout)
{
  _scan_param.timeout = timeout;
  VERIFY_STATUS( sd_ble_gap_scan_start(&_scan_param) );
  Bluefruit.startConnLed(); // start blinking
  return ERROR_NONE;
}

err_t BLECentral::stopScanning(void)
{
  Bluefruit.stopConnLed(); // stop blinking
  return sd_ble_gap_scan_stop();
}

uint8_t* BLECentral::extractScanData(uint8_t const* scandata, uint8_t scanlen, uint8_t type, uint8_t* result_len)
{
  *result_len = 0;

  // len (1+data), type, data
  while ( scanlen )
  {
    if ( scandata[1] == type )
    {
      *result_len = scandata[0]-1;
      return (uint8_t*) (scandata + 2);
    }
    else
    {
      scanlen  -= (scandata[0] + 1);
      scandata += (scandata[0] + 1);
    }
  }

  return NULL;
}

uint8_t* BLECentral::extractScanData(const ble_gap_evt_adv_report_t* report, uint8_t type, uint8_t* result_len)
{
  return extractScanData(report->data, report->dlen, type, result_len);
}

bool BLECentral::checkUuidInScan(const ble_gap_evt_adv_report_t* report, BLEUuid ble_uuid)
{
  const uint8_t* uuid;
  uint8_t uuid_len = ble_uuid.size();

  uint8_t type_arr[2];

  // Check both UUID16 more available and complete list
  if ( uuid_len == 16)
  {
    type_arr[0] = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE;
    type_arr[1] = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE;

    uuid = (uint8_t*) &ble_uuid._uuid.uuid;
  }else
  {
    type_arr[0] = BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE;
    type_arr[1] = BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE;

    uuid = ble_uuid._uuid128;
  }

  uuid_len /= 8; // convert uuid_len to number of bytes

  for (int i=0; i<2; i++)
  {
    uint8_t len = 0;
    uint8_t const* data = extractScanData(report, type_arr[i] , &len);

    while( len )
    {
      // found matched
      if ( !memcmp(data, uuid, uuid_len) )
      {
        return true;
      }else
      {
        data += uuid_len;
        len  -= uuid_len;
      }
    }
  }

  return false;
}

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
err_t BLECentral::connect(const ble_gap_addr_t* peer_addr, uint16_t min_conn_interval, uint16_t max_conn_interval)
{
  ble_gap_conn_params_t gap_conn_params =
  {
      .min_conn_interval = min_conn_interval, // in 1.25ms unit
      .max_conn_interval = max_conn_interval, // in 1.25ms unit
      .slave_latency     = BLE_GAP_CONN_SLAVE_LATENCY,
      .conn_sup_timeout  = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  };

  return sd_ble_gap_connect(peer_addr, &_scan_param, &gap_conn_params);
}

err_t BLECentral::connect(const ble_gap_evt_adv_report_t* adv_report, uint16_t min_conn_interval, uint16_t max_conn_interval)
{
  return connect(&adv_report->peer_addr, min_conn_interval, max_conn_interval);
}

bool BLECentral::connected(void)
{
  return (_conn_hdl != BLE_CONN_HANDLE_INVALID);
}

uint16_t BLECentral::connHandle (void)
{
  return _conn_hdl;
}

void BLECentral::setConnectCallback( connect_callback_t fp)
{
  _connect_cb = fp;
}

void BLECentral::setDisconnectCallback( disconnect_callback_t fp)
{
  _disconnect_cb = fp;
}


/**
 * Event is forwarded from Bluefruit Poll() method
 * @param event
 */
void BLECentral::_event_handler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t evt_conn_hdl = evt->evt.common_evt.conn_handle;

  /* Only handle Central events with matched connection handle
   * or a few special one
   * - Connected event
   * - Advertising Report
   * - Advertising timeout (could be connected and advertising at the same time)
   */
  if ( evt_conn_hdl       == _conn_hdl              ||
       evt->header.evt_id == BLE_GAP_EVT_CONNECTED  ||
       evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT ||
       evt->header.evt_id == BLE_GAP_EVT_TIMEOUT )
  {
    switch ( evt->header.evt_id  )
    {
      case BLE_GAP_EVT_ADV_REPORT:
      {
        ble_gap_evt_adv_report_t* adv_report = &evt->evt.gap_evt.params.adv_report;
        if (_scan_cb) _scan_cb(adv_report);
      }
      break;

      case BLE_GAP_EVT_CONNECTED:
      {
        ble_gap_evt_connected_t* para = &evt->evt.gap_evt.params.connected;

        if (para->role == BLE_GAP_ROLE_CENTRAL)
        {
          Bluefruit.stopConnLed();
          if (Bluefruit._led_conn) ledOn(LED_BLUE);

          // TODO multiple connections
          _conn_hdl = evt->evt.gap_evt.conn_handle;

          // Init transmission buffer for notification. TODO multiple connections
          uint8_t txbuf_max;
          (void) sd_ble_tx_packet_count_get(_conn_hdl, &txbuf_max);
          _txpacket_sem = xSemaphoreCreateCounting(txbuf_max, txbuf_max);

          if ( _connect_cb ) _connect_cb();
        }
      }
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        if (Bluefruit._led_conn)  ledOff(LED_BLUE);

        _conn_hdl = BLE_CONN_HANDLE_INVALID;

        // TODO multiple connections
        vSemaphoreDelete(_txpacket_sem);
        _txpacket_sem = NULL;

        // disconnect all registered services
        for(uint8_t i=0; i<_svc_count; i++) _svc_list[i]->disconnect();

        if ( _disconnect_cb ) _disconnect_cb(evt->evt.gap_evt.params.disconnected.reason);

        startScanning();
      break;

      case BLE_EVT_TX_COMPLETE:
        if ( _txpacket_sem )
        {
          for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
          {
            xSemaphoreGive(_txpacket_sem);
          }
        }
      break;

      case BLE_GAP_EVT_TIMEOUT:
        if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
        {
          // TODO Advance Scanning
          // Restart Scanning
          startScanning();
        }
      break;

      default: break;
    }
  }

  // GATT characteristics event handler only if Connection Handle matches
  if ( evt_conn_hdl == _conn_hdl)
  {
    for(int i=0; i<_chars_count; i++)
    {
      bool matched = false;

      switch(evt->header.evt_id)
      {
        case BLE_GATTC_EVT_HVX:
        case BLE_GATTC_EVT_WRITE_RSP:
        case BLE_GATTC_EVT_READ_RSP:
          // write & read response's handle has same offset.
          matched = (_chars_list[i]->_chr.handle_value == evt->evt.gattc_evt.params.write_rsp.handle);
        break;

        default: break;
      }

      // invoke charactersistic handler if matched
      if ( matched ) _chars_list[i]->_eventHandler(evt);
    }
  }
}
