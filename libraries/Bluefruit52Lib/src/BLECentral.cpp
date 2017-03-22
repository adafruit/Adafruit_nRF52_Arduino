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

#define BLE_CENTRAL_TIMEOUT   10000

/**
 * Constructor
 */
BLECentral::BLECentral(void)
{
  _conn_hdl    = BLE_CONN_HANDLE_INVALID;

  _evt_sem     = NULL;
  _evt_buf     = NULL;
  _evt_bufsize = 0;

  _scan_cb     = NULL;
  _scan_param  = (ble_gap_scan_params_t) {
    .active      = 1,
    .selective   = 0,
    .p_whitelist = NULL,
    .interval    = 0x00A0,
    .window      = 0x0050,
    .timeout     = 0, // no timeout
  };

  _disc_hdl_range.start_handle = 1;
  _disc_hdl_range.end_handle   = 0xffff;

  _connect_cb     = NULL;
  _disconnect_cb = NULL;
}

void BLECentral::begin(void)
{
  _evt_sem = xSemaphoreCreateBinary();
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

void BLECentral::setConnectCallback( connect_callback_t fp)
{
  _connect_cb = fp;
}

void BLECentral::setDisconnectCallback( disconnect_callback_t fp)
{
  _disconnect_cb = fp;
}

/*------------------------------------------------------------------*/
/* DISCOVERY
 *------------------------------------------------------------------*/
bool BLECentral::discoverService(BLEUuid uuid, uint16_t start_handle)
{
  uuid.begin(); // add uuid128 if needed

  ble_gattc_evt_prim_srvc_disc_rsp_t disc_svc;

  _evt_buf     = &disc_svc;
  _evt_bufsize = sizeof(disc_svc);

  VERIFY_STATUS( sd_ble_gattc_primary_services_discover(_conn_hdl, start_handle, &uuid._uuid), false );

  // wait for discovery event: timeout or has no data
  if ( !xSemaphoreTake(_evt_sem, BLE_CENTRAL_TIMEOUT) || (_evt_bufsize == 0) ) return false;

  // Check the discovered UUID with input one
  if ( (disc_svc.count == 1) && (uuid == disc_svc.services[0].uuid) )
  {
    _disc_hdl_range = disc_svc.services[0].handle_range;
    LOG_LV1(Discover, "Handle start = %d, end = %d", _disc_hdl_range.start_handle, _disc_hdl_range.end_handle);
    return true;
  }

  return false;
}

bool BLECentral::discoverService(BLECentralService& svc, uint16_t start_handle)
{
  return svc.discover(start_handle);
}

bool BLECentral::discoverCharacteristic(BLECentralCharacteristic& chr)
{
  ble_gattc_evt_char_disc_rsp_t disc_chr;

  _evt_buf     = &disc_chr;
  _evt_bufsize = sizeof(disc_chr);

  LOG_LV1(Discover, "Handle start = %d, end = %d", _disc_hdl_range.start_handle, _disc_hdl_range.end_handle);

  VERIFY_STATUS( sd_ble_gattc_characteristics_discover(_conn_hdl, &_disc_hdl_range), false );

  // wait for discovery event: timeout or has no data
  if ( !xSemaphoreTake(_evt_sem, BLE_CENTRAL_TIMEOUT) || (_evt_bufsize == 0) ) return false;

  if ( disc_chr.count > 0 )
  {
    chr = BLECentralCharacteristic(&disc_chr.chars[0]);

    // increase handle range for next characteristic
    _disc_hdl_range.start_handle = disc_chr.chars[0].handle_value + 1;
  }

  return true;
}

/**
 * Event is forwarded from Bluefruit Poll() method
 * @param event
 */
void BLECentral::_event_handler(ble_evt_t* evt)
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

        _conn_hdl = evt->evt.gap_evt.conn_handle;

        if ( _connect_cb ) _connect_cb();
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      if ( _conn_hdl == evt->evt.gap_evt.conn_handle)
      {
        if (Bluefruit._led_conn)  ledOff(LED_BLUE);

        _conn_hdl = BLE_CONN_HANDLE_INVALID;

        if ( _disconnect_cb ) _disconnect_cb(evt->evt.gap_evt.params.disconnected.reason);

        startScanning();
      }
    break;

    case BLE_GAP_EVT_TIMEOUT:
      if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
      {
        // Restart Scanning
        startScanning();
      }
    break;

    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
    {
      ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;

      if ( _conn_hdl == gattc->conn_handle )
      {
        if ( _evt_buf )
        {
          if (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS)
          {
            ble_gattc_evt_prim_srvc_disc_rsp_t* svc_rsp = &gattc->params.prim_srvc_disc_rsp;

            LOG_LV1(Discover, "Service Count: %d", svc_rsp->count);

            COMMENT_OUT (
              uint16_t len = sizeof(ble_gattc_evt_prim_srvc_disc_rsp_t) + (svc_rsp->count-1)*sizeof(ble_gattc_service_t);
              _evt_bufsize = min16(_evt_bufsize, len);
            )

            // Only support 1 service
            _evt_bufsize = min16(_evt_bufsize, sizeof(ble_gattc_evt_prim_srvc_disc_rsp_t));
            memcpy(_evt_buf, svc_rsp, _evt_bufsize);
          }else
          {
            _evt_bufsize = 0; // no data
          }
        }

        xSemaphoreGive(_evt_sem);
      }
    }
    break;

    case BLE_GATTC_EVT_CHAR_DISC_RSP:
    {
      ble_gattc_evt_t* gattc = &evt->evt.gattc_evt;
      ble_gattc_evt_char_disc_rsp_t* chr_rsp = &gattc->params.char_disc_rsp;

      if ( _conn_hdl == gattc->conn_handle )
      {
        LOG_LV1(Discover, "Characteristic Count: %d", chr_rsp->count);
        if ( (gattc->gatt_status == BLE_GATT_STATUS_SUCCESS) && (chr_rsp->count > 0) )
        {
          // TODO support only 1 discovered char now
          _evt_bufsize = min16(_evt_bufsize, sizeof(ble_gattc_evt_char_disc_rsp_t));

          memcpy(_evt_buf, chr_rsp, _evt_bufsize);
        }else
        {
          _evt_bufsize = 0; // no data
        }
      }

      xSemaphoreGive(_evt_sem);
    }
    break;

    default: break;
  }
}
