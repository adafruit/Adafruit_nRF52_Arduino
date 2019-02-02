/**************************************************************************/
/*!
    @file     BLECentral.cpp
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
#include "utility/bonding.h"

/**
 * Constructor
 */
BLECentral::BLECentral(void)
{
  _connect_cb = NULL;
  _disconnect_cb = NULL;

  _conn_param.min_conn_interval = _conn_param.max_conn_interval = BLE_GAP_CONN_MIN_INTERVAL_DFLT;
  _conn_param.slave_latency = BLE_GAP_CONN_SLAVE_LATENCY;
  _conn_param.conn_sup_timeout = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS/10;
}

void BLECentral::begin(void)
{
  // Central will very likely use Discovery
  Bluefruit.Discovery.begin();
}

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
bool BLECentral::setConnInterval(uint16_t min, uint16_t max)
{
  _conn_param.min_conn_interval = min;
  _conn_param.max_conn_interval = max;

  return true;
}

bool BLECentral::setConnIntervalMS (uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}

bool BLECentral::connect(const ble_gap_addr_t* peer_addr)
{
  // Connect with default connection parameter
  VERIFY_STATUS( sd_ble_gap_connect(peer_addr, Bluefruit.Scanner.getParams(), &_conn_param, CONN_CFG_CENTRAL), false );

  return true;
}

bool BLECentral::connect(const ble_gap_evt_adv_report_t* adv_report)
{
  return connect(&adv_report->peer_addr);
}

/**
 * Check if connected to a specific peripheral
 * @param conn_handle
 * @return
 */
bool BLECentral::connected(uint16_t conn_hdl)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  return conn && conn->connected() && (conn->getRole() == BLE_GAP_ROLE_CENTRAL);
}

/**
 * @return number of peripherals
 */
uint8_t BLECentral::connected(void)
{
  uint8_t count = 0;
  for (uint16_t c=0; c<BLE_MAX_CONNECTION; c++)
  {
    if ( this->connected(c) ) count++;
  }

  return count;
}

void BLECentral::setConnectCallback( ble_connect_callback_t fp )
{
  _connect_cb = fp;
}

void BLECentral::setDisconnectCallback( ble_disconnect_callback_t fp )
{
  _disconnect_cb = fp;
}

void BLECentral::clearBonds(void)
{
  bond_clear_cntr();
}

/**
 * Event is forwarded from Bluefruit Poll() method
 * @param event
 */
void BLECentral::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t conn_hdl = evt->evt.common_evt.conn_handle;

  /* PrPh handle connection is already filtered. Only handle Central events or
   * connection handle is BLE_CONN_HANDLE_INVALID (e.g BLE_GAP_EVT_ADV_REPORT) */
  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    {
      // Peripheral request to change connection parameter
      ble_gap_conn_params_t* request_param = &evt->evt.gap_evt.params.conn_param_update_request.conn_params;

      LOG_LV2("GAP", "Conn Param Update Request: (min, max, latency, sup) = (%.2f,  %.2f, %d, %d)",
              request_param->min_conn_interval*1.25f, request_param->max_conn_interval*1.25f, request_param->slave_latency, request_param->conn_sup_timeout*10);

      // Central could perform checks to accept or reject request
      // For now just accept parameter from prph
      ble_gap_conn_params_t conn_param = *request_param;
      conn_param.max_conn_interval = conn_param.min_conn_interval;

      sd_ble_gap_conn_param_update(conn_hdl, &conn_param);
    }
    break;

    default: break;
  }
}
