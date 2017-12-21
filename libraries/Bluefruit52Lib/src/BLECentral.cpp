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

/**
 * Constructor
 */
BLECentral::BLECentral(void)
{
  _ppcp_min_conn = BLE_GAP_CONN_MIN_INTERVAL_DFLT;
  _ppcp_max_conn = BLE_GAP_CONN_MAX_INTERVAL_DFLT;

  _connect_cb    = NULL;
  _disconnect_cb = NULL;
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
  _ppcp_min_conn = min;
  _ppcp_max_conn = max;

  return true;
}

bool BLECentral::setConnIntervalMS (uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}

bool BLECentral::connect(const ble_gap_addr_t* peer_addr)
{
  ble_gap_conn_params_t gap_conn_params =
  {
      .min_conn_interval = _ppcp_min_conn, // in 1.25ms unit
      .max_conn_interval = _ppcp_max_conn, // in 1.25ms unit
      .slave_latency     = BLE_GAP_CONN_SLAVE_LATENCY,
      .conn_sup_timeout  = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  };

#if SD_VER < 500
  VERIFY_STATUS( sd_ble_gap_connect(peer_addr, Bluefruit.Scanner.getParams(), &gap_conn_params), false );
#else
  VERIFY_STATUS( sd_ble_gap_connect(peer_addr, Bluefruit.Scanner.getParams(), &gap_conn_params, CONN_CFG_CENTRAL), false );
#endif
  return true;
}

bool BLECentral::connect(const ble_gap_evt_adv_report_t* adv_report)
{
  return connect(&adv_report->peer_addr);
}

bool BLECentral::disconnect(uint16_t conn_handle)
{
  return ERROR_NONE == sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

/**
 * Check if connected to a specific peripheral
 * @param conn_handle
 * @return
 */
bool BLECentral::connected(uint16_t conn_handle)
{
  return Bluefruit.Gap.connected(conn_handle);
}

/**
 * Check if connected to ANY peripherals
 * @param conn_handle
 * @return
 */
bool BLECentral::connected(void)
{
  for (uint8_t conn=0; conn<BLE_MAX_CONN; conn++)
  {
    // skip Peripheral Role handle
    if ( Bluefruit.Gap.connected(conn) && (Bluefruit.Gap.getRole(conn) == BLE_GAP_ROLE_CENTRAL) )
    {
      return true;
    }
  }

  return false;
}

void BLECentral::setConnectCallback( BLEGap::connect_callback_t fp)
{
  _connect_cb = fp;
}

void BLECentral::setDisconnectCallback( BLEGap::disconnect_callback_t fp)
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

  /* PrPh handle connection is already filtered. Only handle Central events or
   * connection handle is BLE_CONN_HANDLE_INVALID (e.g BLE_GAP_EVT_ADV_REPORT)
   */
  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
      if ( Bluefruit.Gap.getRole(evt_conn_hdl) == BLE_GAP_ROLE_CENTRAL)
      {
        // Invoke callback
        if ( _connect_cb) ada_callback(NULL, _connect_cb, evt_conn_hdl);
      }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      if ( Bluefruit.Gap.getRole(evt_conn_hdl) == BLE_GAP_ROLE_CENTRAL)
      {
        // Invoke callback
        if ( _disconnect_cb) ada_callback(NULL, _disconnect_cb, evt_conn_hdl, evt->evt.gap_evt.params.disconnected.reason);
      }
    break;

    default: break;
  }
}
