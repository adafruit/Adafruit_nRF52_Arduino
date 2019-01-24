/**************************************************************************/
/*!
    @file     BLEGap.cpp
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

BLEGap::BLEGap(void)
{
  _prph.mtu_max          = BLE_GATT_ATT_MTU_DEFAULT;
  _prph.event_len        = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _prph.hvn_qsize        = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _prph.wrcmd_qsize      = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;
  _prph.connect_cb       = NULL;
  _prph.disconnect_cb    = NULL;

  _central.mtu_max       = BLE_GATT_ATT_MTU_DEFAULT;
  _central.event_len     = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _central.hvn_qsize     = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _central.wrcmd_qsize   = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;
  _central.connect_cb    = NULL;
  _central.disconnect_cb = NULL;

  memclr(_connection, sizeof(_connection));

  _rssi_cb = NULL;

  _sec_param = (ble_gap_sec_params_t)
                {
                  .bond         = 1,
                  .mitm         = 0,
                  .lesc         = 0,
                  .keypress     = 0,
                  .io_caps      = BLE_GAP_IO_CAPS_NONE,
                  .oob          = 0,
                  .min_key_size = 7,
                  .max_key_size = 16,
                  .kdist_own    = { .enc = 1, .id = 1},
                  .kdist_peer   = { .enc = 1, .id = 1},
                };
}


void BLEGap::configPrphConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _prph.mtu_max      = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
  _prph.event_len    = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _prph.hvn_qsize = hvn_qsize;
  _prph.wrcmd_qsize = wrcmd_qsize;
}

void BLEGap::configCentralConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _central.mtu_max      = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
  _central.event_len    = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _central.hvn_qsize = hvn_qsize;
  _central.wrcmd_qsize = wrcmd_qsize;
}

void BLEGap::_prph_setConnectCallback( connect_callback_t fp)
{
  _prph.connect_cb = fp;
}

void BLEGap::_prph_setDisconnectCallback( disconnect_callback_t fp)
{
  _prph.disconnect_cb = fp;
}

void BLEGap::_central_setConnectCallback( connect_callback_t fp)
{
  _central.connect_cb = fp;
}

void BLEGap::_central_setDisconnectCallback( disconnect_callback_t fp)
{
  _central.disconnect_cb = fp;
}


uint16_t BLEGap::getMaxMtu(uint8_t role)
{
  return (role == BLE_GAP_ROLE_PERIPH) ? _prph.mtu_max : _central.mtu_max;
}

BLEConnection* BLEGap::Connection(uint16_t conn_hdl)
{
  return (conn_hdl < BLE_MAX_CONNECTION) ?_connection[conn_hdl] : NULL;
}

void BLEGap::setRssiCallback(rssi_callback_t fp)
{
  _rssi_cb = fp;
}

/**
 * Event handler
 */
void BLEGap::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t conn_hdl = evt->evt.common_evt.conn_handle;
  BLEConnection* conn = this->Connection(conn_hdl);

  // Connection handler
  if ( conn ) conn->_eventHandler(evt);

  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      // Turn on Conn LED
      Bluefruit._stopConnLed();
      Bluefruit._setConnLed(true);

      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;

      if ( _connection[conn_hdl] )
      {
        LOG_LV1("GAP", "Connection is already in used, something wrong !!");
        delete _connection[conn_hdl];
        _connection[conn_hdl] = NULL;
      }

      // Transmission buffer pool
      uint8_t const hvn_qsize = (para->role == BLE_GAP_ROLE_PERIPH) ? _prph.hvn_qsize : _central.hvn_qsize;
      uint8_t const wrcmd_qsize = (para->role == BLE_GAP_ROLE_PERIPH) ? _prph.wrcmd_qsize : _central.wrcmd_qsize;

      _connection[conn_hdl] = new BLEConnection(conn_hdl, para, hvn_qsize, wrcmd_qsize);
      conn = _connection[conn_hdl];

      LOG_LV2("GAP", "Conn Interval= %f", para->conn_params.min_conn_interval*1.25f);

      // Invoke connect callback
      if ( conn->getRole() == BLE_GAP_ROLE_PERIPH )
      {
        if ( _prph.connect_cb ) ada_callback(NULL, _prph.connect_cb, conn_hdl);
      }else
      {
        if ( _central.connect_cb ) ada_callback(NULL, _central.connect_cb, conn_hdl);
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
      ble_gap_evt_disconnected_t const* para = &evt->evt.gap_evt.params.disconnected;

      LOG_LV2("GAP", "Disconnect Reason 0x%02X", evt->evt.gap_evt.params.disconnected.reason);

      delete _connection[conn_hdl];
      _connection[conn_hdl] = NULL;

      // Turn off Conn LED If not connected at all
      bool still_connected = false;
      for (uint8_t i=0; i<BLE_MAX_CONNECTION; i++)
      {
        if ( _connection[i] && _connection[i]->connected() )
        {
          still_connected = true;
          break;
        }
      }
      if ( !still_connected ) Bluefruit._setConnLed(false);

      // Invoke disconnect callback
      if ( conn->getRole() == BLE_GAP_ROLE_PERIPH )
      {
        if ( _prph.disconnect_cb ) ada_callback(NULL, _prph.disconnect_cb, conn_hdl, para->reason);
      }else
      {
        if ( _central.disconnect_cb ) ada_callback(NULL, _central.disconnect_cb, conn_hdl, para->reason);
      }
    }
    break;

    case BLE_GAP_EVT_RSSI_CHANGED:
    {
      ble_gap_evt_rssi_changed_t const * rssi_changed = &evt->evt.gap_evt.params.rssi_changed;
      if ( _rssi_cb )
      {
         ada_callback(NULL, _rssi_cb, conn_hdl, rssi_changed->rssi);
      }
    }
    break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
      ble_gap_data_length_params_t* param = &evt->evt.gap_evt.params.data_length_update_request.peer_params;
      LOG_LV2("GAP", "Data Length Req is (tx, rx) octets = (%d, %d), (tx, rx) time = (%d, %d) us",
              param->max_tx_octets, param->max_rx_octets, param->max_tx_time_us, param->max_rx_time_us);

      // Let Softdevice decide the data length
      VERIFY_STATUS( sd_ble_gap_data_length_update(conn_hdl, NULL, NULL), );
    }
    break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
    {
      ble_gap_data_length_params_t* datalen =  &evt->evt.gap_evt.params.data_length_update.effective_params;
      LOG_LV2("GAP", "Data Length is (tx, rx) octets = (%d, %d), (tx, rx) time = (%d, %d) us",
                   datalen->max_tx_octets, datalen->max_rx_octets, datalen->max_tx_time_us, datalen->max_rx_time_us);
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      ble_gap_phys_t* req_phy = &evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys;

      #if CFG_DEBUG >= 1
      char const *phy_str[] = { "Auto", "1 Mbps", "2 Mbps", "Coded" };
      LOG_LV1("GAP", "PHY request tx: %s, rx: %s", phy_str[req_phy->tx_phys], phy_str[req_phy->rx_phys]);
      #endif

      // Tell SoftDevice to choose PHY automatically
      ble_gap_phys_t phy = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
      (void) sd_ble_gap_phy_update(conn_hdl, &phy);
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE:
    {
      ble_gap_evt_phy_update_t* active_phy = &evt->evt.gap_evt.params.phy_update;

      #if CFG_DEBUG >= 1
      if ( active_phy->status != BLE_HCI_STATUS_CODE_SUCCESS )
      {
        LOG_LV1("GAP", "Failed HCI status = 0x%02X", active_phy->status);
      }else
      {
        char const *phy_str[] = { "Auto", "1 Mbps", "2 Mbps", "Coded" };
        LOG_LV1("GAP", "PHY active tx: %s, rx: %s", phy_str[active_phy->tx_phy], phy_str[active_phy->rx_phy]);
      }
      #endif
    }
    break;

    default: break;
  }
}
