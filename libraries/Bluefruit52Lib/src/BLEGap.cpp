/**************************************************************************/
/*!
    @file     BLEGap.cpp
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

BLEGap::BLEGap(void)
{
  memclr(_peers, sizeof(_peers));

  _cfg_prph.mtu_max         = BLE_GATT_ATT_MTU_DEFAULT;
  _cfg_central.mtu_max      = BLE_GATT_ATT_MTU_DEFAULT;

  _cfg_prph.event_len       = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _cfg_prph.hvn_tx_qsize    = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _cfg_prph.wr_cmd_qsize    = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;

  _cfg_central.event_len    = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _cfg_central.hvn_tx_qsize = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _cfg_central.wr_cmd_qsize = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;
}


void BLEGap::configPrphConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _cfg_prph.mtu_max      = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
  _cfg_prph.event_len    = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _cfg_prph.hvn_tx_qsize = hvn_qsize;
  _cfg_prph.wr_cmd_qsize = wrcmd_qsize;
}

void BLEGap::configCentralConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _cfg_central.mtu_max      = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
  _cfg_central.event_len    = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _cfg_central.hvn_tx_qsize = hvn_qsize;
  _cfg_central.wr_cmd_qsize = wrcmd_qsize;
}


uint16_t BLEGap::getMaxMtuByConnCfg(uint8_t conn_cfg)
{
  return (conn_cfg == CONN_CFG_PERIPHERAL) ? _cfg_prph.mtu_max : _cfg_central.mtu_max;
}

uint16_t BLEGap::getMaxMtu (uint8_t conn_handle)
{
  return (getRole(conn_handle) == BLE_GAP_ROLE_PERIPH) ? _cfg_prph.mtu_max : _cfg_central.mtu_max;
}

uint8_t BLEGap::getHvnQueueSize (uint8_t conn_handle)
{
  return (getRole(conn_handle) == BLE_GAP_ROLE_PERIPH) ? _cfg_prph.hvn_tx_qsize : _cfg_central.hvn_tx_qsize;
}

uint8_t BLEGap::getWriteCmdQueueSize (uint8_t conn_handle)
{
  return (getRole(conn_handle) == BLE_GAP_ROLE_PERIPH) ? _cfg_prph.wr_cmd_qsize : _cfg_central.wr_cmd_qsize;
}


/**
 * Get current Mac address and its type
 * @param mac address
 * @return Address type e.g BLE_GAP_ADDR_TYPE_RANDOM_STATIC
 */
uint8_t BLEGap::getAddr(uint8_t mac[6])
{
  ble_gap_addr_t addr;

  sd_ble_gap_addr_get(&addr);
  memcpy(mac, addr.addr, 6);

  return addr.addr_type;
}

/**
 * Set the MAC address
 * @param mac   Bluetooth MAC Address
 * @param type  Must be either BLE_GAP_ADDR_TYPE_PUBLIC or BLE_GAP_ADDR_TYPE_RANDOM_STATIC
 * @return true if success
 */
bool BLEGap::setAddr(uint8_t mac[6], uint8_t type)
{
  ble_gap_addr_t addr;
  addr.addr_type = type;

  VERIFY (type == BLE_GAP_ADDR_TYPE_PUBLIC || type == BLE_GAP_ADDR_TYPE_RANDOM_STATIC);

  memcpy(addr.addr, mac, 6);

#if SD_VER < 500
  VERIFY_STATUS( sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr), false );
#else
  VERIFY_STATUS( sd_ble_gap_addr_set(&addr), false );
#endif

  return true;
}

bool BLEGap::connected(uint16_t conn_handle)
{
  return _peers[conn_handle].connected;
}

uint8_t BLEGap::getRole(uint16_t conn_handle)
{
  return _peers[conn_handle].role;
}

uint8_t BLEGap::getPeerAddr(uint16_t conn_handle, uint8_t addr[6])
{
  memcpy(addr, _peers[conn_handle].addr.addr, BLE_GAP_ADDR_LEN);
  return _peers[conn_handle].addr.addr_type;
}

ble_gap_addr_t BLEGap::getPeerAddr(uint16_t conn_handle)
{
  return _peers[conn_handle].addr;
}

bool BLEGap::getHvnPacket(uint16_t conn_handle)
{
  VERIFY( (conn_handle < BLE_MAX_CONN) && (_peers[conn_handle].hvn_tx_sem != NULL) );

  return xSemaphoreTake(_peers[conn_handle].hvn_tx_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

bool BLEGap::getWriteCmdPacket(uint16_t conn_handle)
{
  VERIFY( (conn_handle < BLE_MAX_CONN) && (_peers[conn_handle].wrcmd_tx_sem != NULL) );
  return xSemaphoreTake(_peers[conn_handle].wrcmd_tx_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

uint16_t BLEGap::getMTU (uint16_t conn_handle)
{
  return _peers[conn_handle].att_mtu;
}

uint16_t BLEGap::getPeerName(uint16_t conn_handle, char* buf, uint16_t bufsize)
{
  return Bluefruit.Gatt.readCharByUuid(conn_handle, BLEUuid(BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME), buf, bufsize);
}

/**
 * Event handler
 * @param evt
 */
void BLEGap::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t conn_handle = evt->evt.common_evt.conn_handle;

  gap_peer_t* peer = &_peers[conn_handle];

  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;

      peer->connected = true;
      peer->role      = para->role;
      peer->addr      = para->peer_addr;
      peer->att_mtu   = BLE_GATT_ATT_MTU_DEFAULT;

      // Init transmission buffer for notification
      peer->hvn_tx_sem   = xSemaphoreCreateCounting(getHvnQueueSize(conn_handle), getHvnQueueSize(conn_handle));
      peer->wrcmd_tx_sem = xSemaphoreCreateCounting(getWriteCmdQueueSize(conn_handle), getWriteCmdQueueSize(conn_handle));
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
      ble_gap_evt_disconnected_t const* para = &evt->evt.gap_evt.params.disconnected;

      // mark as disconnected, but keep the role for sub sequence event handler
      peer->connected = false;

      vSemaphoreDelete( peer->hvn_tx_sem );
      peer->hvn_tx_sem = NULL;

      vSemaphoreDelete( peer->wrcmd_tx_sem );
      peer->wrcmd_tx_sem = NULL;
    }
    break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      if ( peer->hvn_tx_sem )
      {
        for(uint8_t i=0; i<evt->evt.gatts_evt.params.hvn_tx_complete.count; i++)
        {
          xSemaphoreGive(peer->hvn_tx_sem);
        }
      }
    break;

    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      if ( peer->wrcmd_tx_sem )
      {
        for(uint8_t i=0; i<evt->evt.gattc_evt.params.write_cmd_tx_complete.count; i++)
        {
          xSemaphoreGive(peer->wrcmd_tx_sem);
        }
      }
    break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
      ble_gap_data_length_params_t* param = &evt->evt.gap_evt.params.data_length_update_request.peer_params;
      LOG_LV2("GAP", "Data Length Req is (tx, rx) octets = (%d, %d), (tx, rx) time = (%d, %d) us",
              param->max_tx_octets, param->max_rx_octets, param->max_tx_time_us, param->max_rx_time_us);

      // Let Softdevice decide the data length
      VERIFY_STATUS( sd_ble_gap_data_length_update(conn_handle, NULL, NULL), );
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
      (void) sd_ble_gap_phy_update(conn_handle, &phy);
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

    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
    {
      peer->att_mtu = minof(evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu, getMaxMtu(conn_handle));
      VERIFY_STATUS( sd_ble_gatts_exchange_mtu_reply(conn_handle, peer->att_mtu), );

      LOG_LV1("GAP", "ATT MTU is changed to %d", peer->att_mtu);
    }
    break;

    default: break;
  }
}
