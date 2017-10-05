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
  varclr(&_peers);
}

/**
 * Get current Mac address and its type
 * @param mac address
 * @return Address type e.g BLE_GAP_ADDR_TYPE_RANDOM_STATIC
 */
uint8_t BLEGap::getAddr(uint8_t mac[6])
{
  ble_gap_addr_t addr;

#if SD_VER < 500
  sd_ble_gap_address_get(&addr);
#else
  sd_ble_gap_addr_get(&addr);
#endif

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

bool BLEGap::getTxPacket(uint16_t conn_handle)
{
  VERIFY( (conn_handle < BLE_MAX_CONN) && (_peers[conn_handle].txpacket_sem != NULL) );

  return xSemaphoreTake(_peers[conn_handle].txpacket_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

uint16_t BLEGap::getMTU (uint16_t conn_handle)
{
  return BLE_GATT_ATT_MTU_DEFAULT;
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

      // Init transmission buffer for notification
      #if SD_VER < 500
      uint8_t txbuf_max;
      (void) sd_ble_tx_packet_count_get(conn_handle, &txbuf_max);
      peer->txpacket_sem = xSemaphoreCreateCounting(txbuf_max, txbuf_max);
      #else
      peer->txpacket_sem = xSemaphoreCreateCounting(BLEGAP_HVN_TX_QUEUE_SIZE, BLEGAP_HVN_TX_QUEUE_SIZE);
      // TODO BLEGAP_WRITECMD_TX_QUEUE_SIZE
      #endif
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
      ble_gap_evt_disconnected_t const* para = &evt->evt.gap_evt.params.disconnected;

      // mark as disconnected, but keep the role for sub sequence event handler
      peer->connected = false;

      vSemaphoreDelete( peer->txpacket_sem );
      peer->txpacket_sem = NULL;
    }
    break;

    #if SD_VER < 500
    case BLE_EVT_TX_COMPLETE:
      if ( peer->txpacket_sem )
      {
        for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
        {
          xSemaphoreGive(peer->txpacket_sem);
        }
      }
    break;

    #else
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      if ( peer->txpacket_sem )
      {
        for(uint8_t i=0; i<evt->evt.gatts_evt.params.hvn_tx_complete.count; i++)
        {
          xSemaphoreGive(peer->txpacket_sem);
        }
      }
    break;

    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      if ( peer->txpacket_sem )
      {
        for(uint8_t i=0; i<evt->evt.gattc_evt.params.write_cmd_tx_complete.count; i++)
        {
          // TODO BLEGAP_WRITECMD_TX_QUEUE_SIZE
          xSemaphoreGive(peer->txpacket_sem);
        }
      }
    break;
    #endif

    default: break;
  }
}
