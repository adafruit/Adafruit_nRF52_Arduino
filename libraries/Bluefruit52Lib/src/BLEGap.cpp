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
#include "utility/AdaCallback.h"

BLEGap::BLEGap(void)
{
  varclr(&_peers);
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
      uint8_t txbuf_max;
      (void) sd_ble_tx_packet_count_get(conn_handle, &txbuf_max);
      peer->txpacket_sem = xSemaphoreCreateCounting(txbuf_max, txbuf_max);
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

    case BLE_EVT_TX_COMPLETE:
      if ( peer->txpacket_sem )
      {
        for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
        {
          xSemaphoreGive(peer->txpacket_sem);
        }
      }
    break;

    default: break;
  }
}
