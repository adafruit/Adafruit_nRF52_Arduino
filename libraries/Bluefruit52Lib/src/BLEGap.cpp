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
  for(int i=0; i<BLE_GAP_MAX_CONN; i++) _txpacket_sem[i] = NULL;
}

bool BLEGap::getTxPacket(void)
{
  return getTxPacket( Bluefruit.connHandle() );
}

bool BLEGap::getTxPacket(uint16_t conn_handle)
{
  VERIFY( (conn_handle < BLE_GAP_MAX_CONN) && (_txpacket_sem[conn_handle] != NULL) );

  return xSemaphoreTake(_txpacket_sem[conn_handle], ms2tick(BLE_GENERIC_TIMEOUT));
}

void BLEGap::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t conn_handle = evt->evt.common_evt.conn_handle;

  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      // Init transmission buffer for notification
      uint8_t txbuf_max;
      (void) sd_ble_tx_packet_count_get(conn_handle, &txbuf_max);
      _txpacket_sem[conn_handle] = xSemaphoreCreateCounting(txbuf_max, txbuf_max);
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      vSemaphoreDelete( _txpacket_sem[conn_handle] );
      _txpacket_sem[conn_handle] = NULL;
    break;

    case BLE_EVT_TX_COMPLETE:
      if ( _txpacket_sem[conn_handle] )
      {
        for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
        {
          xSemaphoreGive(_txpacket_sem[conn_handle]);
        }
      }
    break;

    default: break;
  }
}
