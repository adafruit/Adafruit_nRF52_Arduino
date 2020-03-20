/**************************************************************************/
/*!
    @file     BLEPeriph.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019, Adafruit Industries (adafruit.com)
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

// Constructor
BLEPeriph::BLEPeriph(void)
{
  _connect_cb = NULL;
  _disconnect_cb = NULL;

  _ppcp = ((ble_gap_conn_params_t) {
    .min_conn_interval = BLE_GAP_CONN_MIN_INTERVAL_DFLT,
    .max_conn_interval = BLE_GAP_CONN_MAX_INTERVAL_DFLT,
    .slave_latency     = 0,
    .conn_sup_timeout  = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  });

}

bool BLEPeriph::begin(void)
{
  // Peripheral Preferred Connection Parameters
  VERIFY_STATUS(sd_ble_gap_ppcp_set(&_ppcp), false);

  VERIFY_STATUS( sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN), false );

  return true;
}

bool BLEPeriph::connected (uint16_t conn_hdl)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  return conn && conn->connected() && (conn->getRole() == BLE_GAP_ROLE_PERIPH);
}

uint8_t BLEPeriph::connected (void)
{
  uint8_t count = 0;
  for (uint16_t c=0; c<BLE_MAX_CONNECTION; c++)
  {
    if ( this->connected(c) ) count++;
  }

  return count;
}

bool BLEPeriph::setConnInterval (uint16_t min, uint16_t max)
{
  _ppcp.min_conn_interval = min;
  _ppcp.max_conn_interval = max;

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&_ppcp), false);

  return true;
}

bool BLEPeriph::setConnIntervalMS (uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}

bool BLEPeriph::setConnSupervisionTimeout(uint16_t timeout)
{
  _ppcp.conn_sup_timeout = timeout;

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&_ppcp), false);

  return true;
}

bool BLEPeriph::setConnSupervisionTimeoutMS(uint16_t timeout_ms)
{
  return setConnSupervisionTimeout(timeout_ms / 10); // 10ms unit
}

void BLEPeriph::setConnectCallback( ble_connect_callback_t fp )
{
  _connect_cb = fp;
}

void BLEPeriph::setDisconnectCallback( ble_disconnect_callback_t fp )
{
  _disconnect_cb = fp;
}


void BLEPeriph::_eventHandler(ble_evt_t* evt)
{
  uint16_t const conn_hdl = evt->evt.common_evt.conn_handle;

  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t* para = &evt->evt.gap_evt.params.connected;

      if (para->role == BLE_GAP_ROLE_PERIPH)
      {
        // Connection interval set by Central is out of preferred range
        // Try to negotiate with Central using our preferred values
        if ( !is_within(_ppcp.min_conn_interval, para->conn_params.min_conn_interval, _ppcp.max_conn_interval) )
        {
          VERIFY_STATUS( sd_ble_gap_conn_param_update(conn_hdl, &_ppcp), );
        }
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:

    break;

    default: break;
  }
}

void BLEPeriph::printInfo(void)
{
  char const * title_fmt = "%-16s: ";
  // prepare for ability to change output, based on compile-time flags
  Print& logger = Serial;

  logger.printf(title_fmt, "Conn Intervals");
  logger.printf("min = %.2f ms, ", _ppcp.min_conn_interval*1.25f);
  logger.printf("max = %.2f ms", _ppcp.max_conn_interval*1.25f);
  logger.println();

  logger.printf(title_fmt, "Conn Timeout");
  logger.printf("%d ms", _ppcp.conn_sup_timeout*10);
  logger.println();
}

