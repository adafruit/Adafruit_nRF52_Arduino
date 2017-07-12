/**************************************************************************/
/*!
    @file     BLEScanner.cpp
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

BLEScanner::BLEScanner(void)
{
  _runnning = false;
  _start_if_disconnect = true;
  _rx_cb    = NULL;

  _scan_param  = (ble_gap_scan_params_t) {
    .active      = 1,
    .selective   = 0,
    .p_whitelist = NULL,
    .interval    = 0x00A0,
    .window      = 0x0050,
    .timeout     = 0, // no timeout
  };
}

bool BLEScanner::isRunning(void)
{
  return _runnning;
}

void BLEScanner::setRxCallback(rx_callback_t fp)
{
  _rx_cb = fp;
}

void BLEScanner::restartOnDisconnect(bool enable)
{
  _start_if_disconnect = enable;
}

ble_gap_scan_params_t* BLEScanner::getParams(void)
{
  return &_scan_param;
}

bool BLEScanner::start(uint16_t timeout)
{
  _scan_param.timeout = timeout;
  VERIFY_STATUS( sd_ble_gap_scan_start(&_scan_param), false );

  Bluefruit._startConnLed(); // start blinking
  _runnning = true;

  return true;
}

bool BLEScanner::stop(void)
{
  VERIFY_STATUS( sd_ble_gap_scan_stop(), false );

  Bluefruit._stopConnLed(); // stop blinking
  _runnning = false;

  return true;
}

void BLEScanner::_eventHandler(ble_evt_t* evt)
{
  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_ADV_REPORT:
    {
      // evt_conn_hdl is equal to BLE_CONN_HANDLE_INVALID
      ble_gap_evt_adv_report_t* adv_report = &evt->evt.gap_evt.params.adv_report;
      if (_rx_cb) _rx_cb(adv_report); // TODO ada_callback
    }
    break;

    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;

      if ( para->role == BLE_GAP_ROLE_CENTRAL)
      {
        _runnning = false;

        // Turn on Conn LED
        Bluefruit._stopConnLed();
        Bluefruit._setConnLed(true);
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
      if ( BLE_GAP_ROLE_CENTRAL == Bluefruit.Gap.getRole(evt->evt.common_evt.conn_handle) )
      {
        // Turn off Conn LED
        Bluefruit._setConnLed(false);

        // Auto start if enabled
        if ( _start_if_disconnect )
        {
          if (!_runnning) start(_scan_param.timeout);
        }
      }
    break;

    case BLE_GAP_EVT_TIMEOUT:
      if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
      {

      }
    break;

    default: break;
  }
}
