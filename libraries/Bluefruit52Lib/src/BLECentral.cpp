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
  _scan_cb    = NULL;
  _scan_param = (ble_gap_scan_params_t) {
    .active      = 1,
    .selective   = 0,
    .p_whitelist = NULL,
    .interval    = 0x00A0,
    .window      = 0x0050,
    .timeout     = 0, // no timeout
  };
}

void BLECentral::setScanCallback(scan_callback_t fp)
{
  _scan_cb = fp;
}

err_t BLECentral::startScanning(void)
{
  VERIFY_STATUS( sd_ble_gap_scan_start(&_scan_param) );
  Bluefruit._startConnLed(); // start blinking
  return ERROR_NONE;
}

err_t BLECentral::stopScanning(void)
{
  Bluefruit._stopConnLed(); // stop blinking
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

bool BLECentral::_checkUuidInScan(const ble_gap_evt_adv_report_t* report, const uint8_t uuid[], uint8_t uuid_len)
{
  uint8_t const* data_arr[2];
  uint8_t type_arr[2];

  // Check both UUID16 more available and complete list
  if ( uuid_len == 2)
  {
    type_arr[0] = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE;
    type_arr[1] = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE;
  }else
  {
    type_arr[0] = BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE;
    type_arr[1] = BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE;
  }

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

bool BLECentral::checkUuidInScan(const ble_gap_evt_adv_report_t* report, uint16_t uuid16)
{
  return _checkUuidInScan(report, (uint8_t*) &uuid16, 2);
}

bool BLECentral::checkUuidInScan(const ble_gap_evt_adv_report_t* report, const uint8_t uuid128[])
{
  return _checkUuidInScan(report, uuid128, 16);
}
