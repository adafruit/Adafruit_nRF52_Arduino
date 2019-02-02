/**************************************************************************/
/*!
    @file     BLEScanner.h
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
#ifndef BLESCANNER_H_
#define BLESCANNER_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#define BLE_SCAN_INTERVAL_DFLT    160 // 100 ms (in 0.625 ms)
#define BLE_SCAN_WINDOW_DFLT      80  // 50  ms (in 0.625 ms)

class BLEScanner
{
public:
  typedef void (*rx_callback_t  ) (ble_gap_evt_adv_report_t*);
  typedef void (*stop_callback_t) (void);

  BLEScanner(void);

  ble_gap_scan_params_t* getParams(void);
  bool isRunning(void);

  void useActiveScan(bool enable);
  void setInterval(uint16_t interval, uint16_t window);
  void setIntervalMS(uint16_t interval, uint16_t window);
  void restartOnDisconnect(bool enable);

  void filterRssi(int8_t min_rssi);
  void filterMSD(uint16_t manuf_id);


  void filterUuid(BLEUuid ble_uuid);
  void filterUuid(BLEUuid ble_uuid1, BLEUuid ble_uuid2);
  void filterUuid(BLEUuid ble_uuid1, BLEUuid ble_uuid2, BLEUuid ble_uuid3);
  void filterUuid(BLEUuid ble_uuid1, BLEUuid ble_uuid2, BLEUuid ble_uuid3, BLEUuid ble_uuid4);
  void filterUuid(BLEUuid ble_uuid[], uint8_t count);

  void filterService(BLEService& svc);
  void filterService(BLEClientService& cli);

  void clearFilters(void);

  bool start(uint16_t timeout = 0);
  bool resume(void);
  bool stop(void);

  /*------------- Callbacks -------------*/
  void setRxCallback(rx_callback_t fp);
  void setStopCallback(stop_callback_t fp);

  /*------------- Data Parser -------------*/
  uint8_t parseReportByType(const uint8_t* scandata, uint8_t scanlen, uint8_t type, uint8_t* buf, uint8_t bufsize = 0);
  uint8_t parseReportByType(const ble_gap_evt_adv_report_t* report, uint8_t type, uint8_t* buf, uint8_t bufsize = 0);

  bool    checkReportForUuid(const ble_gap_evt_adv_report_t* report, BLEUuid ble_uuid);
  bool    checkReportForService(const ble_gap_evt_adv_report_t* report, BLEClientService& svc);
  bool    checkReportForService(const ble_gap_evt_adv_report_t* report, BLEService& svc);

  /*------------------------------------------------------------------*/
  /* INTERNAL USAGE ONLY
   * Although declare as public, it is meant to be invoked by internal
   * code. User should not call these directly
   *------------------------------------------------------------------*/
  void _eventHandler(ble_evt_t* evt);

private:
  uint8_t    _scan_data[BLE_GAP_SCAN_BUFFER_MAX];
  ble_data_t _report_data;

  uint32_t   _conn_mask;

  bool       _runnning;
  bool       _start_if_disconnect;

  int8_t     _filter_rssi;
  bool       _filter_msd_en; // since all value of manufacturer id is valid (0-FFFF)
  uint16_t   _filter_msd_id;

  BLEUuid*   _filter_uuid;
  uint8_t    _filter_uuid_count;

  rx_callback_t   _rx_cb;
  stop_callback_t _stop_cb;

  ble_gap_scan_params_t _param;
};



#endif /* BLESCANNER_H_ */
