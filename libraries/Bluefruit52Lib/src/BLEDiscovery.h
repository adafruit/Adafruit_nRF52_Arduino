/**************************************************************************/
/*!
    @file     BLEDiscovery.h
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
#ifndef BLEDISCOVERY_H_
#define BLEDISCOVERY_H_

#include <Arduino.h>
#include "bluefruit_common.h"
#include "BLEClientCharacteristic.h"

#include "BLEUuid.h"
#include "BLEClientService.h"

#define BLE_DISCOVERY_TIMEOUT     1000

class BLEDiscovery
{
  private:
    ble_gattc_handle_range_t _hdl_range;
    AdaMsg _adamsg;
    bool   _begun;

    void  _eventHandler(ble_evt_t* evt);

  public:
    BLEDiscovery(void);

    void     begin(void);
    bool     begun(void);

    void                     setHandleRange(ble_gattc_handle_range_t handle_range);
    ble_gattc_handle_range_t getHandleRange(void);

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic* chr[], uint8_t count);

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3, BLEClientCharacteristic& chr4)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3, &chr4};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3, BLEClientCharacteristic& chr4, BLEClientCharacteristic& chr5)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3, &chr4, &chr5};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3, BLEClientCharacteristic& chr4, BLEClientCharacteristic& chr5, BLEClientCharacteristic& chr6)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3, &chr4, &chr5, &chr6};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3, BLEClientCharacteristic& chr4, BLEClientCharacteristic& chr5, BLEClientCharacteristic& chr6, BLEClientCharacteristic& chr7)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3, &chr4, &chr5, &chr6, &chr7};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    uint8_t  discoverCharacteristic(uint16_t conn_handle, BLEClientCharacteristic& chr1, BLEClientCharacteristic& chr2, BLEClientCharacteristic& chr3, BLEClientCharacteristic& chr4, BLEClientCharacteristic& chr5, BLEClientCharacteristic& chr6, BLEClientCharacteristic& chr7, BLEClientCharacteristic& chr8)
    {
      BLEClientCharacteristic* chr_arr[] = {&chr1, &chr2, &chr3, &chr4, &chr5, &chr6, &chr7, &chr8};
      return discoverCharacteristic(conn_handle, chr_arr, arrcount(chr_arr));
    }

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    bool     _discoverService(uint16_t conn_handle, BLEClientService& svc, uint16_t start_handle = 1);
    uint16_t _discoverDescriptor(uint16_t conn_handle, ble_gattc_evt_desc_disc_rsp_t* disc_desc, uint16_t bufsize, ble_gattc_handle_range_t hdl_range);

    friend class AdafruitBluefruit;
};

#endif /* BLEDISCOVERY_H_ */
