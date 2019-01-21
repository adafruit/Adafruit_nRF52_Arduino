/**************************************************************************/
/*!
    @file     BLEGatt.h
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
#ifndef BLEGATT_H_
#define BLEGATT_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#include "BLEUuid.h"
#include "BLECharacteristic.h"
#include "BLEClientCharacteristic.h"
#include "BLEService.h"

#include "BLEClientService.h"

#define CFG_GATT_MAX_SERVER_SERVICE      20
#define CFG_GATT_MAX_SERVER_CHARS        40

#define CFG_GATT_MAX_CLIENT_SERVICE      20
#define CFG_GATT_MAX_CLIENT_CHARS        40


class BLEGatt
{
  public:
    BLEGatt(void);

    uint16_t readCharByUuid(uint16_t conn_hdl, BLEUuid bleuuid, void* buffer, uint16_t bufsize, uint16_t start_hdl = 1, uint16_t end_hdl = 0xffff);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    // Server
    bool _addCharacteristic(BLECharacteristic* chr);
    bool _addService(BLEService* svc);

    // Client
    bool _addCharacteristic(BLEClientCharacteristic* chr);
    void _removeCharacteristic(BLEClientCharacteristic* chr);
    bool _addService(BLEClientService* svc);

    void _eventHandler(ble_evt_t* evt);

  private:
    struct {
      uint8_t            svc_count;
      BLEService*        svc_list[CFG_GATT_MAX_SERVER_SERVICE];

      uint8_t            chr_count;
      BLECharacteristic* chr_list[CFG_GATT_MAX_SERVER_CHARS];
    } _server;

    struct {
      uint8_t                  svc_count;
      BLEClientService*        svc_list[CFG_GATT_MAX_CLIENT_SERVICE];

      uint8_t                  chr_count;
      BLEClientCharacteristic* chr_list[CFG_GATT_MAX_CLIENT_CHARS];
    }_client;

    AdaMsg             _adamsg;
};

#endif /* BLEGATT_H_ */
