/**************************************************************************/
/*!
    @file     BLECentral.h
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
#ifndef BLECENTRAL_H_
#define BLECENTRAL_H_

#include <Arduino.h>
#include "bluefruit_common.h"

class AdafruitBluefruit;

class BLECentral
{
  public:
    BLECentral(void); // Constructor
    void begin(void);

    /*------------------------------------------------------------------*/
    /* GAP
     *------------------------------------------------------------------*/
    bool setConnInterval(uint16_t min, uint16_t max);
    bool setConnIntervalMS(uint16_t min_ms, uint16_t max_ms);

    bool connect(const ble_gap_evt_adv_report_t* adv_report);
    bool connect(const ble_gap_addr_t *peer_addr);

    bool    connected(uint16_t conn_hdl); // Connected as central to this connection
    uint8_t connected(void);              // Number of connected Peripherals

    void clearBonds (void);

    /*------------- Callbacks -------------*/
    void setConnectCallback   ( ble_connect_callback_t    fp);
    void setDisconnectCallback( ble_disconnect_callback_t fp);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal code.
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);

  private:
    ble_gap_conn_params_t _conn_param;

    ble_connect_callback_t _connect_cb;
    ble_disconnect_callback_t _disconnect_cb;

    friend class AdafruitBluefruit;
};



#endif /* BLECENTRAL_H_ */
