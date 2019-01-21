/**************************************************************************/
/*!
    @file     BLEGap.h
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
#ifndef BLEGAP_H_
#define BLEGAP_H_

#include <Arduino.h>
#include "bluefruit_common.h"
#include "BLEConnection.h"
#include "BLEUuid.h"
#include "utility/bonding.h"

enum
{
  CONN_CFG_PERIPHERAL = 1,
  CONN_CFG_CENTRAL = 2,
};


class BLEGap
{
  public:
    typedef void (*connect_callback_t    ) (uint16_t conn_hdl);
    typedef void (*disconnect_callback_t ) (uint16_t conn_hdl, uint8_t reason);

    BLEGap(void);

    uint8_t  getAddr              (uint8_t mac[6]);
    bool     setAddr              (uint8_t mac[6], uint8_t type);
//    bool    setPrivacy                ();  sd_ble_gap_privacy_set()

    ble_gap_sec_params_t getSecureParam(void)
    {
      return _sec_param;
    }

    BLEConnection* getConnection(uint16_t conn_hdl);
    bool     connected           (uint16_t conn_hdl);
    bool     requestPairing      (uint16_t conn_hdl);

    uint8_t  getRole             (uint16_t conn_hdl);
    uint16_t getPeerName         (uint16_t conn_hdl, char* buf, uint16_t bufsize);

    uint16_t getMaxMtu(uint8_t role);

    void     configPrphConn      (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void     configCentralConn   (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);

//    bool     startRssi(uint16_t conn_hdl, uint8_t );
//    bool     stopRssi(uint16_t conn_hdl);
//    int8_t   getRssi(uint16_t conn_hdl);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal code.
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);

    void _prph_setConnectCallback   ( connect_callback_t    fp);
    void _prph_setDisconnectCallback( disconnect_callback_t fp);

    void _central_setConnectCallback   ( connect_callback_t    fp);
    void _central_setDisconnectCallback( disconnect_callback_t fp);

    // Array of TX Packet semaphore, indexed by connection handle
    // Peer info where conn_hdl serves as index
    typedef struct {
      uint8_t role;
    } gap_peer_t;

  private:
    struct {
      // Bandwidth configuration
      uint16_t mtu_max;
      uint8_t  event_len;
      uint8_t  hvn_qsize;
      uint8_t  wrcmd_qsize;

      connect_callback_t connect_cb;
      disconnect_callback_t disconnect_cb;
    } _prph, _central;

    gap_peer_t _peers[BLE_MAX_CONN];

    ble_gap_sec_params_t _sec_param;

    BLEConnection* _connection[BLE_MAX_CONN];

    friend class AdafruitBluefruit;
};

#endif /* BLEGAP_H_ */
