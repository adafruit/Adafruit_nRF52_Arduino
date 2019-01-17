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
#include "BLEUuid.h"
#include "utility/bonding.h"

enum
{
  CONN_CFG_PERIPHERAL = 1,
  CONN_CFG_CENTRAL = 2,
};

class BLEGapConnection
{
  private:
    uint16_t _conn_hdl;

  public:
    BLEGapConnection(void) { _conn_hdl = BLE_CONN_HANDLE_INVALID; }
    BLEGapConnection(uint16_t conn_hdl) { _conn_hdl = conn_hdl; }

    uint16_t handle(void) { return _conn_hdl; }
    bool     connected(void);
    uint8_t  getRole(void);

//    bool    getHvnPacket     (void);
//    bool    getWriteCmdPacket(void);

    ble_gap_addr_t addr;
    uint16_t att_mtu;
    bool paired;
    uint8_t  role;

    SemaphoreHandle_t pair_sem;

    SemaphoreHandle_t hvn_tx_sem;
    SemaphoreHandle_t wrcmd_tx_sem;
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

    bool     connected            (uint16_t conn_hdl);
    bool     paired               (uint16_t conn_hdl);
    bool     requestPairing       (uint16_t conn_hdl);

    uint8_t  getRole              (uint16_t conn_hdl);

    uint8_t        getPeerAddr    (uint16_t conn_hdl, uint8_t addr[6]);
    ble_gap_addr_t getPeerAddr    (uint16_t conn_hdl);
    uint16_t       getPeerName    (uint16_t conn_hdl, char* buf, uint16_t bufsize);

    uint16_t getMTU               (uint16_t conn_hdl);
    uint16_t getMaxMtu            (uint8_t conn_hdl);
    uint16_t getMaxMtuByConnCfg   (uint8_t conn_cfg);

    bool     getHvnPacket         (uint16_t conn_hdl);
    bool     getWriteCmdPacket    (uint16_t conn_hdl);

    void     configPrphConn       (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void     configCentralConn    (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);

//    bool     startRssi(uint16_t conn_hdl, uint8_t );
//    bool     stopRssi(uint16_t conn_hdl);
//    int8_t   getRssi(uint16_t conn_hdl);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void     _eventHandler(ble_evt_t* evt);

    void _prph_setConnectCallback   ( connect_callback_t    fp);
    void _prph_setDisconnectCallback( disconnect_callback_t fp);

    void _central_setConnectCallback   ( connect_callback_t    fp);
    void _central_setDisconnectCallback( disconnect_callback_t fp);

    // Array of TX Packet semaphore, indexed by connection handle
    // Peer info where conn_hdl serves as index
    typedef struct {
      uint16_t         ediv;
      bond_keys_t*     bond_keys; // Shared keys with bonded device, size ~ 80 bytes

      bool              hvc_received;
      uint8_t role;

      // On-demand semaphore that are created on the fly
      SemaphoreHandle_t hvc_sem;
    } gap_peer_t;

    gap_peer_t* _get_peer(uint16_t conn_hdl) { return &_peers[conn_hdl]; }

  private:
    struct {
        uint16_t mtu_max;
        uint8_t  event_len;
        uint8_t  hvn_tx_qsize;
        uint8_t  wr_cmd_qsize;
    } _cfg_prph, _cfg_central;

    gap_peer_t _peers[BLE_MAX_CONN];

    ble_gap_sec_params_t _sec_param;

    BLEGapConnection* _connection[BLE_MAX_CONN];

    struct {
        connect_callback_t connect_cb;
        disconnect_callback_t disconnect_cb;
    }_prph, _central;

    friend class AdafruitBluefruit;
};

#endif /* BLEGAP_H_ */
