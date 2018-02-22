/**************************************************************************/
/*!
    @file     BLEGap.h
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

class BLEGap
{
  public:
    typedef void (*connect_callback_t    ) (uint16_t conn_hdl);
    typedef void (*disconnect_callback_t ) (uint16_t conn_hdl, uint8_t reason);

    BLEGap(void);

    uint8_t        getAddr               (uint8_t mac[6]);
    bool           setAddr               (uint8_t mac[6], uint8_t type);
//    bool    setPrivacy                ();  sd_ble_gap_privacy_set()

    bool           connected            (uint16_t conn_hdl);
    bool           bonded               (uint16_t conn_hdl);
    bool           requestPairing       (uint16_t conn_hdl);

    uint8_t        getRole              (uint16_t conn_hdl);

    uint8_t        getPeerAddr          (uint16_t conn_hdl, uint8_t addr[6]);
    ble_gap_addr_t getPeerAddr          (uint16_t conn_hdl);
    uint16_t       getPeerName          (uint16_t conn_hdl, char* buf, uint16_t bufsize);

    uint16_t       getMTU               (uint16_t conn_hdl);
    uint16_t       getMaxMtuByConnCfg   (uint8_t conn_cfg);
    uint16_t       getMaxMtu            (uint8_t conn_hdl);

    uint8_t        getHvnQueueSize      (uint8_t conn_hdl);
    uint8_t        getWriteCmdQueueSize (uint8_t conn_hdl);

    bool           getHvnPacket         (uint16_t conn_hdl);
    bool           getWriteCmdPacket    (uint16_t conn_hdl);

    void           configPrphConn       (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void           configCentralConn    (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);

    // Array of TX Packet semaphore, indexed by connection handle
    // Peer info where conn_hdl serves as index
    typedef struct {
      bool     connected;
      bool     bonded;
      uint8_t  role;
      uint16_t att_mtu;

      ble_gap_addr_t addr;

      uint16_t         ediv;
      bond_data_t*     bond_data; // Shared keys with bonded device, size ~ 80 bytes

      SemaphoreHandle_t hvn_tx_sem;
      SemaphoreHandle_t wrcmd_tx_sem;

      SemaphoreHandle_t indicate_confirm_sem;
      bool              hvc_received;
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

    friend class AdafruitBluefruit;
    friend class BLEGatt;
};

#endif /* BLEGAP_H_ */
