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

class BLEGap
{
  public:
    typedef void (*connect_callback_t    ) (uint16_t conn_handle);
    typedef void (*disconnect_callback_t ) (uint16_t conn_handle, uint8_t reason);

    BLEGap(void);

    uint8_t getAddr(uint8_t mac[6]);
    bool    setAddr(uint8_t mac[6], uint8_t type);
//    bool    setPrivacy()

    bool connected(uint16_t conn_handle);

    uint8_t        getRole(uint16_t conn_handle);

    uint8_t        getPeerAddr(uint16_t conn_handle, uint8_t addr[6]);
    ble_gap_addr_t getPeerAddr(uint16_t conn_handle);

    uint16_t       getPeerName(uint16_t conn_handle, char* buf, uint16_t bufsize);
    bool           getTxPacket(uint16_t conn_handle);

    uint16_t       getMTU(uint16_t conn_handle);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);

  private:
    // Array of TX Packet semaphore, indexed by connection handle
    // Peer info where conn_handle serves as index
    typedef struct {
      bool    connected;
      uint8_t role;

      ble_gap_addr_t addr;

      SemaphoreHandle_t txpacket_sem;
    } gap_peer_t;

    gap_peer_t _peers[BLE_MAX_CONN];
};

#endif /* BLEGAP_H_ */
