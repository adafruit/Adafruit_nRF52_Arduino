/**************************************************************************/
/*!
    @file     BLEConnection.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019, Adafruit Industries (adafruit.com)
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

#ifndef BLECONNECTION_H_
#define BLECONNECTION_H_

#include <Arduino.h>
#include "bluefruit_common.h"
#include "utility/bonding.h"

class BLEConnection
{
  private:
    uint16_t _conn_hdl;
    uint16_t _mtu;
    uint16_t _conn_interval;
    uint16_t _ediv;
    uint8_t  _role;
    bool _paired;
    bool _hvc_received;

    ble_gap_addr_t _addr;

    SemaphoreHandle_t _hvn_sem;
    SemaphoreHandle_t _wrcmd_sem;

    // On-demand semaphore/data that are created on the fly
    SemaphoreHandle_t _hvc_sem;

    SemaphoreHandle_t _pair_sem;
    bond_keys_t*     _bond_keys; // Shared keys with bonded device, size ~ 80 bytes

  public:
    BLEConnection(uint16_t conn_hdl, ble_gap_evt_connected_t const * evt_connected, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    virtual ~BLEConnection();

    uint16_t handle(void);
    bool     connected(void);
    bool     paired(void);
    uint8_t  getRole(void);

    uint16_t getMtu (void);

    ble_gap_addr_t getPeerAddr(void);
    uint8_t getPeerAddr(uint8_t addr[6]);

    bool getHvnPacket(void);
    bool getWriteCmdPacket(void);
    bool waitForIndicateConfirm(void);

    bool requestPairing(void);
    bool storeCccd(void);
    bool loadKeys(bond_keys_t* bkeys);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal code.
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);

};


#endif /* BLECONNECTION_H_ */
