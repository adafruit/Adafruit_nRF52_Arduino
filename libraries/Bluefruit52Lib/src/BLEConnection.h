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

  public:
    ble_gap_addr_t _addr;
    uint16_t att_mtu;
    bool _paired;
    uint8_t  role;

    SemaphoreHandle_t hvn_sem;
    SemaphoreHandle_t wrcmd_sem;

    // On-demand semaphore that are created on the fly
    SemaphoreHandle_t hvc_sem;
    bool              hvc_received;

    SemaphoreHandle_t pair_sem;

    uint16_t         ediv;
    bond_keys_t*     bond_keys; // Shared keys with bonded device, size ~ 80 bytes

    BLEConnection(uint16_t conn_hdl) { _conn_hdl = conn_hdl; }

    uint16_t handle(void) { return _conn_hdl; }
    bool     connected(void);
    bool     paired(void) { return _paired; }
    uint8_t  getRole(void);

    uint16_t getMTU (void)
    {
      return att_mtu;
    }

    ble_gap_addr_t getPeerAddr(void)
    {
      return _addr;
    }

    uint8_t getPeerAddr(uint8_t addr[6])
    {
      memcpy(addr, _addr.addr, BLE_GAP_ADDR_LEN);
      return _addr.addr_type;
    }

    bool    getHvnPacket     (void)
    {
      VERIFY(hvn_sem != NULL);
      return xSemaphoreTake(hvn_sem, ms2tick(BLE_GENERIC_TIMEOUT));
    }

    bool    getWriteCmdPacket(void)
    {
      VERIFY(wrcmd_sem != NULL);
      return xSemaphoreTake(wrcmd_sem, ms2tick(BLE_GENERIC_TIMEOUT));
    }
};


#endif /* BLECONNECTION_H_ */
