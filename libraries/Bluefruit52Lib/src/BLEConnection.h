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
    uint16_t _slave_latency;
    uint16_t _sup_timeout;
    uint16_t _data_length;
    uint8_t  _phy;

    uint8_t  _role;
    uint16_t _ediv;

    bool _connected;
    bool _bonded; // have LTK stored in InternalFS
    bool _hvc_received;

    ble_gap_conn_sec_mode_t _sec_mode;

    ble_gap_addr_t _peer_addr; // resolvable connect address
    ble_gap_addr_t _bond_id_addr; // address stored as bonded

    SemaphoreHandle_t _hvn_sem;
    SemaphoreHandle_t _wrcmd_sem;

    // On-demand semaphore/data that are created on the fly
    SemaphoreHandle_t _hvc_sem;

  public:
    BLEConnection(uint16_t conn_hdl, ble_gap_evt_connected_t const * evt_connected, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    virtual ~BLEConnection();

    uint16_t handle(void);
    bool     connected(void);
    bool     bonded(void);
    bool     secured(void);

    uint8_t  getRole(void);
    uint16_t getMtu (void);
    uint16_t getConnectionInterval(void);
    uint16_t getSlaveLatency(void);
    uint16_t getSupervisionTimeout(void);
    uint16_t getDataLength(void);
    uint8_t  getPHY(void);

    ble_gap_addr_t getPeerAddr(void);
    uint16_t getPeerName(char* buf, uint16_t bufsize);

    ble_gap_conn_sec_mode_t getSecureMode(void);

    bool disconnect(void);

    bool setTxPower(int8_t power); // set power for this connection

    bool requestDataLengthUpdate(ble_gap_data_length_params_t const *p_dl_params = NULL, ble_gap_data_length_limitation_t *p_dl_limitation = NULL);
    bool requestMtuExchange(uint16_t mtu);
    bool requestPHY(uint8_t phy = BLE_GAP_PHY_AUTO);
    bool requestConnectionParameter(uint16_t conn_interval, uint16_t slave_latency = BLE_GAP_CONN_SLAVE_LATENCY, uint16_t sup_timeout = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS/10);
    bool requestPairing(void);

    bool monitorRssi(uint8_t threshold = BLE_GAP_RSSI_THRESHOLD_INVALID);
    int8_t getRssi(void);
    void stopRssi(void);

    bool getHvnPacket(void);
    bool releaseHvnPacket(void);
    bool getWriteCmdPacket(void);
    bool waitForIndicateConfirm(void);

    bool saveBondKey(bond_keys_t const* ltkey);
    bool loadBondKey(bond_keys_t* ltkey);
    bool removeBondKey(void);

    bool saveCccd(void);
    bool loadCccd(void);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal code.
     *------------------------------------------------------------------*/
    void _eventHandler(ble_evt_t* evt);
};


#endif /* BLECONNECTION_H_ */
