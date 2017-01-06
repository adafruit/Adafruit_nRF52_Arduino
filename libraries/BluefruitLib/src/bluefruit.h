/**************************************************************************/
/*!
    @file     bluefruit.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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

#include <Arduino.h>
#include "utility/common_inc.h"

#include "BLEUuid.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

// Services
#include "utility/services/BLEDis.h"
#include "utility/services/BLEDfu.h"
#include "utility/services/BLEUart.h"
#include "utility/services/BLEBas.h"

#define BLE_MAX_CHARS         50

#define BLE_MAX_DATA_PER_MTU  (GATT_MTU_SIZE_DEFAULT - 3)

class AdafruitBluefruit
{
  public:
    // Constructor
    AdafruitBluefruit(void);

    err_t begin(void);
    
    void setName(const char* str);
//    bool setTxPower(int8_t power);

    // Advertising Packet
    uint8_t getAdvLen(void);
    uint8_t getAdvData(uint8_t* buffer);
    bool    setAdvData(const uint8_t* data, uint8_t count);
    void    clearAdvData(void);

    bool addAdvData(uint8_t type, const void* data, uint8_t len);
    bool addAdvFlags(uint8_t flags);
    bool addAdvTxPower(void);
    bool addAdvName(void);
    bool addAdvUuid(uint16_t uuid16);
    bool addAdvUuid(uint8_t const  uuid128[]);
    bool addAdvService(BLEService& service);

    // Scan Response Data (less helper than Adv packet)
    uint8_t getScanLen(void);
    uint8_t getScanData(uint8_t* buffer);
    bool    setScanData(const uint8_t* data, uint8_t count);
    void    clearScanData(void);

    bool addScanData(uint8_t type, const void* data, uint8_t len);
    bool addScanName(void);
    bool addScanUuid(uint16_t uuid16);
    bool addScanUuid(uint8_t const  uuid128[]);

    err_t startAdvertising(void);
    void  stopAdvertising(void);

    void poll(void);

    // Add service without using BLEService instance
    err_t addService(uint16_t uuid16);
    err_t addService(uint8_t const  uuid128[]);

    uint16_t connHandle(void);
    ble_gap_addr_t peerAddr(void);

    void txbuf_decrease(void);
    bool txbuf_available(void);

    void disconnect(void);
    bool connected(void);

    // internal usage only
    err_t _registerCharacteristic(BLECharacteristic* chars);

  private:
    BLEDfu _dfu_svc;

    // ADV Data
    struct {
      uint8_t data[BLE_GAP_ADV_MAX_SIZE];
      uint8_t count;
    } _adv, _scan_resp;

    int8_t _tx_power;

    char _name[32+1]; // 32 including null terminator

    BLECharacteristic* _chars_list[BLE_MAX_CHARS];
    uint8_t            _chars_count;

    uint16_t _conn_hdl;

    // TODO move to bonding place
    ble_gap_enc_key_t _enc_key;
    ble_gap_addr_t    _peer_addr;

    // Transmission Buffer Count for HVX notification, max is seen at 7
    uint8_t _txbuf_max;
    uint8_t _txbuf_left;

    bool _addToAdv(bool scan_resp, uint8_t type, const void* data, uint8_t len);
};

extern AdafruitBluefruit Bluefruit;



