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
#include "bluefruit_common.h"

#include "BLEUuid.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

// Services
#include "services/BLEDis.h"
#include "services/BLEDfu.h"
#include "services/BLEUart.h"
#include "services/BLEBas.h"
#include "services/BLEBeacon.h"
#include "services/BLEHidGeneric.h"
#include "services/BLEHidAdafruit.h"


#define BLE_MAX_CHARS         50

#define BLE_MAX_DATA_PER_MTU  (GATT_MTU_SIZE_DEFAULT - 3)

class AdafruitBluefruit
{
  public:
    // Constructor
    AdafruitBluefruit(void);

    err_t begin(void);
    
    void autoConnLed(bool enabled);

    void setName(const char* str);
//    bool setTxPower(int8_t power);

    /*------------- Advertising  -------------*/
    uint8_t getAdvLen(void);
    uint8_t getAdvData(uint8_t* buffer);
    bool    setAdvData(const uint8_t* data, uint8_t count);
    void    clearAdvData(void);

    bool addAdvData(uint8_t type, const void* data, uint8_t len);

    bool addAdvFlags(uint8_t flags);
    bool addAdvTxPower(void);
    bool addAdvName(void);
    bool addAdvApperance(uint16_t appearance);

    bool addAdvUuid(uint16_t uuid16);
    bool addAdvUuid(uint8_t const  uuid128[]);
    bool addAdvService(BLEService& service);

    bool setAdvBeacon(BLEBeacon& beacon);

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

    // Add service without using BLEService instance
    err_t addService(uint16_t uuid16);
    err_t addService(uint8_t const  uuid128[]);

    void disconnect(void);
    bool connected(void);

    err_t setConnInterval(uint16_t min, uint16_t max);
    err_t setConnIntervalMS(uint16_t min_ms, uint16_t max_ms);

    uint16_t connHandle(void);
    ble_gap_addr_t peerAddr(void);

    bool txbuf_get(uint32_t ms);
    err_t saveAllCCCD(void);

    /*------------- Callback -------------*/
    void setConnectCallback   ( void (*fp) (uint32_t hostID) );
    void setDisconnectCallback( void (*fp) (uint32_t hostID, uint8_t reason) );

    // internal usage only
    err_t _registerCharacteristic(BLECharacteristic* chars);
    void  _sd_event_isr(void);

  private:
    SemaphoreHandle_t _ble_event_sem;
    BLEDfu _dfu_svc;

    TimerHandle_t _led_blink_th;
    bool _led_conn;

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
    bool     _bonded;

    // TODO move to bonding place
    ble_gap_enc_key_t _enc_key;
    ble_gap_id_key_t  _peer_id;

    ble_gap_addr_t    _peer_addr;

    uint8_t*          _sys_attr;
    uint16_t          _sys_attr_len;

    // Transmission Buffer Count for HVX notification, max is seen at 7
    SemaphoreHandle_t _txbuf_sem;

    void (*_connect_cb) (uint32_t hostID);
    void (*_discconnect_cb) (uint32_t hostID, uint8_t reason);

    bool _addToAdv(bool scan_resp, uint8_t type, const void* data, uint8_t len);
    void _poll(void);

    friend void adafruit_bluefruit_task(void* arg);
};

extern AdafruitBluefruit Bluefruit;



