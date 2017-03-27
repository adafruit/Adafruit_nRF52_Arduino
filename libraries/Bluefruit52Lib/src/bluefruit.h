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
#include "BLEAdvertising.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

#include "BLECentral.h"
#include "BLECentralCharacteristic.h"
#include "BLECentralService.h"

// Services
#include "services/BLEDis.h"
#include "services/BLEDfu.h"
#include "services/BLEUart.h"
#include "services/BLEBas.h"
#include "services/BLEBeacon.h"
#include "services/BLEHidGeneric.h"
#include "services/BLEHidAdafruit.h"
#include "services/BLEMidi.h"

#include "central_services/BLECentralUart.h"

#define BLE_MAX_CHARS         50
#define BLE_MAX_DATA_PER_MTU  (GATT_MTU_SIZE_DEFAULT - 3)

extern "C"
{
  void SD_EVT_IRQHandler(void);
}

class AdafruitBluefruit
{
  public:
    // Constructor
    AdafruitBluefruit(void);

    err_t begin(bool prph_enable = true, bool central_enable = false);

    /*------------------------------------------------------------------*/
    /* Lower Level Classes (Bluefruit.Advertising.*, etc.)
     *------------------------------------------------------------------*/
    BLEAdvertising Advertising;
    BLEAdvertising ScanResponse;
    BLECentral     Central;

    /*------------------------------------------------------------------*/
    /* General Purpose Functions
     *------------------------------------------------------------------*/
    void   autoConnLed         (bool enabled);
    void   setConnLedInterval  (uint32_t ms);
    void   startConnLed        (void);
    void   stopConnLed         (void);
    void   setName             (const char* str);
    char*  getName             (void);
    bool   setTxPower          (int8_t power);
    int8_t getTxPower          (void);

    /*------------------------------------------------------------------*/
    /* GAP, Connections and Bonding
     *------------------------------------------------------------------*/
    bool     connected         (void);
    void     disconnect        (void);
    err_t    setConnInterval   (uint16_t min, uint16_t max);
    err_t    setConnIntervalMS (uint16_t min_ms, uint16_t max_ms);
    uint16_t connHandle        (void);
    bool     connBonded        (void);
    uint16_t connInterval      (void);
    void     clearBonds        (void);

    ble_gap_addr_t peerAddr(void);

    bool getTxPacket(uint32_t ms);

    /*------------------------------------------------------------------*/
    /* Callbacks
     *------------------------------------------------------------------*/
    typedef void (*connect_callback_t) (void);
    typedef void (*disconnect_callback_t) (uint8_t reason);

    void setConnectCallback   ( connect_callback_t    fp);
    void setDisconnectCallback( disconnect_callback_t fp);

    COMMENT_OUT ( bool setPIN(const char* pin); )

    // internal usage only
    bool _registerCharacteristic(BLECharacteristic* chars);

  private:
    /*------------- BLE para -------------*/
    bool _prph_enabled;
    bool _central_enabled;

    // Peripheral Preferred Connection Parameters (PPCP)
    uint16_t _ppcp_min_conn;
    uint16_t _ppcp_max_conn;

    // Actual connection interval in use
    uint16_t _conn_interval;

    int8_t _tx_power;
    char _name[32+1];

    SemaphoreHandle_t _ble_event_sem;
    SemaphoreHandle_t _soc_event_sem;

    TimerHandle_t _led_blink_th;
    bool _led_conn;

    BLEDfu _dfu_svc;

    BLECharacteristic* _chars_list[BLE_MAX_CHARS];
    uint8_t            _chars_count;

    uint16_t _conn_hdl;
    bool     _bonded;

public: // TODO temporary for bledfu to load bonding data
    struct
    {
      // Keys
      ble_gap_enc_key_t own_enc;
      ble_gap_enc_key_t peer_enc;
      ble_gap_id_key_t  peer_id;
    } _bond_data;

private:
    ble_gap_addr_t    _peer_addr;

COMMENT_OUT(
    uint8_t _auth_type;
    char _pin[BLE_GAP_PASSKEY_LEN];
)

    // Transmission Buffer Count for HVX notification, max is seen at 7
    SemaphoreHandle_t _txpacket_sem;

    /*------------- Callbacks -------------*/
    connect_callback_t    _connect_cb;
    disconnect_callback_t _discconnect_cb;

    bool _addToAdv(bool scan_resp, uint8_t type, const void* data, uint8_t len);

    bool _saveBondKeys(void);
    bool _loadBondKeys(uint16_t ediv);

    void _saveBondedCCCD(void);
    void _loadBondedCCCD(uint16_t ediv);

    void _ble_handler(ble_evt_t* evt);

    friend void SD_EVT_IRQHandler(void);
    friend void adafruit_ble_task(void* arg);
    friend void adafruit_soc_task(void* arg);
    friend class BLECentral;
};

extern AdafruitBluefruit Bluefruit;
