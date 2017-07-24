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
#ifndef BLUEFRUIT_H_
#define BLUEFRUIT_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#define CFG_ADV_BLINKY_INTERVAL          500
#define CFG_MAX_DEVNAME_LEN              32

// Note chaning these parameters will affect APP_RAM_BASE
// --> need to update RAM in feather52_s132.ld linker
#define BLE_GATTS_ATTR_TABLE_SIZE        0x0B00
#define BLE_VENDOR_UUID_MAX              10
#define BLE_PRPH_MAX_CONN                1
#define BLE_CENTRAL_MAX_CONN             4
#define BLE_CENTRAL_MAX_SECURE_CONN      1 // should be enough

#define BLE_MAX_CONN                     (BLE_CENTRAL_MAX_CONN+BLE_PRPH_MAX_CONN)

#include "BLEUuid.h"
#include "BLEAdvertising.h"
#include "BLECharacteristic.h"
#include "BLEService.h"

#include "BLEScanner.h"
#include "BLECentral.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"
#include "BLEDiscovery.h"
#include "BLEGap.h"
#include "BLEGatt.h"

// Services
#include "services/BLEDis.h"
#include "services/BLEDfu.h"
#include "services/BLEUart.h"
#include "services/BLEBas.h"
#include "services/BLEBeacon.h"
#include "services/BLEHidGeneric.h"
#include "services/BLEHidAdafruit.h"
#include "services/BLEMidi.h"

#include "clients/BLEAncs.h"
#include "clients/BLEClientUart.h"
#include "clients/BLEClientDis.h"
#include "clients/BLEClientCts.h"

#include "utility/AdaCallback.h"

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
    BLEGap             Gap;
    BLEGatt            Gatt;

    BLEAdvertising     Advertising;
    BLEAdvertisingData ScanResponse;
    BLEScanner         Scanner;
    BLECentral         Central;
    BLEDiscovery       Discovery;

    /*------------------------------------------------------------------*/
    /* General Purpose Functions
     *------------------------------------------------------------------*/
    void    autoConnLed         (bool enabled);
    void    setConnLedInterval  (uint32_t ms);

    void    setName             (const char* str);
    uint8_t getName             (char* name, uint16_t bufsize);

    bool    setTxPower          (int8_t power);
    int8_t  getTxPower          (void);

    /*------------------------------------------------------------------*/
    /* GAP, Connections and Bonding
     *------------------------------------------------------------------*/
    bool     connected         (void);
    bool     disconnect        (void);

    bool     setConnInterval   (uint16_t min, uint16_t max);
    bool     setConnIntervalMS (uint16_t min_ms, uint16_t max_ms);

    uint16_t connHandle        (void);
    bool     connPaired        (void);
    uint16_t connInterval      (void);

    bool     requestPairing    (void);
    void     clearBonds        (void);

    ble_gap_addr_t getPeerAddr(void);
    uint8_t        getPeerAddr(uint8_t addr[6]);

    void      printInfo(void);

    /*------------------------------------------------------------------*/
    /* Callbacks
     *------------------------------------------------------------------*/
    void setConnectCallback   ( BLEGap::connect_callback_t    fp);
    void setDisconnectCallback( BLEGap::disconnect_callback_t fp);

    COMMENT_OUT ( bool setPIN(const char* pin); )

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _startConnLed       (void);
    void _stopConnLed        (void);
    void _setConnLed         (bool on_off);
    void _bledfu_get_bond_data(ble_gap_addr_t* addr, ble_gap_irk_t* irk, ble_gap_enc_key_t* enc_key);

    void _saveBondKeys(void);
    void _saveBondCCCD(void);

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

    SemaphoreHandle_t _ble_event_sem;
    SemaphoreHandle_t _soc_event_sem;

    TimerHandle_t _led_blink_th;
    bool _led_conn;

    BLEDfu _dfu_svc;

    uint16_t _conn_hdl;
    bool     _bonded;

    BLEGap::connect_callback_t    _connect_cb;
    BLEGap::disconnect_callback_t _disconnect_cb;

    ble_gap_sec_params_t _sec_param;

    // Shared keys with bonded device, size = 80 bytes
    struct
    {
      ble_gap_enc_key_t own_enc;
      ble_gap_enc_key_t peer_enc;
      ble_gap_id_key_t  peer_id;
    } _bond_data;

    enum
    {
      BOND_FILE_DEVNAME_OFFSET = sizeof(_bond_data),
      BOND_FILE_CCCD_OFFSET    = BOND_FILE_DEVNAME_OFFSET + CFG_MAX_DEVNAME_LEN
    };

COMMENT_OUT(
    uint8_t _auth_type;
    char _pin[BLE_GAP_PASSKEY_LEN];
)

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     *------------------------------------------------------------------*/
    bool _loadBondKeys(uint16_t ediv);
    void _loadBondCCCD(uint16_t ediv);
    void _ble_handler(ble_evt_t* evt);

    friend void SD_EVT_IRQHandler(void);
    friend void adafruit_ble_task(void* arg);
    friend void adafruit_soc_task(void* arg);
    friend class BLECentral;
};

extern AdafruitBluefruit Bluefruit;

#endif
