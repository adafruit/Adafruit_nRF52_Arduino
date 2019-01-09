/**************************************************************************/
/*!
    @file     bluefruit.h
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
#ifndef BLUEFRUIT_H_
#define BLUEFRUIT_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#define CFG_ADV_BLINKY_INTERVAL          500

/* Note changing these parameters will affect APP_RAM_BASE
 * --> need to update RAM region in linker file
 * - BLE_GATT_ATT_MTU_MAX from 23 (default) to 247
 */
#define BLE_GATT_ATT_MTU_MAX            247
#define BLE_PRPH_MAX_CONN               1
#define BLE_CENTRAL_MAX_CONN            4
#define BLE_CENTRAL_MAX_SECURE_CONN     1 // should be enough

#define BLE_MAX_CONN                    (BLE_CENTRAL_MAX_CONN+BLE_PRPH_MAX_CONN)

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
#include "services/EddyStone.h"

#include "clients/BLEAncs.h"
#include "clients/BLEClientUart.h"
#include "clients/BLEClientDis.h"
#include "clients/BLEClientCts.h"
#include "clients/BLEClientHidAdafruit.h"

#include "utility/AdaCallback.h"
#include "utility/bonding.h"

enum
{
  BANDWIDTH_AUTO = 0,
  BANDWIDTH_LOW,
  BANDWIDTH_NORMAL,
  BANDWIDTH_HIGH,
  BANDWIDTH_MAX,
};

extern "C"
{
  void SD_EVT_IRQHandler(void);
}

class AdafruitBluefruit
{
  public:
    AdafruitBluefruit(void); // Constructor

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
    /* SoftDevice Configure Functions, must call before begin().
     * These function affect the SRAM consumed by SoftDevice.
     *------------------------------------------------------------------*/
    void     configServiceChanged (bool     changed);
    void     configUuid128Count   (uint8_t  uuid128_max);
    void     configAttrTableSize  (uint32_t attr_table_size);

    // Config Bandwidth for connections
    void     configPrphConn        (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void     configCentralConn     (uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);

    // Convenient function to config connection
    void     configPrphBandwidth   (uint8_t bw);
    void     configCentralBandwidth(uint8_t bw);

    err_t    begin(uint8_t prph_count = 1, uint8_t central_count = 0);

    /*------------------------------------------------------------------*/
    /* General Functions
     *------------------------------------------------------------------*/
    void     setName            (const char* str);
    uint8_t  getName            (char* name, uint16_t bufsize);

    bool     setTxPower         (int8_t power);
    int8_t   getTxPower         (void);

    bool     setApperance       (uint16_t appear);
    uint16_t getApperance       (void);

    void     autoConnLed        (bool enabled);
    void     setConnLedInterval (uint32_t ms);

    /*------------------------------------------------------------------*/
    /* GAP, Connections and Bonding
     *------------------------------------------------------------------*/
    bool     connected         (void);
    bool     disconnect        (void);

    bool     setConnInterval   (uint16_t min, uint16_t max);
    bool     setConnIntervalMS (uint16_t min_ms, uint16_t max_ms);
    bool     setConnSupervisionTimeout(uint16_t timeout);
    bool     setConnSupervisionTimeoutMS(uint16_t timeout_ms);

    uint16_t connHandle        (void);
    bool     connPaired        (void);
    uint16_t connInterval      (void);

    bool     requestPairing    (void);
    void     clearBonds        (void);

    ble_gap_addr_t getPeerAddr (void);
    uint8_t        getPeerAddr (uint8_t addr[6]);

    void     printInfo(void);

    /*------------------------------------------------------------------*/
    /* Callbacks
     *------------------------------------------------------------------*/
    void setConnectCallback   ( BLEGap::connect_callback_t    fp);
    void setDisconnectCallback( BLEGap::disconnect_callback_t fp);

    void setEventCallback ( void (*fp) (ble_evt_t*) );

    COMMENT_OUT ( bool setPIN(const char* pin); )

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _startConnLed       (void);
    void _stopConnLed        (void);
    void _setConnLed         (bool on_off);

  private:
    /*------------- SoftDevice Configuration -------------*/
    struct {
      uint32_t attr_table_size;
      uint8_t  service_changed;
      uint8_t  uuid128_max;
    }_sd_cfg;

    uint8_t _prph_count;
    uint8_t _central_count;

    // Peripheral Preferred Connection Parameters (PPCP)
    uint16_t _ppcp_min_conn;
    uint16_t _ppcp_max_conn;
    uint16_t _ppcp_conn_sup_timeout;

    // Actual connection interval in use
    uint16_t _conn_interval;

    int8_t _tx_power;

    SemaphoreHandle_t _ble_event_sem;
    SemaphoreHandle_t _soc_event_sem;

    TimerHandle_t _led_blink_th;
    bool _led_conn;

    BLEDfu _dfu_svc;

    uint16_t _conn_hdl;

    BLEGap::connect_callback_t    _connect_cb;
    BLEGap::disconnect_callback_t _disconnect_cb;

    void (*_event_cb) (ble_evt_t*);

COMMENT_OUT(
    uint8_t _auth_type;
    char _pin[BLE_GAP_PASSKEY_LEN];
)

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     *------------------------------------------------------------------*/
    void _ble_handler(ble_evt_t* evt);

    friend void SD_EVT_IRQHandler(void);
    friend void adafruit_ble_task(void* arg);
    friend void adafruit_soc_task(void* arg);
    friend class BLECentral;
};

extern AdafruitBluefruit Bluefruit;

#endif
