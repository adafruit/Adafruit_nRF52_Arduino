/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach (tinyusb.org) for Adafruit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef BLUEFRUIT_H_
#define BLUEFRUIT_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#define CFG_ADV_BLINKY_INTERVAL   500

/* Note changing these parameters will affect APP_RAM_BASE
 * --> need to update RAM region in linker file
 * - BLE_GATT_ATT_MTU_MAX from 23 (default) to 247
 */
#define BLE_GATT_ATT_MTU_MAX      247
#define BLE_MAX_CONNECTION        20 // SD support up to 20 connections

// Allocate more memory for GATT table for 840
#ifdef NRF52840_XXAA
  #define CFG_SD_ATTR_TABLE_SIZE    0x1000
#else
  #define CFG_SD_ATTR_TABLE_SIZE    0xC00
#endif

#include "BLEUuid.h"
#include "BLEAdvertising.h"
#include "BLECharacteristic.h"
#include "BLEService.h"
#include "BLEScanner.h"
#include "BLEPeriph.h"
#include "BLECentral.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"
#include "BLEDiscovery.h"
#include "BLEConnection.h"
#include "BLEGatt.h"
#include "BLESecurity.h"

// Services
#include "services/BLEDis.h"
#include "services/BLEDfu.h"
#include "services/BLEUart.h"
#include "services/BLEBas.h"
#include "services/BLEBeacon.h"
#include "services/BLEHidGeneric.h"
#include "services/BLEHidAdafruit.h"
#include "services/BLEHidGamepad.h"
#include "services/BLEMidi.h"
#include "services/EddyStone.h"

#include "clients/BLEAncs.h"
#include "clients/BLEClientUart.h"
#include "clients/BLEClientDis.h"
#include "clients/BLEClientCts.h"
#include "clients/BLEClientHidAdafruit.h"
#include "clients/BLEClientBas.h"

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

enum
{
  CONN_CFG_PERIPHERAL = 1,
  CONN_CFG_CENTRAL = 2,
};

extern "C"
{
  void SD_EVT_IRQHandler(void);
}

class AdafruitBluefruit
{
  public:
    typedef void (*event_cb_t) (ble_evt_t* evt);
    typedef void (*rssi_cb_t) (uint16_t conn_hdl, int8_t rssi);

    AdafruitBluefruit(void);

    /*------------------------------------------------------------------*/
    /* Lower Level Classes (Bluefruit.Advertising.*, etc.)
     *------------------------------------------------------------------*/
    BLEPeriph          Periph;
    BLECentral         Central;
    BLESecurity        Security;
    BLEGatt            Gatt;

    BLEAdvertising     Advertising;
    BLEAdvertisingData ScanResponse;
    BLEScanner         Scanner;
    BLEDiscovery       Discovery;

    /*------------------------------------------------------------------*/
    /* SoftDevice Configure Functions, must call before begin().
     * These function affect the SRAM consumed by SoftDevice.
     *------------------------------------------------------------------*/
    void configServiceChanged (bool     changed);
    void configUuid128Count   (uint8_t  uuid128_max);
    void configAttrTableSize  (uint32_t attr_table_size);

    // Configure Bandwidth for connections
    void configPrphConn        (uint16_t mtu_max, uint16_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void configCentralConn     (uint16_t mtu_max, uint16_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize);
    void configPrphBandwidth   (uint8_t bw);
    void configCentralBandwidth(uint8_t bw);

    bool begin(uint8_t prph_count = 1, uint8_t central_count = 0);

    /*------------------------------------------------------------------*/
    /* General Functions
     *------------------------------------------------------------------*/
    ble_gap_addr_t  getAddr(void);
    uint8_t         getAddr(uint8_t mac[6]);
    bool            setAddr(ble_gap_addr_t* gap_addr);

    void     setName            (const char* str);
    uint8_t  getName            (char* name, uint16_t bufsize);

    // Supported tx_power values depending on mcu:
    // - nRF52832: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
    // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
    bool     setTxPower         (int8_t power);
    int8_t   getTxPower         (void);

    bool     setAppearance      (uint16_t appear);
    uint16_t getAppearance      (void);

    void     autoConnLed        (bool enabled);
    void     setConnLedInterval (uint32_t ms);

    /*------------------------------------------------------------------*/
    /* GAP, Connections and Bonding
     *------------------------------------------------------------------*/
    uint8_t  connected         (void); // Number of connected
    bool     connected         (uint16_t conn_hdl);

    uint8_t  getConnectedHandles(uint16_t* hdl_list, uint8_t max_count);
    uint16_t connHandle        (void);

    // Alias to BLEConnection API()
    bool     disconnect        (uint16_t conn_hdl);

    uint16_t getMaxMtu(uint8_t role);

    BLEConnection* Connection(uint16_t conn_hdl);

#ifdef ANT_LICENSE_KEY
    /*------------------------------------------------------------------*
     * Optional semaphore for additional event handlers for SD event.
     * It can be used for handling non-BLE  SD events 
     *------------------------------------------------------------------*/
    void setMultiprotocolSemaphore(SemaphoreHandle_t mprot_event_semaphore) 
    { 
      _mprot_event_sem= mprot_event_semaphore;
    } 
#endif

    /*------------------------------------------------------------------*/
    /* Callbacks
     *------------------------------------------------------------------*/
    void setRssiCallback(rssi_cb_t fp);
    void setEventCallback(event_cb_t fp);

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _startConnLed (void);
    void _stopConnLed  (void);
    void _setConnLed   (bool on_off);

    void printInfo(void);

  private:
    /*------------- SoftDevice Configuration -------------*/
    struct {
      uint32_t attr_table_size;
      uint8_t  service_changed;
      uint8_t  uuid128_max;

      // Bandwidth configuration
      struct {
        uint16_t  mtu_max;
        uint16_t  event_len;
        uint8_t   hvn_qsize;
        uint8_t   wrcmd_qsize;
      }prph, central;
    }_sd_cfg;

    uint8_t _prph_count;
    uint8_t _central_count;

    int8_t _tx_power;

    ble_gap_sec_params_t _sec_param;

    SemaphoreHandle_t _ble_event_sem;
    SemaphoreHandle_t _soc_event_sem;
#ifdef ANT_LICENSE_KEY
    /* Optional semaphore for additional event handlers for SD event.
     * It can be used for handling non-BLE  SD events 
     */
    SemaphoreHandle_t _mprot_event_sem;
#endif

    // Auto LED Blinky
    TimerHandle_t _led_blink_th;
    bool _led_conn;

    uint16_t _conn_hdl;

    BLEConnection* _connection[BLE_MAX_CONNECTION];

    //------------- Callbacks -------------//
    rssi_cb_t _rssi_cb;
    event_cb_t _event_cb;

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
