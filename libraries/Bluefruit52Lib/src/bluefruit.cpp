/**************************************************************************/
/*!
    @file     bluefruit.cpp
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

#include "bluefruit.h"
#include "utility/bonding.h"

#ifndef CFG_BLE_TX_POWER_LEVEL
#define CFG_BLE_TX_POWER_LEVEL    0
#endif

#ifndef CFG_BLE_TASK_STACKSIZE
#define CFG_BLE_TASK_STACKSIZE    (256*5)
#endif

#ifndef CFG_SOC_TASK_STACKSIZE
#define CFG_SOC_TASK_STACKSIZE    (200)
#endif

#ifdef USB_PRODUCT
  #define CFG_DEFAULT_NAME    USB_PRODUCT
#else
  #define CFG_DEFAULT_NAME    "Feather nRF52832"
#endif

#ifdef USE_TINYUSB
#include "nrfx_power.h"

/* tinyusb function that handles power event (detected, ready, removed)
 * We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled. */
extern "C" void tusb_hal_nrf_power_event(uint32_t event);

// Must be called before sd_softdevice_enable()
// NRF_POWER is restricted prph used by Softdevice, must be release before enable SD
void usb_softdevice_pre_enable(void)
{
  nrfx_power_usbevt_disable();
  nrfx_power_usbevt_uninit();
  nrfx_power_uninit();
}

// Must be called after sd_softdevice_enable()
// To re-enable USB
void usb_softdevice_post_enable(void)
{
  sd_power_usbdetected_enable(true);
  sd_power_usbpwrrdy_enable(true);
  sd_power_usbremoved_enable(true);

  uint32_t usb_reg;
  sd_power_usbregstatus_get(&usb_reg);

  // Note: Detect event is possibly handled by usb_hardware_init() however depending on how fast
  // Bluefruit.begin() is called, Ready event may or may not be handled before we disable the nrfx_power.
  //    USBPULLUP not enabled -> Ready event not yet handled
  if ( (usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk) && (NRF_USBD->USBPULLUP == 0) )
  {
    tusb_hal_nrf_power_event(NRFX_POWER_USB_EVT_READY);
  }
}

#endif

AdafruitBluefruit Bluefruit;

/*------------------------------------------------------------------*/
/* PROTOTYPTES
 *------------------------------------------------------------------*/
extern "C"
{
void flash_nrf5x_event_cb (uint32_t event) ATTR_WEAK;
}

void adafruit_ble_task(void* arg);
void adafruit_soc_task(void* arg);

/*------------------------------------------------------------------*/
/* INTERNAL FUNCTION
 *------------------------------------------------------------------*/
static void bluefruit_blinky_cb( TimerHandle_t xTimer )
{
  (void) xTimer;
  digitalToggle(LED_BLUE);
}

static void nrf_error_cb(uint32_t id, uint32_t pc, uint32_t info)
{
#if CFG_DEBUG
  PRINT_INT(id);
  PRINT_HEX(pc);
  PRINT_HEX(info);

  if ( id == NRF_FAULT_ID_SD_ASSERT && info != 0)
  {
    typedef struct
    {
        uint16_t        line_num;    /**< The line number where the error occurred. */
        uint8_t const * p_file_name; /**< The file in which the error occurred. */
    } assert_info_t;

    assert_info_t* assert_info = (assert_info_t*) info;

    LOG_LV1("SD Err", "assert at %s : %d", assert_info->p_file_name, assert_info->line_num);
  }

  while(1) yield();
#endif
}

// Constructor
AdafruitBluefruit::AdafruitBluefruit(void)
{
  /*------------------------------------------------------------------*/
  /*  SoftDevice Default Configuration
   *  Most config use Nordic default value, except the follows:
   *  - Max MTU : up to 247 for maximum throughput
   *
   *  Attr Table Size, HVN queue size, Write Command queue size is
   *  determined later in begin() depending on number of peripherals
   *  and central connections for optimum SRAM usage.
   */
  /*------------------------------------------------------------------*/
  varclr(&_sd_cfg);

  _sd_cfg.attr_table_size = CFG_SD_ATTR_TABLE_SIZE;
  _sd_cfg.uuid128_max     = BLE_UUID_VS_COUNT_DEFAULT;
  _sd_cfg.service_changed = 1;

  _sd_cfg.prph.mtu_max     = BLE_GATT_ATT_MTU_DEFAULT;
  _sd_cfg.prph.event_len   = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _sd_cfg.prph.hvn_qsize   = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _sd_cfg.prph.wrcmd_qsize = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;

  _sd_cfg.central.mtu_max     = BLE_GATT_ATT_MTU_DEFAULT;
  _sd_cfg.central.event_len   = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _sd_cfg.central.hvn_qsize   = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _sd_cfg.central.wrcmd_qsize = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;

  _prph_count    = 0;
  _central_count = 0;

  memclr(_connection, sizeof(_connection));

  _ble_event_sem = NULL;
  _soc_event_sem = NULL;
#ifdef ANT_LICENSE_KEY
  _mprot_event_sem = NULL;
#endif

  _led_blink_th  = NULL;
  _led_conn      = true;

  _tx_power      = CFG_BLE_TX_POWER_LEVEL;

  _conn_hdl      = BLE_CONN_HANDLE_INVALID;

  _event_cb = NULL;
  _rssi_cb = NULL;
}

void AdafruitBluefruit::configServiceChanged(bool changed)
{
  _sd_cfg.service_changed = (changed ? 1 : 0);
}

void AdafruitBluefruit::configUuid128Count(uint8_t uuid128_max)
{
  _sd_cfg.uuid128_max = uuid128_max;
}

void AdafruitBluefruit::configAttrTableSize(uint32_t attr_table_size)
{
  _sd_cfg.attr_table_size = align4( maxof(attr_table_size, (uint32_t)(BLE_GATTS_ATTR_TAB_SIZE_MIN)) );
}

void AdafruitBluefruit::configPrphConn(uint16_t mtu_max, uint16_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _sd_cfg.prph.mtu_max     = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);;
  _sd_cfg.prph.event_len   = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _sd_cfg.prph.hvn_qsize   = hvn_qsize;
  _sd_cfg.prph.wrcmd_qsize = wrcmd_qsize;
}

void AdafruitBluefruit::configCentralConn(uint16_t mtu_max, uint16_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _sd_cfg.central.mtu_max     = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);;
  _sd_cfg.central.event_len   = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
  _sd_cfg.central.hvn_qsize   = hvn_qsize;
  _sd_cfg.central.wrcmd_qsize = wrcmd_qsize;

}

void AdafruitBluefruit::configPrphBandwidth(uint8_t bw)
{
  /* Note default value from SoftDevice are
   * MTU = 23, Event Len = 3, HVN QSize = 1, WrCMD QSize =1
   */
  switch (bw)
  {
    case BANDWIDTH_LOW:
      configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_MIN, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    // TODO Bandwidth auto
    case BANDWIDTH_AUTO:
    case BANDWIDTH_NORMAL:
      configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    case BANDWIDTH_HIGH:
      configPrphConn(128, 6, 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    case BANDWIDTH_MAX:
      configPrphConn(247, 100, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    default: break;
  }
}

void AdafruitBluefruit::configCentralBandwidth(uint8_t bw)
{
  /* Note default value from SoftDevice are
   * MTU = 23, Event Len = 3, HVN QSize = 1, WrCMD QSize =1
   */
  switch (bw)
  {
    case BANDWIDTH_LOW:
      configCentralConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_MIN, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    // TODO Bandwidth auto
    case BANDWIDTH_AUTO:
    case BANDWIDTH_NORMAL:
      configCentralConn(BLE_GATT_ATT_MTU_DEFAULT, BLE_GAP_EVENT_LENGTH_DEFAULT, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    case BANDWIDTH_HIGH:
      configCentralConn(128, 6, 2, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    case BANDWIDTH_MAX:
      configCentralConn(247, 6, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    break;

    default: break;
  }
}

bool AdafruitBluefruit::begin(uint8_t prph_count, uint8_t central_count)
{
  _prph_count    = prph_count;
  _central_count = central_count;

#ifdef USE_TINYUSB
  usb_softdevice_pre_enable();
#endif

  // Configure Clock
#if defined( USE_LFXO )
  nrf_clock_lf_cfg_t clock_cfg =
  {
    // LFXO
    .source        = NRF_CLOCK_LF_SRC_XTAL,
    .rc_ctiv       = 0,
    .rc_temp_ctiv  = 0,
    .accuracy      = NRF_CLOCK_LF_ACCURACY_20_PPM
  };
#elif defined( USE_LFRC )
  nrf_clock_lf_cfg_t clock_cfg = 
  {
    // LXRC
    .source        = NRF_CLOCK_LF_SRC_RC,
    .rc_ctiv       = 16,
    .rc_temp_ctiv  = 2,
    .accuracy      = NRF_CLOCK_LF_ACCURACY_250_PPM
  };
#else
  #error Clock Source is not configured, define USE_LFXO or USE_LFRC according to your board in variant.h
#endif

  // Enable SoftDevice
#ifdef ANT_LICENSE_KEY
  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, nrf_error_cb, ANT_LICENSE_KEY), false );
#else
  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, nrf_error_cb), false );
#endif

#ifdef USE_TINYUSB
  usb_softdevice_post_enable();
#endif

  /*------------------------------------------------------------------*/
  /*  SoftDevice Default Configuration depending on the number of
   * prph and central connections for optimal SRAM usage.
   *
   * - If Peripheral mode is enabled
   *   - ATTR Table Size          = CFG_SD_ATTR_TABLE_SIZE.
   *   - HVN TX Queue Size        = 3
   *
   * - If Central mode is enabled
   *   - Write Command Queue Size = 3
   *
   * Otherwise value will have default as follows:
   *  - ATTR Table Size           = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT (0x580)
   *  - HVN TX Queue Size         = 1
   *  - Write Command Queue Size  = 1
   *
   *  Note: Value is left as it is if already configured by user.
   */
  /*------------------------------------------------------------------*/

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Vendor UUID count
  varclr(&blecfg);
  blecfg.common_cfg.vs_uuid_cfg.vs_uuid_count = _sd_cfg.uuid128_max;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &blecfg, ram_start), false );

  // Roles
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = _prph_count;
  blecfg.gap_cfg.role_count_cfg.central_role_count = _central_count;
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = (_central_count ? 1 : 0); // 1 should be enough
  VERIFY_STATUS( sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start), false );

  // Device Name
//  varclr(&blecfg);
//  blecfg.gap_cfg.device_name_cfg =
//  VERIFY_STATUS( sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &blecfg, ram_start) );

  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = _sd_cfg.service_changed;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start), false );

  // ATTR Table Size
  varclr(&blecfg);
  blecfg.gatts_cfg.attr_tab_size.attr_tab_size = _sd_cfg.attr_table_size;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &blecfg, ram_start), false );

  /*------------- Event Length + MTU + HVN queue + WRITE CMD queue setting affecting bandwidth -------------*/
  if ( _prph_count )
  {
    // ATT MTU
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = _sd_cfg.prph.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start), false );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _prph_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length = _sd_cfg.prph.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start), false );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = _sd_cfg.prph.hvn_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start), false );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = _sd_cfg.prph.wrcmd_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &blecfg, ram_start), false );
  }

  if ( _central_count)
  {
    // ATT MTU
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = _sd_cfg.central.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start), false );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _central_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length =_sd_cfg.central.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start), false );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = _sd_cfg.central.hvn_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start), false );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = _sd_cfg.central.wrcmd_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &blecfg, ram_start), false );
  }

  // Enable BLE stack
  // The memory requirement for a specific configuration will not increase
  // between SoftDevices with the same major version number
  uint32_t err = sd_ble_enable(&ram_start);
  if ( err )
  {
    LOG_LV1("CFG", "SoftDevice config require more SRAM than provided by linker.\n"
                 "App Ram Start must be at least 0x%08lX (provided 0x%08lX)\n"
                 "Please update linker file or re-config SoftDevice", ram_start, (uint32_t) __data_start__);
  }

  LOG_LV1("CFG", "SoftDevice's RAM requires: 0x%08lX", ram_start);
  VERIFY_STATUS(err, false);

  /*------------- Configure BLE Option -------------*/
  ble_opt_t  opt;
  varclr(&opt);

  opt.common_opt.conn_evt_ext.enable = 1; // enable Data Length Extension
  VERIFY_STATUS( sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt), false );

  // Init Peripheral role
  VERIFY( Periph.begin() );

  Security.begin();

  // Default device name
  ble_gap_conn_sec_mode_t sec_mode = BLE_SECMODE_OPEN;
  VERIFY_STATUS(sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) CFG_DEFAULT_NAME, strlen(CFG_DEFAULT_NAME)), false);

  // Init Central role
  if (_central_count)  Central.begin();

  // Create RTOS Semaphore & Task for BLE Event
  _ble_event_sem = xSemaphoreCreateBinary();
  VERIFY(_ble_event_sem);

  TaskHandle_t ble_task_hdl;
  xTaskCreate( adafruit_ble_task, "BLE", CFG_BLE_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &ble_task_hdl);

  // Create RTOS Semaphore & Task for SOC Event
  _soc_event_sem = xSemaphoreCreateBinary();
  VERIFY(_soc_event_sem);

  TaskHandle_t soc_task_hdl;
  xTaskCreate( adafruit_soc_task, "SOC", CFG_SOC_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &soc_task_hdl);

  NVIC_EnableIRQ(SD_EVT_IRQn); // enable SD interrupt

  // Create Timer for led advertising blinky
  _led_blink_th = xTimerCreate(NULL, ms2tick(CFG_ADV_BLINKY_INTERVAL/2), true, NULL, bluefruit_blinky_cb);

  // Initialize bonding
  bond_init();

  return true;
}

/*------------------------------------------------------------------*/
/* General Functions
 *------------------------------------------------------------------*/
ble_gap_addr_t AdafruitBluefruit::getAddr (void)
{
  ble_gap_addr_t gap_addr;
  sd_ble_gap_addr_get(&gap_addr);
  return gap_addr;
}

/**
 * Get current Mac address and its type
 * @param mac address
 * @return Address type e.g BLE_GAP_ADDR_TYPE_RANDOM_STATIC
 */
uint8_t AdafruitBluefruit::getAddr (uint8_t mac[6])
{
  ble_gap_addr_t gap_addr;
  sd_ble_gap_addr_get(&gap_addr);

  memcpy(mac, gap_addr.addr, 6);
  return gap_addr.addr_type;
}

bool AdafruitBluefruit::setAddr (ble_gap_addr_t* gap_addr)
{
  VERIFY_STATUS( sd_ble_gap_addr_set(gap_addr), false );
  return true;
}

void AdafruitBluefruit::setName (char const * str)
{
  ble_gap_conn_sec_mode_t sec_mode = BLE_SECMODE_OPEN;
  sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) str, strlen(str));
}

uint8_t AdafruitBluefruit::getName(char* name, uint16_t bufsize)
{
  VERIFY_STATUS( sd_ble_gap_device_name_get((uint8_t*) name, &bufsize), 0);
  return bufsize;
}


static inline bool is_tx_power_valid(int8_t power)
{
#if defined(NRF52832_XXAA)
  int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 3, 4 };
#elif defined( NRF52840_XXAA)
  int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 };
#endif

  for (uint32_t i=0; i<sizeof(accepted); i++)
  {
    if (accepted[i] == power) return true;
  }

  return false;
}

bool AdafruitBluefruit::setTxPower(int8_t power)
{
  VERIFY(is_tx_power_valid(power));
  _tx_power = power;
  return true;
}

int8_t AdafruitBluefruit::getTxPower(void)
{
  return _tx_power;
}

void AdafruitBluefruit::autoConnLed(bool enabled)
{
  _led_conn = enabled;
}

void AdafruitBluefruit::setConnLedInterval(uint32_t ms)
{
  BaseType_t active = xTimerIsTimerActive(_led_blink_th);
  xTimerChangePeriod(_led_blink_th, ms2tick(ms), 0);

  // Change period of inactive timer will also start it !!
  if ( !active ) xTimerStop(_led_blink_th, 0);
}

bool AdafruitBluefruit::setAppearance(uint16_t appear)
{
  return ERROR_NONE == sd_ble_gap_appearance_set(appear);
}

uint16_t AdafruitBluefruit::getAppearance(void)
{
  uint16_t appear = 0;
  (void) sd_ble_gap_appearance_get(&appear);
  return appear;
}

/*------------------------------------------------------------------*/
/* GAP, Connections and Bonding
 *------------------------------------------------------------------*/
uint8_t AdafruitBluefruit::connected(void)
{
  uint8_t count = 0;
  for (uint16_t c=0; c<BLE_MAX_CONNECTION; c++)
  {
    if ( this->connected(c) ) count++;
  }

  return count;
}

bool AdafruitBluefruit::connected(uint16_t conn_hdl)
{
  BLEConnection* conn = this->Connection(conn_hdl);
  return conn && conn->connected();
}

uint8_t AdafruitBluefruit::getConnectedHandles(uint16_t* hdl_list, uint8_t max_count)
{
  uint8_t count = 0;
  for (uint16_t hdl = 0; (hdl < BLE_MAX_CONNECTION) && (count < max_count); ++hdl)
  {
    if (this->connected(hdl))
    {
      hdl_list[count] = hdl;
      count++;
    }
  }

  return count;
}

bool AdafruitBluefruit::disconnect(uint16_t conn_hdl)
{
  BLEConnection* conn = this->Connection(conn_hdl);

  // disconnect if connected
  if ( conn && conn->connected() )
  {
    return conn->disconnect();
  }

  return true; // not connected still return true
}

void AdafruitBluefruit::setEventCallback (event_cb_t fp)
{
  _event_cb = fp;
}

uint16_t AdafruitBluefruit::connHandle(void)
{
  return _conn_hdl;
}

uint16_t AdafruitBluefruit::getMaxMtu(uint8_t role)
{
  return (role == BLE_GAP_ROLE_PERIPH) ? _sd_cfg.prph.mtu_max : _sd_cfg.central.mtu_max;
}

BLEConnection* AdafruitBluefruit::Connection(uint16_t conn_hdl)
{
  return (conn_hdl < BLE_MAX_CONNECTION) ? _connection[conn_hdl] : NULL;
}

void AdafruitBluefruit::setRssiCallback(rssi_cb_t fp)
{
  _rssi_cb = fp;
}


/*------------------------------------------------------------------*/
/* Thread & SoftDevice Event handler
 *------------------------------------------------------------------*/
extern "C" void SD_EVT_IRQHandler(void)
{
#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  // Notify both BLE & SOC & MultiProtocol (if any) Task
  xSemaphoreGiveFromISR(Bluefruit._soc_event_sem, NULL);
  xSemaphoreGiveFromISR(Bluefruit._ble_event_sem, NULL);

#ifdef ANT_LICENSE_KEY
  if (Bluefruit._mprot_event_sem)  xSemaphoreGiveFromISR(Bluefruit._mprot_event_sem, NULL);
#endif

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif
}

/**
 * Handle SOC event such as FLASH operation
 */
void adafruit_soc_task(void* arg)
{
  (void) arg;

  while (1)
  {
    if ( xSemaphoreTake(Bluefruit._soc_event_sem, portMAX_DELAY) )
    {
      uint32_t soc_evt;
      uint32_t err = ERROR_NONE;

      // until no more pending events
      while ( NRF_ERROR_NOT_FOUND != (err = sd_evt_get(&soc_evt)) )
      {
        if (ERROR_NONE == err)
        {
          switch (soc_evt)
          {
            // Flash
            case NRF_EVT_FLASH_OPERATION_SUCCESS:
            case NRF_EVT_FLASH_OPERATION_ERROR:
              LOG_LV1("SOC", "NRF_EVT_FLASH_OPERATION_%s", soc_evt == NRF_EVT_FLASH_OPERATION_SUCCESS ? "SUCCESS" : "ERROR");
              if ( flash_nrf5x_event_cb ) flash_nrf5x_event_cb(soc_evt);
            break;

            #ifdef USE_TINYUSB
            /*------------- usb power event handler -------------*/
            case NRF_EVT_POWER_USB_DETECTED:
            case NRF_EVT_POWER_USB_POWER_READY:
            case NRF_EVT_POWER_USB_REMOVED:
            {
              int32_t usbevt = (soc_evt == NRF_EVT_POWER_USB_DETECTED   ) ? NRFX_POWER_USB_EVT_DETECTED:
                               (soc_evt == NRF_EVT_POWER_USB_POWER_READY) ? NRFX_POWER_USB_EVT_READY   :
                               (soc_evt == NRF_EVT_POWER_USB_REMOVED    ) ? NRFX_POWER_USB_EVT_REMOVED : -1;

              if ( usbevt >= 0) tusb_hal_nrf_power_event(usbevt);
            }
            break;
            #endif

            default: break;
          }
        }
      }
    }
  }
}

/*------------------------------------------------------------------*/
/* BLE Event handler
 *------------------------------------------------------------------*/
void adafruit_ble_task(void* arg)
{
  (void) arg;

  // malloc buffered is algined by 4
  uint8_t * ev_buf = (uint8_t*) rtos_malloc(BLE_EVT_LEN_MAX(BLE_GATT_ATT_MTU_MAX));

  while (1)
  {
    if ( xSemaphoreTake(Bluefruit._ble_event_sem, portMAX_DELAY) )
    {
      uint32_t err = NRF_SUCCESS;

      // Until no pending events
      while( NRF_ERROR_NOT_FOUND != err )
      {
        uint16_t ev_len = BLE_EVT_LEN_MAX(BLE_GATT_ATT_MTU_MAX);

        // Get BLE Event
        err = sd_ble_evt_get(ev_buf, &ev_len);

        // Handle valid event
        if( NRF_SUCCESS == err)
        {
          Bluefruit._ble_handler( (ble_evt_t*) ev_buf );
        }else if ( NRF_ERROR_NOT_FOUND != err )
        {
          LOG_LV1("BLE", "SD event error %s", dbg_err_str(err));
        }
      }
    }
  }
}

/**
 * BLE event handler
 * @param evt event
 */
void AdafruitBluefruit::_ble_handler(ble_evt_t* evt)
{
  // conn handle has fixed offset for all events
  uint16_t const conn_hdl = evt->evt.common_evt.conn_handle;
  BLEConnection* conn = this->Connection(conn_hdl);

  LOG_LV2("BLE", "%s : Conn Handle = %d", dbg_ble_event_str(evt->header.evt_id), conn_hdl);

  // GAP handler
  if ( conn )
  {
    conn->_eventHandler(evt);
    Security._eventHandler(evt);
  }

  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      // Turn on Conn LED
      _stopConnLed();
      _setConnLed(true);

      ble_gap_evt_connected_t const * para = &evt->evt.gap_evt.params.connected;
      ble_gap_addr_t const* peer_addr = &para->peer_addr;

      (void) peer_addr;

      LOG_LV2("GAP", "MAC = %02X:%02X:%02X:%02X:%02X:%02X, Type = %d, Resolved = %d",
              peer_addr->addr[5], peer_addr->addr[4], peer_addr->addr[3], peer_addr->addr[2], peer_addr->addr[1], peer_addr->addr[0],
              peer_addr->addr_type, peer_addr->addr_id_peer);
      LOG_LV2("GAP", "Conn Interval = %.2f ms, Latency = %d, Supervisor Timeout = %d ms",
              para->conn_params.max_conn_interval*1.25f, para->conn_params.slave_latency, 10*para->conn_params.conn_sup_timeout);

      if ( _connection[conn_hdl] )
      {
        LOG_LV1("GAP", "Connection is already in used, something wrong !!");
        delete _connection[conn_hdl];
        _connection[conn_hdl] = NULL;
      }

      // Transmission buffer pool
      uint8_t const hvn_qsize = (para->role == BLE_GAP_ROLE_PERIPH) ? _sd_cfg.prph.hvn_qsize : _sd_cfg.central.hvn_qsize;
      uint8_t const wrcmd_qsize = (para->role == BLE_GAP_ROLE_PERIPH) ? _sd_cfg.prph.wrcmd_qsize : _sd_cfg.central.wrcmd_qsize;

      _connection[conn_hdl] = new BLEConnection(conn_hdl, para, hvn_qsize, wrcmd_qsize);
      conn = _connection[conn_hdl];

      // Invoke connect callback
      if ( conn->getRole() == BLE_GAP_ROLE_PERIPH )
      {
        if ( Periph._connect_cb ) ada_callback(NULL, 0, Periph._connect_cb, conn_hdl);
      }else
      {
        if ( Central._connect_cb ) ada_callback(NULL, 0, Central._connect_cb, conn_hdl);
      }
    }
    break;

    case BLE_GAP_EVT_DISCONNECTED:
    {
      ble_gap_evt_disconnected_t const* para = &evt->evt.gap_evt.params.disconnected;

      LOG_LV2("GAP", "Disconnect Reason: %s", dbg_hci_str(evt->evt.gap_evt.params.disconnected.reason));

      // Turn off Conn LED If not connected at all
      if ( !this->connected() ) _setConnLed(false);

      // Invoke disconnect callback
      if ( conn->getRole() == BLE_GAP_ROLE_PERIPH )
      {
        if ( Periph._disconnect_cb ) ada_callback(NULL, 0, Periph._disconnect_cb, conn_hdl, para->reason);
      }else
      {
        if ( Central._disconnect_cb ) ada_callback(NULL, 0, Central._disconnect_cb, conn_hdl, para->reason);
      }

      delete _connection[conn_hdl];
      _connection[conn_hdl] = NULL;
    }
    break;

    case BLE_GAP_EVT_RSSI_CHANGED:
    {
      ble_gap_evt_rssi_changed_t const * rssi_changed = &evt->evt.gap_evt.params.rssi_changed;
      if ( _rssi_cb )
      {
         ada_callback(NULL, 0, _rssi_cb, conn_hdl, rssi_changed->rssi);
      }
    }
    break;

    case BLE_EVT_USER_MEM_REQUEST:
      // We will handle Long Write sequence (RW Authorize PREP_WRITE_REQ)
      sd_ble_user_mem_reply(conn_hdl, NULL);
    break;

    case BLE_EVT_USER_MEM_RELEASE:
      // nothing to do
    break;

    default: break;
  }

  Advertising._eventHandler(evt);
  Scanner._eventHandler(evt);

  /*------------- BLE Peripheral Events -------------*/
  /* Only handle Peripheral events with matched connection handle
   * or a few special one
   * - Connected event
   * - Advertising timeout (could be connected and advertising at the same time)
   */
  if ( conn_hdl       == _conn_hdl             ||
       evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
  {
    switch ( evt->header.evt_id  )
    {
      case BLE_GAP_EVT_CONNECTED:
      {
        ble_gap_evt_connected_t* para = &evt->evt.gap_evt.params.connected;

        if (para->role == BLE_GAP_ROLE_PERIPH)
        {
          _conn_hdl = evt->evt.gap_evt.conn_handle;
        }
      }
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        _conn_hdl = BLE_CONN_HANDLE_INVALID;
      break;

      default: break;
    }
  }

  // Periph event handler (also handle generic non-connection event)
  if ( (conn == NULL) || (conn->getRole() == BLE_GAP_ROLE_PERIPH) )
  {
    Periph._eventHandler(evt);
  }

  // Central Event Handler (also handle generic non-connection event)
  if ( (conn == NULL) || (conn->getRole() == BLE_GAP_ROLE_CENTRAL) )
  {
    Central._eventHandler(evt);
  }

  // Discovery Event Handler
  if ( Discovery.begun() ) Discovery._eventHandler(evt);

  // GATTs characteristics event handler
  Gatt._eventHandler(evt);

  // User callback if set
  if (_event_cb) _event_cb(evt);
}

/*------------------------------------------------------------------*/
/* Internal Connection LED
 *------------------------------------------------------------------*/
void AdafruitBluefruit::_startConnLed(void)
{
  if (_led_conn) xTimerStart(_led_blink_th, 0);
}

void AdafruitBluefruit::_stopConnLed(void)
{
  xTimerStop(_led_blink_th, 0);

  _setConnLed( this->connected() );
}

void AdafruitBluefruit::_setConnLed (bool on_off)
{
  if (_led_conn)
  {
    digitalWrite(LED_BLUE, on_off ? LED_STATE_ON : (1-LED_STATE_ON) );
  }
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void Bluefruit_printInfo(void)
{
  Bluefruit.printInfo();
}

void AdafruitBluefruit::printInfo(void)
{
  // Skip if Serial is not initialized
  if ( !Serial ) return;
  // prepare for ability to change output, based on compile-time flags
  Print& logger = Serial;

  // Skip if Bluefruit.begin() is not called
  if ( _ble_event_sem == NULL ) return;

  logger.println("--------- SoftDevice Config ---------");

  char const * title_fmt = "%-16s: ";

  /*------------- SoftDevice Config -------------*/
  // Max uuid128
  logger.printf(title_fmt, "Max UUID128");
  logger.println(_sd_cfg.uuid128_max);

  // ATTR Table Size
  logger.printf(title_fmt, "ATTR Table Size");
  logger.println(_sd_cfg.attr_table_size);

  // Service Changed
  logger.printf(title_fmt, "Service Changed");
  logger.println(_sd_cfg.service_changed);

  if ( _prph_count )
  {
    logger.println("Peripheral Connect Setting");

    logger.print("  - ");
    logger.printf(title_fmt, "Max MTU");
    logger.println(_sd_cfg.prph.mtu_max);

    logger.print("  - ");
    logger.printf(title_fmt, "Event Length");
    logger.println(_sd_cfg.prph.event_len);

    logger.print("  - ");
    logger.printf(title_fmt, "HVN Queue Size");
    logger.println(_sd_cfg.prph.hvn_qsize);

    logger.print("  - ");
    logger.printf(title_fmt, "WrCmd Queue Size");
    logger.println(_sd_cfg.prph.wrcmd_qsize);
  }

  if ( _central_count )
  {
    logger.println("Central Connect Setting");

    logger.print("  - ");
    logger.printf(title_fmt, "Max MTU");
    logger.println(_sd_cfg.central.mtu_max);

    logger.print("  - ");
    logger.printf(title_fmt, "Event Length");
    logger.println(_sd_cfg.central.event_len);

    logger.print("  - ");
    logger.printf(title_fmt, "HVN Queue Size");
    logger.println(_sd_cfg.central.hvn_qsize);

    logger.print("  - ");
    logger.printf(title_fmt, "WrCmd Queue Size");
    logger.println(_sd_cfg.central.wrcmd_qsize);
  }

  /*------------- Settings -------------*/
  logger.println("\n--------- BLE Settings ---------");
  // Name
  logger.printf(title_fmt, "Name");
  {
    char name[32];
    memclr(name, sizeof(name));
    getName(name, sizeof(name));
    logger.printf(name);
  }
  logger.println();

  // Max Connections
  logger.printf(title_fmt, "Max Connections");
  logger.printf("Peripheral = %d, ", _prph_count);
  logger.printf("Central = %d ", _central_count);
  logger.println();

  // Address
  logger.printf(title_fmt, "Address");
  {
    char const * type_str[] = { "Public", "Static", "Private Resolvable", "Private Non Resolvable" };
    ble_gap_addr_t gap_addr = this->getAddr();

    // MAC is in little endian --> print reverse
    logger.printBufferReverse(gap_addr.addr, 6, ':');
    logger.printf(" (%s)", type_str[gap_addr.addr_type]);
  }
  logger.println();

  // Tx Power
  logger.printf(title_fmt, "TX Power");
  logger.printf("%d dBm", _tx_power);
  logger.println();

  Periph.printInfo();

  /*------------- List the paired devices -------------*/
  if ( _prph_count )
  {
    logger.printf(title_fmt, "Peripheral Paired Devices");
    logger.println();
    bond_print_list(BLE_GAP_ROLE_PERIPH);
  }

  if ( _central_count )
  {
    logger.printf(title_fmt, "Central Paired Devices");
    logger.println();
    bond_print_list(BLE_GAP_ROLE_CENTRAL);
  }

  logger.println();
}
