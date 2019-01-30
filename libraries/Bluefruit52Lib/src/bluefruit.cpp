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

#ifdef NRF52840_XXAA
#include "nrfx_power.h"
#include "usb/usb.h"
#endif

#define CFG_BLE_TX_POWER_LEVEL           0
#define CFG_DEFAULT_NAME                 "Bluefruit52"

#define CFG_BLE_TASK_STACKSIZE          (512*3)
#define CFG_SOC_TASK_STACKSIZE          (200)

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

#if CFG_DEBUG
  while(1)
  {

  }
#endif
}

/**
 * Constructor
 */
AdafruitBluefruit::AdafruitBluefruit(void)
  : Central(), Gap()
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

  _sd_cfg.attr_table_size = 0x800;
  _sd_cfg.uuid128_max     = BLE_UUID_VS_COUNT_DEFAULT;
  _sd_cfg.service_changed = 1;

  _prph_count    = 0;
  _central_count = 0;

  _ble_event_sem = NULL;
  _soc_event_sem = NULL;

  _led_blink_th  = NULL;
  _led_conn      = true;

  _tx_power      = CFG_BLE_TX_POWER_LEVEL;

  _conn_hdl      = BLE_CONN_HANDLE_INVALID;

  _ppcp.min_conn_interval = BLE_GAP_CONN_MIN_INTERVAL_DFLT;
  _ppcp.max_conn_interval = BLE_GAP_CONN_MAX_INTERVAL_DFLT;
  _ppcp.slave_latency = 0;
  _ppcp.conn_sup_timeout = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10; // in 10ms unit

  _conn_interval = 0;

  _connect_cb    = NULL;
  _disconnect_cb = NULL;
  _event_cb = NULL;

COMMENT_OUT(
  _auth_type = BLE_GAP_AUTH_KEY_TYPE_NONE;
  varclr(_pin);
)
}

void AdafruitBluefruit::configServiceChanged(bool changed)
{
  _sd_cfg.service_changed = (changed ? 1 : 0);
}

void AdafruitBluefruit::configUuid128Count(uint8_t  uuid128_max)
{
  _sd_cfg.uuid128_max = uuid128_max;
}

void AdafruitBluefruit::configAttrTableSize(uint32_t attr_table_size)
{
  _sd_cfg.attr_table_size = align4( maxof(attr_table_size, BLE_GATTS_ATTR_TAB_SIZE_MIN) );
}

void AdafruitBluefruit::configPrphConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  Gap.configPrphConn(mtu_max, event_len, hvn_qsize, wrcmd_qsize);
}

void AdafruitBluefruit::configCentralConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  Gap.configCentralConn(mtu_max, event_len, hvn_qsize, wrcmd_qsize);
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
      configPrphConn(247, 6, 3, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
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


err_t AdafruitBluefruit::begin(uint8_t prph_count, uint8_t central_count)
{
  _prph_count    = prph_count;
  _central_count = central_count;

#ifdef NRF52840_XXAA
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

  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, nrf_error_cb) );

#ifdef NRF52840_XXAA
  usb_softdevice_post_enable();
#endif

  /*------------------------------------------------------------------*/
  /*  SoftDevice Default Configuration depending on the number of
   * prph and central connections for optimal SRAM usage.
   *
   * - If Peripheral mode is enabled
   *   - ATTR Table Size          = 0x800.
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
//  if ( _prph_count )
//  {
//    // If not configured by user, set Attr Table Size large enough for
//    // most peripheral applications
//    if ( _sd_cfg.attr_table_size == 0 ) _sd_cfg.attr_table_size = 0x800;
//  }
//
//  if ( _central_count)
//  {
//
//  }
//
//  // Not configure, default value are used
//  if ( _sd_cfg.attr_table_size == 0 ) _sd_cfg.attr_table_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;

  /*------------- Configure BLE params  -------------*/
  extern uint32_t  __data_start__[]; // defined in linker
  uint32_t ram_start = (uint32_t) __data_start__;

  ble_cfg_t blecfg;

  // Vendor UUID count
  varclr(&blecfg);
  blecfg.common_cfg.vs_uuid_cfg.vs_uuid_count = _sd_cfg.uuid128_max;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &blecfg, ram_start) );

  // Roles
  varclr(&blecfg);
  blecfg.gap_cfg.role_count_cfg.periph_role_count  = _prph_count;
  blecfg.gap_cfg.role_count_cfg.central_role_count = _central_count; // ? BLE_CENTRAL_MAX_CONN : 0);
  blecfg.gap_cfg.role_count_cfg.central_sec_count  = (_central_count ? 1 : 0); // should be enough
  VERIFY_STATUS( sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &blecfg, ram_start) );

  // Device Name
//  varclr(&blecfg);
//  blecfg.gap_cfg.device_name_cfg =
//  VERIFY_STATUS( sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &blecfg, ram_start) );

  varclr(&blecfg);
  blecfg.gatts_cfg.service_changed.service_changed = _sd_cfg.service_changed;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_GATTS_CFG_SERVICE_CHANGED, &blecfg, ram_start) );

  // ATTR Table Size
  varclr(&blecfg);
  blecfg.gatts_cfg.attr_tab_size.attr_tab_size = _sd_cfg.attr_table_size;
  VERIFY_STATUS ( sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &blecfg, ram_start) );

  /*------------- Event Length + MTU + HVN queue + WRITE CMD queue setting affecting bandwidth -------------*/
  if ( _prph_count )
  {
    // ATT MTU
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = Gap._cfg_prph.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start) );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _prph_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length = Gap._cfg_prph.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start) );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = Gap._cfg_prph.hvn_tx_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start) );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = Gap._cfg_prph.wr_cmd_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &blecfg, ram_start) );
  }

  if ( _central_count)
  {
    // ATT MTU
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = Gap._cfg_central.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start) );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _central_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length = Gap._cfg_central.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start) );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = Gap._cfg_central.hvn_tx_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start) );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = Gap._cfg_central.wr_cmd_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &blecfg, ram_start) );
  }

  // Enable BLE stack
  // The memory requirement for a specific configuration will not increase
  // between SoftDevices with the same major version number
  uint32_t err = sd_ble_enable(&ram_start);
  if ( err )
  {
    LOG_LV1("CFG", "SoftDevice config require more SRAM than provided by linker.\n"
                 "App Ram Start must be at least 0x%08X (provided 0x%08X)\n"
                 "Please update linker file or re-config SoftDevice", ram_start, (uint32_t) __data_start__);
  }

  LOG_LV1("CFG", "SoftDevice's RAM requires: 0x%08X", ram_start);
  VERIFY_STATUS(err);

  /*------------- Configure BLE Option -------------*/
  ble_opt_t  opt;
  varclr(&opt);

  opt.common_opt.conn_evt_ext.enable = 1; // enable Data Length Extension
  VERIFY_STATUS( sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt) );

  /*------------- Configure GAP  -------------*/

  // Peripheral Preferred Connection Parameters
  VERIFY_STATUS( sd_ble_gap_ppcp_set(&_ppcp) );

  // Default device name
  ble_gap_conn_sec_mode_t sec_mode = BLE_SECMODE_OPEN;
  VERIFY_STATUS(sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) CFG_DEFAULT_NAME, strlen(CFG_DEFAULT_NAME)));

  VERIFY_STATUS( sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN) );

  //------------- USB -------------//
#if NRF52840_XXAA
  sd_power_usbdetected_enable(true);
  sd_power_usbpwrrdy_enable(true);
  sd_power_usbremoved_enable(true);
#endif

  /*------------- DFU OTA as built-in service -------------*/
  _dfu_svc.begin();

  if (_central_count)  Central.begin(); // Init Central

  // Create RTOS Semaphore & Task for BLE Event
  _ble_event_sem = xSemaphoreCreateBinary();
  VERIFY(_ble_event_sem, NRF_ERROR_NO_MEM);

  TaskHandle_t ble_task_hdl;
  xTaskCreate( adafruit_ble_task, "BLE", CFG_BLE_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &ble_task_hdl);

  // Create RTOS Semaphore & Task for SOC Event
  _soc_event_sem = xSemaphoreCreateBinary();
  VERIFY(_soc_event_sem, NRF_ERROR_NO_MEM);

  TaskHandle_t soc_task_hdl;
  xTaskCreate( adafruit_soc_task, "SOC", CFG_SOC_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &soc_task_hdl);

  // Interrupt priority has already been set by the stack.
//  NVIC_SetPriority(SD_EVT_IRQn, 6);
  NVIC_EnableIRQ(SD_EVT_IRQn);

  // Create Timer for led advertising blinky
  _led_blink_th = xTimerCreate(NULL, ms2tick(CFG_ADV_BLINKY_INTERVAL/2), true, NULL, bluefruit_blinky_cb);

  // Initialize bonding
  bond_init();

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* General Functions
 *------------------------------------------------------------------*/
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

bool AdafruitBluefruit::setTxPower(int8_t power)
{
#if defined(NRF52832_XXAA)
int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 3, 4 };
#elif defined( NRF52840_XXAA)
int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 };
#endif

  // Check if TX Power is valid value
  uint32_t i;
  for (i=0; i<sizeof(accepted); i++)
  {
    if (accepted[i] == power) break;
  }
  VERIFY(i < sizeof(accepted));

  // Apply if connected
  if ( _conn_hdl != BLE_CONN_HANDLE_INVALID )
  {
    VERIFY_STATUS( sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, _conn_hdl, power), false );
  }
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

bool AdafruitBluefruit::setApperance(uint16_t appear)
{
  return ERROR_NONE == sd_ble_gap_appearance_set(appear);
}

uint16_t AdafruitBluefruit::getApperance(void)
{
  uint16_t appear = 0;
  (void) sd_ble_gap_appearance_get(&appear);
  return appear;
}

bool AdafruitBluefruit::setPhy(int8_t phy)
{
#if defined(NRF52832_XXAA)
int8_t const accepted[] = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_1MBPS, BLE_GAP_PHY_2MBPS };
#elif defined( NRF52840_XXAA)
int8_t const accepted[] = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_1MBPS, BLE_GAP_PHY_2MBPS,
                            BLE_GAP_PHY_CODED };
#endif

  // Check if phy is valid value
  uint32_t i;
  for (i=0; i<sizeof(accepted); i++)
  {
    if (accepted[i] == phy) break;
  }
  VERIFY(i < sizeof(accepted));

  // Apply if connected
  ble_gap_phys_t phys;
  phys.tx_phys = phy;
  phys.rx_phys = phy;
  if ( _conn_hdl != BLE_CONN_HANDLE_INVALID )
  {
    VERIFY_STATUS( sd_ble_gap_phy_update(_conn_hdl, &phys), false );
  }
  _phy = phy;

  return true;
}

int8_t AdafruitBluefruit::getPhy(void)
{
  return _phy;
}

/*------------------------------------------------------------------*/
/* GAP, Connections and Bonding
 *------------------------------------------------------------------*/

bool AdafruitBluefruit::connected(void)
{
  return ( _conn_hdl != BLE_CONN_HANDLE_INVALID );
}

bool AdafruitBluefruit::disconnect(void)
{
  // disconnect if connected
  if ( connected() )
  {
    return ERROR_NONE == sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }

  return true; // not connected still return true
}

bool AdafruitBluefruit::setConnInterval(uint16_t min, uint16_t max)
{
  _ppcp.min_conn_interval = min;
  _ppcp.max_conn_interval = max;

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&_ppcp), false);

  return true;
}

bool AdafruitBluefruit::setConnIntervalMS(uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}

bool AdafruitBluefruit::setConnSupervisionTimeout(uint16_t timeout)
{
  _ppcp.conn_sup_timeout = timeout;

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&_ppcp), false);

  return true;
}

bool AdafruitBluefruit::setConnSupervisionTimeoutMS(uint16_t timeout_ms)
{
  return setConnSupervisionTimeout(timeout_ms / 10); // 10ms unit
}

void AdafruitBluefruit::setConnectCallback( BLEGap::connect_callback_t fp )
{
  _connect_cb = fp;
}

void AdafruitBluefruit::setDisconnectCallback( BLEGap::disconnect_callback_t fp )
{
  _disconnect_cb = fp;
}

void AdafruitBluefruit::setEventCallback ( void (*fp) (ble_evt_t*) )
{
  _event_cb = fp;
}

uint16_t AdafruitBluefruit::connHandle(void)
{
  return _conn_hdl;
}

bool AdafruitBluefruit::connPaired(void)
{
  return ( _conn_hdl != BLE_CONN_HANDLE_INVALID ) && Gap.paired(_conn_hdl);
}

uint16_t AdafruitBluefruit::connInterval(void)
{
  return _conn_interval;
}

ble_gap_addr_t AdafruitBluefruit::getPeerAddr(void)
{
  return Gap.getPeerAddr(_conn_hdl);
}

uint8_t AdafruitBluefruit::getPeerAddr(uint8_t addr[6])
{
  return Gap.getPeerAddr(_conn_hdl, addr);
}

COMMENT_OUT (
bool AdafruitBluefruit::setPIN(const char* pin)
{
  VERIFY ( strlen(pin) == BLE_GAP_PASSKEY_LEN );

  _auth_type = BLE_GAP_AUTH_KEY_TYPE_PASSKEY;
  memcpy(_pin, pin, BLE_GAP_PASSKEY_LEN);

// Config Static Passkey
//  ble_opt_t opt
//	uint8_t passkey[] = STATIC_PASSKEY;
//	m_static_pin_option.gap.passkey.p_passkey = passkey;
//err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_static_pin_option);

  return true;
}
)

/*------------------------------------------------------------------*/
/* Thread & SoftDevice Event handler
 *------------------------------------------------------------------*/
void SD_EVT_IRQHandler(void)
{
  // Notify both BLE & SOC Task
  xSemaphoreGiveFromISR(Bluefruit._soc_event_sem, NULL);
  xSemaphoreGiveFromISR(Bluefruit._ble_event_sem, NULL);
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

            #ifdef NRF52840_XXAA
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

  uint8_t * ev_buf = (uint8_t*) rtos_malloc(BLE_EVT_LEN_MAX(BLE_GATT_ATT_MTU_MAX));

  while (1)
  {
    if ( xSemaphoreTake(Bluefruit._ble_event_sem, portMAX_DELAY) )
    {
      uint32_t err = ERROR_NONE;

      // Until no pending events
      while( NRF_ERROR_NOT_FOUND != err )
      {
        uint16_t ev_len = BLE_EVT_LEN_MAX(BLE_GATT_ATT_MTU_MAX);

        // Get BLE Event
        err = sd_ble_evt_get(ev_buf, &ev_len);

        // Handle valid event, ignore error
        if( ERROR_NONE == err)
        {
          Bluefruit._ble_handler( (ble_evt_t*) ev_buf );
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
  // conn handle has fixed offset regardless of event type
  uint16_t const evt_conn_hdl = evt->evt.common_evt.conn_handle;

  LOG_LV1("BLE", "%s : Conn Handle = %d", dbg_ble_event_str(evt->header.evt_id), evt_conn_hdl);

  // GAP handler
  Gap._eventHandler(evt);
  Advertising._eventHandler(evt);
  Scanner._eventHandler(evt);

  /*------------- BLE Peripheral Events -------------*/
  /* Only handle Peripheral events with matched connection handle
   * or a few special one
   * - Connected event
   * - Advertising timeout (could be connected and advertising at the same time)
   *
   * Pairing procedure
   * - Connect -> SEC_PARAMS_REQUEST -> CONN_SEC_UPDATE -> AUTH_STATUS
   *
   * Reconnect to a paired device
   * - Connect -> SEC_INFO_REQUEST -> CONN_SEC_UPDATE
   */
  if ( evt_conn_hdl       == _conn_hdl             ||
       evt->header.evt_id == BLE_GAP_EVT_CONNECTED ||
       evt->header.evt_id == BLE_GAP_EVT_TIMEOUT )
  {
    switch ( evt->header.evt_id  )
    {
      case BLE_GAP_EVT_CONNECTED:
      { // Note callback is invoked by BLEGap
        ble_gap_evt_connected_t* para = &evt->evt.gap_evt.params.connected;

        if (para->role == BLE_GAP_ROLE_PERIPH)
        {
          _conn_hdl      = evt->evt.gap_evt.conn_handle;
          _conn_interval = para->conn_params.min_conn_interval;

          // Connection interval set by Central is out of preferred range
          // Try to negotiate with Central using our preferred values
          if ( !is_within(_ppcp.min_conn_interval, para->conn_params.min_conn_interval, _ppcp.max_conn_interval) )
          {
            // Null, value is set by sd_ble_gap_ppcp_set will be used
            VERIFY_STATUS( sd_ble_gap_conn_param_update(_conn_hdl, NULL), );
          }

          if (_connect_cb) ada_callback(NULL, _connect_cb, _conn_hdl);
        }
      }
      break;

      case BLE_GAP_EVT_CONN_PARAM_UPDATE:
      {
        // Connection Parameter after negotiating with Central
        // min conn = max conn = actual used interval
        ble_gap_conn_params_t* param = &evt->evt.gap_evt.params.conn_param_update.conn_params;
        _conn_interval = param->min_conn_interval;
      }
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        if (_disconnect_cb) ada_callback(NULL, _disconnect_cb, _conn_hdl, evt->evt.gap_evt.params.disconnected.reason);

        LOG_LV2("GAP", "Disconnect Reason 0x%02X", evt->evt.gap_evt.params.disconnected.reason);

        _conn_hdl = BLE_CONN_HANDLE_INVALID;
      break;

      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
      break;

      default: break;
    }
  }

  // Central Event Handler
  if (_central_count)
  {
    // Skip if not central connection
    if (evt_conn_hdl != _conn_hdl ||
        evt_conn_hdl == BLE_CONN_HANDLE_INVALID)
    {
      Central._event_handler(evt);
    }
  }

  // Discovery Event Handler
  if ( Discovery.begun() ) Discovery._event_handler(evt);

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
}

void AdafruitBluefruit::_setConnLed (bool on_off)
{
  if (_led_conn)
  {
    digitalWrite(LED_BLUE, on_off ? LED_STATE_ON : (1-LED_STATE_ON) );
  }
}

/*------------------------------------------------------------------*/
/* Bonds
 *------------------------------------------------------------------*/
bool AdafruitBluefruit::requestPairing(void)
{
  return Gap.requestPairing(_conn_hdl);
}

void AdafruitBluefruit::clearBonds(void)
{
  bond_clear_prph();
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
  // Skip if Serial is not initialised
  if ( !Serial ) return;

  // Skip if Bluefruit.begin() is not called
  if ( _ble_event_sem == NULL ) return;

  Serial.println("--------- SoftDevice Config ---------");

  char const * title_fmt = "%-16s: ";

  /*------------- SoftDevice Config -------------*/
  // Max uuid128
  Serial.printf(title_fmt, "Max UUID128");
  Serial.println(_sd_cfg.uuid128_max);

  // ATTR Table Size
  Serial.printf(title_fmt, "ATTR Table Size");
  Serial.println(_sd_cfg.attr_table_size);

  // Service Changed
  Serial.printf(title_fmt, "Service Changed");
  Serial.println(_sd_cfg.service_changed);

  if ( _prph_count )
  {
    Serial.println("Peripheral Connect Setting");

    Serial.print("  - ");
    Serial.printf(title_fmt, "Max MTU");
    Serial.println(Gap._cfg_prph.mtu_max);

    Serial.print("  - ");
    Serial.printf(title_fmt, "Event Length");
    Serial.println(Gap._cfg_prph.event_len);

    Serial.print("  - ");
    Serial.printf(title_fmt, "HVN Queue Size");
    Serial.println(Gap._cfg_prph.hvn_tx_qsize);

    Serial.print("  - ");
    Serial.printf(title_fmt, "WrCmd Queue Size");
    Serial.println(Gap._cfg_prph.wr_cmd_qsize);
  }

  if ( _central_count )
  {
    Serial.println("Central Connect Setting");

    Serial.print("  - ");
    Serial.printf(title_fmt, "Max MTU");
    Serial.println(Gap._cfg_central.mtu_max);

    Serial.print("  - ");
    Serial.printf(title_fmt, "Event Length");
    Serial.println(Gap._cfg_central.event_len);

    Serial.print("  - ");
    Serial.printf(title_fmt, "HVN Queue Size");
    Serial.println(Gap._cfg_central.hvn_tx_qsize);

    Serial.print("  - ");
    Serial.printf(title_fmt, "WrCmd Queue Size");
    Serial.println(Gap._cfg_central.wr_cmd_qsize);
  }

  /*------------- Settings -------------*/
  Serial.println("\n--------- BLE Settings ---------");
  // Name
  Serial.printf(title_fmt, "Name");
  {
    char name[32];
    memclr(name, sizeof(name));
    getName(name, sizeof(name));
    Serial.printf(name);
  }
  Serial.println();

  // Max Connections
  Serial.printf(title_fmt, "Max Connections");
  Serial.printf("Peripheral = %d, ", _prph_count ? 1 : 0);
  Serial.printf("Central = %d ", _central_count ? BLE_CENTRAL_MAX_CONN : 0);
  Serial.println();

  // Address
  Serial.printf(title_fmt, "Address");
  {
    char const * type_str[] = { "Public", "Static", "Private Resolvable", "Private Non Resolvable" };
    uint8_t mac[6];
    uint8_t type = Gap.getAddr(mac);

    // MAC is in little endian --> print reverse
    Serial.printBufferReverse(mac, 6, ':');
    Serial.printf(" (%s)", type_str[type]);
  }
  Serial.println();

  // Tx Power
  Serial.printf(title_fmt, "TX Power");
  Serial.printf("%d dBm", _tx_power);
  Serial.println();

  // Connection Intervals
  Serial.printf(title_fmt, "Conn Intervals");
  Serial.printf("min = %.2f ms, ", _ppcp.min_conn_interval*1.25f);
  Serial.printf("max = %.2f ms", _ppcp.max_conn_interval*1.25f);
  Serial.println();

  Serial.printf(title_fmt, "Conn Timeout");
  Serial.printf("%.2f ms", _ppcp.conn_sup_timeout*10.0f);
  Serial.println();

  /*------------- List the paried device -------------*/
  if ( _prph_count )
  {
    Serial.printf(title_fmt, "Peripheral Paired Devices");
    Serial.println();
    bond_print_list(BLE_GAP_ROLE_PERIPH);
  }

  if ( _central_count )
  {
    Serial.printf(title_fmt, "Central Paired Devices");
    Serial.println();
    bond_print_list(BLE_GAP_ROLE_CENTRAL);
  }

  Serial.println();
}
