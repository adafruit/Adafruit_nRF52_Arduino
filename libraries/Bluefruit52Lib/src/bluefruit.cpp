/**************************************************************************/
/*!
    @file     bluefruit.cpp
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

#include "bluefruit.h"
#include <Nffs.h>

#define SVC_CONTEXT_FLAG                 (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)

#define CFG_BLE_TX_POWER_LEVEL           0
#define CFG_DEFAULT_NAME                 "Bluefruit52"


#define CFG_BLE_TASK_STACKSIZE          (512*3)
#define CFG_SOC_TASK_STACKSIZE          (200)

#define CFG_BOND_NFFS_DIR                "/adafruit/bond"
#define BOND_FILENAME                    CFG_BOND_NFFS_DIR "/%04x"
#define BOND_FILENAME_LEN                (sizeof(CFG_BOND_NFFS_DIR) + 10)

AdafruitBluefruit Bluefruit;

/*------------------------------------------------------------------*/
/* PROTOTYPTES
 *------------------------------------------------------------------*/
extern "C"
{
  void hal_flash_event_cb(uint32_t event) ATTR_WEAK;
}

void adafruit_ble_task(void* arg);
void adafruit_soc_task(void* arg);

void _adafruit_save_bond_key_dfr(uint32_t conn_handle);

#if CFG_DEBUG >= 2
#define printBondDir()    dbgPrintDir(CFG_BOND_NFFS_DIR)
#else
#define printBondDir()
#endif

/*------------------------------------------------------------------*/
/* INTERNAL FUNCTION
 *------------------------------------------------------------------*/
static void bluefruit_blinky_cb( TimerHandle_t xTimer )
{
  (void) xTimer;
  ledToggle(LED_BLUE);
}


static void nrf_error_cb(uint32_t id, uint32_t pc, uint32_t info)
{
  PRINT_INT(id);
  PRINT_HEX(pc);
  PRINT_HEX(info);
}

/**
 * Constructor
 */
AdafruitBluefruit::AdafruitBluefruit(void)
  : Central()
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
  _sd_cfg.service_changed = 0;

  _sd_cfg.prph.mtu_max    = BLEGATT_ATT_MTU_MAX;
  _sd_cfg.central.mtu_max = BLE_GATT_ATT_MTU_DEFAULT;

#if SD_VER >= 500
  _sd_cfg.prph.event_len    = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _sd_cfg.prph.hvn_tx_qsize = 3;
  _sd_cfg.prph.wr_cmd_qsize = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;

  _sd_cfg.central.event_len    = BLE_GAP_EVENT_LENGTH_DEFAULT;
  _sd_cfg.central.hvn_tx_qsize = BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT;
  _sd_cfg.central.wr_cmd_qsize = BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT;
#endif


  _prph_count    = 0;
  _central_count = 0;

  _ble_event_sem    = NULL;
  _soc_event_sem    = NULL;

  _led_blink_th     = NULL;
  _led_conn         = true;

  _tx_power  = 0;

  _conn_hdl  = BLE_CONN_HANDLE_INVALID;
  _bonded    = false;

  _ppcp_min_conn = BLE_GAP_CONN_MIN_INTERVAL_DFLT;
  _ppcp_max_conn = BLE_GAP_CONN_MAX_INTERVAL_DFLT;
  _conn_interval = 0;

  _connect_cb    = NULL;
  _disconnect_cb = NULL;

COMMENT_OUT(
  _auth_type = BLE_GAP_AUTH_KEY_TYPE_NONE;
  varclr(_pin);
)

  varclr(&_bond_data);
  _bond_data.own_enc.master_id.ediv = 0xFFFF; // invalid value for ediv

  _sec_param = (ble_gap_sec_params_t)
              {
                .bond         = 1,
                .mitm         = 0,
                .lesc         = 0,
                .keypress     = 0,
                .io_caps      = BLE_GAP_IO_CAPS_NONE,
                .oob          = 0,
                .min_key_size = 7,
                .max_key_size = 16,
                .kdist_own    = { .enc = 1, .id = 1},
                .kdist_peer   = { .enc = 1, .id = 1},
              };
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
  _sd_cfg.attr_table_size = maxof(attr_table_size, BLE_GATTS_ATTR_TAB_SIZE_MIN);
}

void AdafruitBluefruit::configPrphConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _sd_cfg.prph.mtu_max = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
#if SD_VER >= 500
  _sd_cfg.prph.event_len = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
#endif
  _sd_cfg.prph.hvn_tx_qsize = hvn_qsize;
  _sd_cfg.prph.wr_cmd_qsize = wrcmd_qsize;
}

void AdafruitBluefruit::configCentralConn(uint16_t mtu_max, uint8_t event_len, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _sd_cfg.central.mtu_max = maxof(mtu_max, BLE_GATT_ATT_MTU_DEFAULT);
#if SD_VER >= 500
  _sd_cfg.central.event_len = maxof(event_len, BLE_GAP_EVENT_LENGTH_MIN);
#endif
  _sd_cfg.central.hvn_tx_qsize = hvn_qsize;
  _sd_cfg.central.wr_cmd_qsize = wrcmd_qsize;
}

uint16_t AdafruitBluefruit::getMaxMtu(void)
{
  return _sd_cfg.prph.mtu_max;
}

uint8_t AdafruitBluefruit::getHvnTxQueue(void)
{
  return _sd_cfg.prph.hvn_tx_qsize;
}

uint8_t AdafruitBluefruit::getWriteCmdQueue(void)
{
  return _sd_cfg.prph.wr_cmd_qsize;
}


err_t AdafruitBluefruit::begin(uint8_t prph_count, uint8_t central_count)
{
  _prph_count    = prph_count;
  _central_count = central_count;

  // Configure Clock
#if defined( USE_LFXO )
  nrf_clock_lf_cfg_t clock_cfg =
  {
      // LFXO
      .source        = NRF_CLOCK_LF_SRC_XTAL,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      #if SD_VER < 500
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
      #else
      .accuracy      = NRF_CLOCK_LF_ACCURACY_20_PPM
      #endif
  };
#endif

  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, nrf_error_cb) );

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

#if SD_VER < 500
  // Configure BLE params & ATTR Size
  ble_enable_params_t params =
  {
      .common_enable_params = { .vs_uuid_count = _sd_cfg.uuid128_max },
      .gap_enable_params = {
          .periph_conn_count  = (uint8_t) (_prph_count    ? 1 : 0),
          .central_conn_count = (uint8_t) (_central_count ? BLE_CENTRAL_MAX_CONN : 0),
          .central_sec_count  = (uint8_t) (_central_count ? BLE_CENTRAL_MAX_SECURE_CONN : 0),
      },
      .gatts_enable_params = {
          .service_changed = 1,
          .attr_tab_size   = _sd_cfg.attr_table_size
      }
  };

  // Enable BLE stack
  VERIFY_STATUS( sd_ble_enable(&params, &ram_start) );
#else

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
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = _sd_cfg.prph.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start) );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _prph_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length = _sd_cfg.prph.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start) );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = _sd_cfg.prph.hvn_tx_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start) );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_PERIPHERAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = _sd_cfg.prph.wr_cmd_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &blecfg, ram_start) );
  }

  if ( _central_count)
  {
    // ATT MTU
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatt_conn_cfg.att_mtu = _sd_cfg.central.mtu_max;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATT, &blecfg, ram_start) );

    // Event length and max connection for this config
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gap_conn_cfg.conn_count   = _central_count;
    blecfg.conn_cfg.params.gap_conn_cfg.event_length = _sd_cfg.central.event_len;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GAP, &blecfg, ram_start) );

    // HVN queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = _sd_cfg.central.hvn_tx_qsize;
    VERIFY_STATUS ( sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &blecfg, ram_start) );

    // WRITE COMMAND queue size
    varclr(&blecfg);
    blecfg.conn_cfg.conn_cfg_tag = CONN_CFG_CENTRAL;
    blecfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = _sd_cfg.central.wr_cmd_qsize;
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
#endif


  /*------------- Configure GAP  -------------*/

  // Peripheral Preferred Connection Parameters
  ble_gap_conn_params_t   gap_conn_params =
  {
      .min_conn_interval = _ppcp_min_conn, // in 1.25ms unit
      .max_conn_interval = _ppcp_max_conn, // in 1.25ms unit
      .slave_latency     = BLE_GAP_CONN_SLAVE_LATENCY,
      .conn_sup_timeout  = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  };

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&gap_conn_params) );

  // Default device name
  ble_gap_conn_sec_mode_t sec_mode = BLE_SECMODE_OPEN;
  VERIFY_STATUS ( sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) CFG_DEFAULT_NAME, strlen(CFG_DEFAULT_NAME)) );

  VERIFY_STATUS( sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN) );
  VERIFY_STATUS( sd_ble_gap_tx_power_set( CFG_BLE_TX_POWER_LEVEL ) );

  /*------------- DFU OTA as built-in service -------------*/
  _dfu_svc.begin();

  if (_central_count)  Central.begin(); // Init Central

  // Create RTOS Semaphore & Task for BLE Event
  _ble_event_sem = xSemaphoreCreateBinary();
  VERIFY(_ble_event_sem, NRF_ERROR_NO_MEM);

  TaskHandle_t ble_task_hdl;
  xTaskCreate( adafruit_ble_task, "SD BLE", CFG_BLE_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &ble_task_hdl);

  // Create RTOS Semaphore & Task for SOC Event
  _soc_event_sem = xSemaphoreCreateBinary();
  VERIFY(_soc_event_sem, NRF_ERROR_NO_MEM);

  TaskHandle_t soc_task_hdl;
  xTaskCreate( adafruit_soc_task, "SD SOC", CFG_SOC_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &soc_task_hdl);

  NVIC_SetPriority(SD_EVT_IRQn, 6);
  NVIC_EnableIRQ(SD_EVT_IRQn);

  // Create Timer for led advertising blinky
  _led_blink_th = xTimerCreate(NULL, ms2tick(CFG_ADV_BLINKY_INTERVAL/2), true, NULL, bluefruit_blinky_cb);

  // Initialize nffs for bonding (it is safe to call nffs_pkg_init() multiple time)
  Nffs.begin();
  (void) Nffs.mkdir_p(CFG_BOND_NFFS_DIR);

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* General Functions
 *------------------------------------------------------------------*/
void AdafruitBluefruit::setName(const char* str)
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
  // Check if TX Power is valid value
  const int8_t accepted[] = { -40, -30, -20, -16, -12, -8, -4, 0, 4 };

  uint32_t i;
  for (i=0; i<sizeof(accepted); i++)
  {
    if (accepted[i] == power) break;
  }
  VERIFY(i < sizeof(accepted));

  // Apply
  VERIFY_STATUS( sd_ble_gap_tx_power_set(power), false);
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
  _ppcp_min_conn = min;
  _ppcp_max_conn = max;

  ble_gap_conn_params_t   gap_conn_params =
  {
      .min_conn_interval = _ppcp_min_conn, // in 1.25ms unit
      .max_conn_interval = _ppcp_max_conn, // in 1.25ms unit
      .slave_latency     = BLE_GAP_CONN_SLAVE_LATENCY,
      .conn_sup_timeout  = BLE_GAP_CONN_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  };

  VERIFY_STATUS( sd_ble_gap_ppcp_set(&gap_conn_params), false);

  return true;
}

bool AdafruitBluefruit::setConnIntervalMS(uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}


void AdafruitBluefruit::setConnectCallback( BLEGap::connect_callback_t fp )
{
  _connect_cb = fp;
}

void AdafruitBluefruit::setDisconnectCallback( BLEGap::disconnect_callback_t fp )
{
  _disconnect_cb = fp;
}

uint16_t AdafruitBluefruit::connHandle(void)
{
  return _conn_hdl;
}

bool AdafruitBluefruit::connPaired(void)
{
  return _bonded;
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

void Bluefruit_printInfo(void)
{
  Bluefruit.printInfo();
}

void AdafruitBluefruit::printInfo(void)
{
  // Skip if Serial is not initialised
  if ( !Serial.started() ) return;

  // Skip if Bluefruit.begin() is not called
  if ( _ble_event_sem == NULL ) return;

  Serial.println("--------- SoftDevice Config ---------");

  const char* title_fmt = "%-16s: ";

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

#if SD_VER >= 500
  if ( _prph_count )
  {
    Serial.println("Peripheral Connect Setting");

    Serial.print("  - ");
    Serial.printf(title_fmt, "Max MTU");
    Serial.println(_sd_cfg.prph.mtu_max);

    Serial.print("  - ");
    Serial.printf(title_fmt, "Event Length");
    Serial.println(_sd_cfg.prph.event_len);

    Serial.print("  - ");
    Serial.printf(title_fmt, "HVN Queue Size");
    Serial.println(_sd_cfg.prph.hvn_tx_qsize);

    Serial.print("  - ");
    Serial.printf(title_fmt, "WrCmd Queue Size");
    Serial.println(_sd_cfg.prph.wr_cmd_qsize);
  }

  if ( _central_count )
  {
    Serial.println("Central Connect Setting");

    Serial.print("  - ");
    Serial.printf(title_fmt, "Max MTU");
    Serial.println(_sd_cfg.central.mtu_max);

    Serial.print("  - ");
    Serial.printf(title_fmt, "Event Length");
    Serial.println(_sd_cfg.central.event_len);

    Serial.print("  - ");
    Serial.printf(title_fmt, "HVN Queue Size");
    Serial.println(_sd_cfg.central.hvn_tx_qsize);

    Serial.print("  - ");
    Serial.printf(title_fmt, "WrCmd Queue Size");
    Serial.println(_sd_cfg.central.wr_cmd_qsize);
  }
#endif

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
    const char* type_str[] = { "Public", "Static", "Private Resolvable", "Private Non Resolvable" };
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
  Serial.printf("min = %.2f ms, ", _ppcp_min_conn*1.25f);
  Serial.printf("max = %.2f ms", _ppcp_max_conn*1.25f);
  Serial.println();

  /*------------- List the paried device -------------*/
  Serial.printf(title_fmt, "Paired Devices");
  Serial.println();

  NffsDir dir(CFG_BOND_NFFS_DIR);
  NffsDirEntry dirEntry;
  while( dir.read(&dirEntry) )
  {
    if ( !dirEntry.isDirectory() )
    {
      char name[64];
      dirEntry.getName(name, sizeof(name));

      Serial.printf("  %s : ", name);

      // open file to read device name
      NffsFile file(CFG_BOND_NFFS_DIR, dirEntry, FS_ACCESS_READ);

      varclr(name);

      file.seek(BOND_FILE_DEVNAME_OFFSET);
      if ( file.read(name, CFG_MAX_DEVNAME_LEN) )
      {
        Serial.println(name);
      }

      file.close();
    }
  }
  Serial.println();

  Serial.println();
}

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
 * Handle SOC event such as FLASH opertion
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
      while ( NRF_ERROR_NOT_FOUND != err )
      {
        err = sd_evt_get(&soc_evt);

        if (ERROR_NONE == err)
        {
          switch (soc_evt)
          {
            case NRF_EVT_FLASH_OPERATION_SUCCESS:
            case NRF_EVT_FLASH_OPERATION_ERROR:
              if (hal_flash_event_cb) hal_flash_event_cb(soc_evt);
            break;

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

#if SD_VER >= 500
  uint8_t * ev_buf = (uint8_t*) rtos_malloc(BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX));
#endif

  while (1)
  {
    if ( xSemaphoreTake(Bluefruit._ble_event_sem, portMAX_DELAY) )
    {
      uint32_t err = ERROR_NONE;

      // Until no pending events
      while( NRF_ERROR_NOT_FOUND != err )
      {
#if SD_VER < 500
        __ALIGN(4) uint8_t ev_buf[ sizeof(ble_evt_t) + (BLE_GATT_ATT_MTU_DEFAULT) ];
        uint16_t ev_len = sizeof(ev_buf);
#else
        uint16_t ev_len = BLE_EVT_LEN_MAX(BLEGATT_ATT_MTU_MAX);
#endif

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
  const uint16_t evt_conn_hdl = evt->evt.common_evt.conn_handle;

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

          LOG_LV2("GAP", "Conn Interval= %f", _conn_interval*1.25f);

          // Connection interval set by Central is out of preferred range
          // Try to negotiate with Central using our preferred values
          if ( !is_within(_ppcp_min_conn, para->conn_params.min_conn_interval, _ppcp_max_conn) )
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

        LOG_LV2("GAP", "Conn Interval= %f", _conn_interval*1.25f);
      }
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        // Save all configured cccd
        if (_bonded) _saveBondCCCD();

        if (_disconnect_cb) ada_callback(NULL, _disconnect_cb, _conn_hdl, evt->evt.gap_evt.params.disconnected.reason);

        LOG_LV2("GAP", "Disconnect Reason 0x%02X", evt->evt.gap_evt.params.disconnected.reason);

        _conn_hdl = BLE_CONN_HANDLE_INVALID;
        _bonded   = false;
      break;

      case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      {
        // Pairing in progress
        varclr(&_bond_data);
        _bond_data.own_enc.master_id.ediv = 0xFFFF; // invalid value for ediv

        /* Step 1: Pairing/Bonding
         * - Central supplies its parameters
         * - We replies with our security parameters
         */
        //      ble_gap_sec_params_t* peer = &evt->evt.gap_evt.params.sec_params_request.peer_params;
        COMMENT_OUT(
            // Change security parameter according to authentication type
            if ( _auth_type == BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
            {
              sec_para.mitm    = 1;
              sec_para.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
            }
        )

        ble_gap_sec_keyset_t keyset =
        {
            .keys_own = {
                .p_enc_key  = &_bond_data.own_enc,
                .p_id_key   = NULL,
                .p_sign_key = NULL,
                .p_pk       = NULL
            },

            .keys_peer = {
                .p_enc_key  = &_bond_data.peer_enc,
                .p_id_key   = &_bond_data.peer_id,
                .p_sign_key = NULL,
                .p_pk       = NULL
            }
        };

        VERIFY_STATUS(sd_ble_gap_sec_params_reply(evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &_sec_param, &keyset), RETURN_VOID);
      }
      break;

      case BLE_GAP_EVT_SEC_INFO_REQUEST:
      {
        // Reconnection. If bonded previously, Central will ask for stored keys.
        // return security information. Otherwise NULL
        ble_gap_evt_sec_info_request_t* sec_request = (ble_gap_evt_sec_info_request_t*) &evt->evt.gap_evt.params.sec_info_request;

        if ( _loadBondKeys(sec_request->master_id.ediv) )
        {
          sd_ble_gap_sec_info_reply(evt->evt.gap_evt.conn_handle, &_bond_data.own_enc.enc_info, &_bond_data.peer_id.id_info, NULL);
        } else
        {
          sd_ble_gap_sec_info_reply(evt->evt.gap_evt.conn_handle, NULL, NULL, NULL);
        }
      }
      break;


      case BLE_GAP_EVT_PASSKEY_DISPLAY:
      {
        //      ble_gap_evt_passkey_display_t const* passkey_display = &evt->evt.gap_evt.params.passkey_display;
        //
        //      PRINT_INT(passkey_display->match_request);
        //      PRINT_BUFFER(passkey_display->passkey, 6);

        // sd_ble_gap_auth_key_reply
      }
      break;

      case BLE_GAP_EVT_CONN_SEC_UPDATE:
      {
        // Connection is secured aka Paired
        COMMENT_OUT( ble_gap_conn_sec_t* conn_sec = (ble_gap_conn_sec_t*) &evt->evt.gap_evt.params.conn_sec_update.conn_sec; )

          // Previously bonded --> secure by re-connection process
          // --> Load & Set Sys Attr (Apply Service Context)
          // Else Init Sys Attr
          _loadBondCCCD(_bond_data.own_enc.master_id.ediv);

        // Consider Paired as Bonded
        _bonded = true;
      }
      break;

      case BLE_GAP_EVT_AUTH_STATUS:
      {
        // Bonding process completed
        ble_gap_evt_auth_status_t* status = &evt->evt.gap_evt.params.auth_status;

        // Pairing/Bonding succeeded --> save encryption keys
        if (BLE_GAP_SEC_STATUS_SUCCESS == status->auth_status)
        {
          _bonded = true;
          ada_callback(NULL, _adafruit_save_bond_key_dfr, _conn_hdl);
        }else
        {
          PRINT_HEX(status->auth_status);
        }
      }
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
  // skip if already bonded
  if (_bonded) return true;

  VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &_sec_param ), false);
  uint32_t start = millis();

  // timeout in 30 seconds
  while ( !_bonded && (start + 30000 > millis()) )
  {
    yield();
  }

  return _bonded;
}

void AdafruitBluefruit::clearBonds(void)
{
  // Detele bonds dir
  Nffs.remove(CFG_BOND_NFFS_DIR);

  // Create an empty one
  Nffs.mkdir_p(CFG_BOND_NFFS_DIR);

  printBondDir();
}

/*------------------------------------------------------------------*/
/* Saving Bond Data to Nffs in following layout
 * - _bond_data 80 bytes
 * - Name       32 bytes
 * - CCCD       variable
 *------------------------------------------------------------------*/
void _adafruit_save_bond_key_dfr(uint32_t conn_handle)
{
  (void) conn_handle;
  Bluefruit._saveBondKeys();
}

void _adafruit_save_bond_cccd_dfr(uint32_t conn_handle)
{
  (void) conn_handle;
  Bluefruit._saveBondCCCD();
}

void AdafruitBluefruit::_saveBondKeys(void)
{
  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, _bond_data.own_enc.master_id.ediv);

  char devname[CFG_MAX_DEVNAME_LEN] = { 0 };
  Gap.getPeerName(_conn_hdl, devname, CFG_MAX_DEVNAME_LEN);

  NffsFile file(filename, FS_ACCESS_WRITE);

  VERIFY( file.exists(), );

  bool result = true;

  // write keys
  if ( !file.write((uint8_t*)&_bond_data, sizeof(_bond_data)) )
  {
    result = false;
  }

  // write device name
  if ( strlen(devname) && !file.write((uint8_t*) devname, CFG_MAX_DEVNAME_LEN) )
  {
    result = false;
  }

  file.close();

  if (result)
  {
    LOG_LV2("BOND", "Keys for \"%s\" is saved to file %s", devname, filename);
  }else
  {
    LOG_LV1("BOND", "Failed to save keys for \"%s\"", devname);
  }
  printBondDir();
}

bool AdafruitBluefruit::_loadBondKeys(uint16_t ediv)
{
  VERIFY_STATIC(sizeof(_bond_data) == 80 );

  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, ediv);

  bool result = (Nffs.readFile(filename, &_bond_data, sizeof(_bond_data)) > 0);

  if ( result )
  {
    LOG_LV2("BOND", "Load Keys from file %s", filename);
  }else
  {
    LOG_LV1("BOND", "Keys not found");
  }

  return result;
}

void AdafruitBluefruit::_saveBondCCCD(void)
{
  VERIFY( _bond_data.own_enc.master_id.ediv != 0xFFFF, );

  uint16_t len=0;
  sd_ble_gatts_sys_attr_get(_conn_hdl, NULL, &len, SVC_CONTEXT_FLAG);

  uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );
  VERIFY( sys_attr, );

  if ( ERROR_NONE == sd_ble_gatts_sys_attr_get(_conn_hdl, sys_attr, &len, SVC_CONTEXT_FLAG) )
  {
    // save to file
    char filename[BOND_FILENAME_LEN];
    sprintf(filename, BOND_FILENAME, _bond_data.own_enc.master_id.ediv);

    if ( Nffs.writeFile(filename, sys_attr, len, BOND_FILE_CCCD_OFFSET) )
    {
      LOG_LV2("BOND", "CCCD setting is saved to file %s", filename);
    }else
    {
      LOG_LV1("BOND", "Failed to save CCCD setting");
    }

  }

  printBondDir();

  rtos_free(sys_attr);
}

void AdafruitBluefruit::_loadBondCCCD(uint16_t ediv)
{
  bool loaded = false;

  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, ediv);

  NffsFile file(filename, FS_ACCESS_READ);

  if ( file.exists() )
  {
    int32_t len = file.size() - BOND_FILE_CCCD_OFFSET;

    if ( len )
    {
      uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );

      if (sys_attr)
      {
        file.seek(BOND_FILE_CCCD_OFFSET);

        if ( file.read(sys_attr, len ) )
        {
          if (ERROR_NONE == sd_ble_gatts_sys_attr_set(_conn_hdl, sys_attr, len, SVC_CONTEXT_FLAG) )
          {
            loaded = true;

            LOG_LV2("BOND", "Load CCCD from file %s", filename);
          }else
          {
            LOG_LV1("BOND", "CCCD setting not found");
          }
        }

        rtos_free(sys_attr);
      }
    }
  }

  file.close();

  if ( !loaded )
  {
    sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
  }
}

void AdafruitBluefruit::_bledfu_get_bond_data(ble_gap_addr_t* addr, ble_gap_irk_t* irk, ble_gap_enc_key_t* enc_key)
{
  if (!_bonded)
  {
    (*addr) = getPeerAddr();
  }else
  {
    (*addr)    = _bond_data.peer_id.id_addr_info;
    (*irk)     = _bond_data.peer_id.id_info;

    (*enc_key) = _bond_data.own_enc;
  }
}
