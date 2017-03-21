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

// Note chaning these parameters will affect APP_RAM_BASE
// --> need to update RAM in feather52_s132.ld linker
#define BLE_GATTS_ATTR_TABLE_SIZE        0x0B00
#define BLE_VENDOR_UUID_MAX              10
#define BLE_PRPH_MAX_CONN                1
#define BLE_CENTRAL_MAX_CONN             4
#define BLE_CENTRAL_MAX_SECURE_CONN      1 // currently 1 to save SRAM

#define SVC_CONTEXT_FLAG                 (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)

#define CFG_BLE_TX_POWER_LEVEL           0
#define CFG_DEFAULT_NAME                 "Bluefruit52"
#define CFG_ADV_BLINKY_INTERVAL          500

#define CFG_BLE_TASK_STACKSIZE          (512*3)
#define CFG_SOC_TASK_STACKSIZE          (configMINIMAL_STACK_SIZE*5) // 300

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


/*------------------------------------------------------------------*/
/* INTERNAL FUNCTION
 *------------------------------------------------------------------*/
static void bluefruit_blinky_cb( TimerHandle_t xTimer )
{
  (void) xTimer;
  ledToggle(LED_BLUE);
}

/**
 * Constructor
 */
AdafruitBluefruit::AdafruitBluefruit(void)
  : Central()
{
  _prph_enabled    = true;
  _central_enabled = false;

  _ble_event_sem    = NULL;
  _soc_event_sem    = NULL;

  _led_blink_th     = NULL;
  _led_conn         = true;

  _tx_power  = 0;

  strcpy(_name, CFG_DEFAULT_NAME);
  _txbuf_sem = NULL;

  _conn_hdl  = BLE_CONN_HANDLE_INVALID;
  _bonded    = false;

   _ppcp_min_conn = BLE_GAP_CONN_MIN_INTERVAL_DFLT;
   _ppcp_max_conn = BLE_GAP_CONN_MAX_INTERVAL_DFLT;
   _conn_interval = 0;

COMMENT_OUT(
  _auth_type = BLE_GAP_AUTH_KEY_TYPE_NONE;
  varclr(_pin);
)

  _chars_count = 0;
  for(uint8_t i=0; i<BLE_MAX_CHARS; i++) _chars_list[i] = NULL;

  varclr(&_bond_data);
  _bond_data.own_enc.master_id.ediv = 0xFFFF; // invalid value for ediv

  _connect_cb     = NULL;
  _discconnect_cb = NULL;
}

err_t AdafruitBluefruit::begin(bool prph_enable, bool central_enable)
{
  _prph_enabled    = prph_enable;
  _central_enabled = central_enable;

  // Configure Clock
  nrf_clock_lf_cfg_t clock_cfg =
  {
#if defined( USE_LFXO )
      // LFXO
      .source        = NRF_CLOCK_LF_SRC_XTAL,
      .rc_ctiv       = 0,
      .rc_temp_ctiv  = 0,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#else
      // LFRC
      .source        = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv       = 16,
      .rc_temp_ctiv  = 2,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#endif
  };

  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, NULL) );

  // Configure BLE params & ATTR Size
  ble_enable_params_t params =
  {
      .common_enable_params = { .vs_uuid_count = BLE_VENDOR_UUID_MAX },
      .gap_enable_params = {
          .periph_conn_count  = (uint8_t) (_prph_enabled    ? 1 : 0),
          .central_conn_count = (uint8_t) (_central_enabled ? BLE_CENTRAL_MAX_CONN : 0),
          .central_sec_count  = (uint8_t) (_central_enabled ? BLE_CENTRAL_MAX_SECURE_CONN : 0),
      },
      .gatts_enable_params = {
          .service_changed = 1,
          .attr_tab_size   = BLE_GATTS_ATTR_TABLE_SIZE
      }
  };

  extern uint32_t __data_start__[]; // defined in linker
  uint32_t app_ram_base = (uint32_t) __data_start__;

  VERIFY_STATUS( sd_ble_enable(&params, &app_ram_base) );

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
  VERIFY_STATUS ( sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) _name, strlen(_name)) );

  VERIFY_STATUS( sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN) );
  VERIFY_STATUS( sd_ble_gap_tx_power_set( CFG_BLE_TX_POWER_LEVEL ) );

  /*------------- DFU OTA as built-in service -------------*/
  _dfu_svc.begin();

  if (_central_enabled)  Central.begin(); // Init Central

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
  _led_blink_th = xTimerCreate(NULL, ms2tick(CFG_ADV_BLINKY_INTERVAL), true, NULL, bluefruit_blinky_cb);

  // Initialize nffs for bonding (it is safe to call nffs_pkg_init() multiple time)
  Nffs.begin();
  (void) Nffs.mkdir_p(CFG_BOND_NFFS_DIR);

  return ERROR_NONE;
}

err_t AdafruitBluefruit::setConnInterval(uint16_t min, uint16_t max)
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

  return sd_ble_gap_ppcp_set(&gap_conn_params);
}

err_t AdafruitBluefruit::setConnIntervalMS(uint16_t min_ms, uint16_t max_ms)
{
  return setConnInterval( MS100TO125(min_ms), MS100TO125(max_ms) );
}


void AdafruitBluefruit::setName(const char* str)
{
  strncpy(_name, str, 32);
}

char* AdafruitBluefruit::getName(void)
{
  return _name;
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

bool AdafruitBluefruit::connected(void)
{
  return ( _conn_hdl != BLE_CONN_HANDLE_INVALID );
}

void AdafruitBluefruit::disconnect(void)
{
  // disconnect if connected
  if ( connected() )
  {
    sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }
}

void AdafruitBluefruit::setConnectCallback   ( connect_callback_t fp )
{
  _connect_cb = fp;
}

void AdafruitBluefruit::setDisconnectCallback( disconnect_callback_t fp )
{
  _discconnect_cb = fp;
}

err_t AdafruitBluefruit::_registerCharacteristic(BLECharacteristic* chars)
{
  if ( _chars_count == BLE_MAX_CHARS ) return NRF_ERROR_NO_MEM;
  _chars_list[ _chars_count++ ] = chars;

  return ERROR_NONE;
}

uint16_t AdafruitBluefruit::connHandle(void)
{
  return _conn_hdl;
}

bool AdafruitBluefruit::connBonded(void)
{
  return _bonded;
}

uint16_t AdafruitBluefruit::connInterval(void)
{
  return _conn_interval;
}

ble_gap_addr_t AdafruitBluefruit::peerAddr(void)
{
  return _peer_addr;
}

bool AdafruitBluefruit::txbuf_get(uint32_t ms)
{
  return xSemaphoreTake(_txbuf_sem, ms2tick(ms));
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
/* Private Methods
 *------------------------------------------------------------------*/
void AdafruitBluefruit::startConnLed(void)
{
  if (_led_conn) xTimerStart(_led_blink_th, 0);
}

void AdafruitBluefruit::stopConnLed(void)
{
  xTimerStop(_led_blink_th, 0);
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

  enum { BLE_STACK_EVT_MSG_BUF_SIZE = (sizeof(ble_evt_t) + (GATT_MTU_SIZE_DEFAULT)) };

  while (1)
  {
    if ( xSemaphoreTake(Bluefruit._ble_event_sem, portMAX_DELAY) )
    {
      uint32_t err = ERROR_NONE;

      // Until no pending events
      while( NRF_ERROR_NOT_FOUND != err )
      {
        uint32_t ev_buf[BLE_STACK_EVT_MSG_BUF_SIZE/4 + 4];
        uint16_t ev_len = sizeof(ev_buf);

        // Get BLE Event
        err = sd_ble_evt_get((uint8_t*)ev_buf, &ev_len);

        // Handle valid event, ignore error
        if( ERROR_NONE == err)
        {
          Bluefruit._ble_handler( (ble_evt_t*) ev_buf );
        }
      }
    }
  }
}

void AdafruitBluefruit::_ble_handler(ble_evt_t* evt)
{
  #if CFG_DEBUG
  Serial.printf("[BLE]: %s\n", dbg_ble_event_str(evt->header.evt_id));
  #endif

  /*------------- BLE Peripheral Events -------------*/
  switch ( evt->header.evt_id  )
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      ble_gap_evt_connected_t* para = &evt->evt.gap_evt.params.connected;

      if (para->role == BLE_GAP_ROLE_PERIPH)
      {
        stopConnLed();
        if (_led_conn) ledOn(LED_BLUE);

        _conn_hdl      = evt->evt.gap_evt.conn_handle;
        _conn_interval = para->conn_params.min_conn_interval;
        _peer_addr     = para->peer_addr;

        // Connection interval set by Central is out of preferred range
        // Try to negotiate with Central using our preferred values
        if ( !is_within(_ppcp_min_conn, para->conn_params.min_conn_interval, _ppcp_max_conn) )
        {
          // Null, value is set by sd_ble_gap_ppcp_set will be used
          VERIFY_STATUS( sd_ble_gap_conn_param_update(_conn_hdl, NULL), );
        }

        // Init transmission buffer for notification
        uint8_t txbuf_max;
        (void) sd_ble_tx_packet_count_get(_conn_hdl, &txbuf_max);
        _txbuf_sem = xSemaphoreCreateCounting(txbuf_max, txbuf_max);

        if ( _connect_cb ) _connect_cb();
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
      // Check if it is peripheral connection
      if (_conn_hdl == evt->evt.gap_evt.conn_handle)
      {
        if (_led_conn)  ledOff(LED_BLUE);

        // Save all configured cccd
        if (_bonded) _saveBondedCCCD();

        _conn_hdl = BLE_CONN_HANDLE_INVALID;
        _bonded   = false;
        varclr(&_peer_addr);

        vSemaphoreDelete(_txbuf_sem);
        _txbuf_sem = NULL;

        if ( _discconnect_cb ) _discconnect_cb(evt->evt.gap_evt.params.disconnected.reason);

        Advertising.start();
      }
    break;

    case BLE_GAP_EVT_TIMEOUT:
      if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
      {
        // Restart Advertising
        Advertising.start();
      }
    break;

    case BLE_EVT_TX_COMPLETE:
      if ( _txbuf_sem )
      {
        for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
        {
          xSemaphoreGive(_txbuf_sem);
        }
      }
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

      ble_gap_sec_params_t sec_para =
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

      VERIFY_STATUS(sd_ble_gap_sec_params_reply(evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &sec_para, &keyset), RETURN_VOID);
    }
    break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
    {
      // Reconnection. If bonded previously, Central will ask for stored keys.
      // return security information. Otherwise NULL
      ble_gap_evt_sec_info_request_t* sec_request = (ble_gap_evt_sec_info_request_t*) &evt->evt.gap_evt.params.sec_info_request;

      //if (_bond_data.own_enc.master_id.ediv == sec_request->master_id.ediv)
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
      _loadBondedCCCD(_bond_data.own_enc.master_id.ediv);

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
        _saveBondKeys();
        _bonded = true;
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

  // Central Event Handler
  if (_central_enabled) Central._event_handler(evt);

  // GATTs characteristics event handler
  for(int i=0; i<_chars_count; i++)
  {
    _chars_list[i]->eventHandler(evt);
  }
}
/*------------------------------------------------------------------*/
/* Bonds
 *------------------------------------------------------------------*/
void AdafruitBluefruit::clearBonds(void)
{
  // Detele bonds dir
  Nffs.remove(CFG_BOND_NFFS_DIR);

  // Create an empty one
  Nffs.mkdir_p(CFG_BOND_NFFS_DIR);
}

bool AdafruitBluefruit::_saveBondKeys(void)
{
  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, _bond_data.own_enc.master_id.ediv);

  return Nffs.writeFile(filename, &_bond_data, sizeof(_bond_data));
}

bool AdafruitBluefruit::_loadBondKeys(uint16_t ediv)
{
  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, ediv);

  bool result = Nffs.readFile(filename, &_bond_data, sizeof(_bond_data)) > 0;

  if ( result ) {
    LOG_LV1(BOND, "Load Keys from %s", filename);
  }

  return result;
}


void AdafruitBluefruit::_loadBondedCCCD(uint16_t ediv)
{
  bool loaded = false;

  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME "_cccd", ediv);

  NffsFile file(filename, FS_ACCESS_READ);

  if ( file.exists() )
  {
    uint16_t len = file.size();
    uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );

    if (sys_attr)
    {
      if ( file.read(sys_attr, len ) )
      {
        if (ERROR_NONE == sd_ble_gatts_sys_attr_set(_conn_hdl, sys_attr, len, SVC_CONTEXT_FLAG) )
        {
          loaded = true;

          LOG_LV1(BOND, "Load CCCD from %s", filename);
        }
      }

      rtos_free(sys_attr);
    }
  }

  file.close();

  if ( !loaded )
  {
    sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
  }
}

void AdafruitBluefruit::_saveBondedCCCD(void)
{
  if ( _bond_data.own_enc.master_id.ediv == 0xFFFF ) return;

  uint16_t len=0;
  sd_ble_gatts_sys_attr_get(_conn_hdl, NULL, &len, SVC_CONTEXT_FLAG);

  uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );
  VERIFY( sys_attr, );

  if ( ERROR_NONE == sd_ble_gatts_sys_attr_get(_conn_hdl, sys_attr, &len, SVC_CONTEXT_FLAG) )
  {
    // save to file
    char filename[BOND_FILENAME_LEN];
    sprintf(filename, BOND_FILENAME "_cccd", _bond_data.own_enc.master_id.ediv);

    Nffs.writeFile(filename, sys_attr, len);
  }

  rtos_free(sys_attr);
}
