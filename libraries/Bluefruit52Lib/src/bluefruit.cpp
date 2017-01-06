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

#define BLE_VENDOR_UUID_MAX          10
#define BLE_PRPH_MAX_CONN            1
#define BLE_CENTRAL_MAX_CONN         0
#define BLE_CENTRAL_MAX_SECURE_CONN  0

#define CFG_GAP_CONNECTION_SUPERVISION_TIMEOUT_MS  3000
#define CFG_GAP_CONNECTION_SLAVE_LATENCY           0
#define CFG_GAP_ADV_INTERVAL_MS                    20
#define CFG_GAP_ADV_TIMEOUT_S                      30

#define CFG_BLE_TX_POWER_LEVEL                     0
#define CFG_DEFAULT_NAME    "Bluefruit"

// Converts an integer of 1.25ms units to msecs
#define MS100TO125(ms100) (((ms100)*4)/5)

// Converts an integer of 1.25ms units to msecs
#define MS125TO100(ms125) (((ms125)*5)/4)

// Converts msec to 0.625 unit
#define MS1000TO625(ms1000) (((ms1000)*8)/5)

// Converts an integer of 625ms units to msecs
#define MS625TO1000(u625) ( ((u625)*5) / 8 )

// To change this value, an adjustment for SRAM is required in linker script
#define BLE_GATTS_ATTR_TABLE_SIZE   0x1000

#define BLUEFRUIT_TASK_STACKSIZE   (1024*4)


static void adafruit_bluefruit_task(void* arg);

AdafruitBluefruit Bluefruit;

AdafruitBluefruit::AdafruitBluefruit(void)
{
  varclr(&_adv);
  varclr(&_scan_resp);

  _tx_power = 0;

  strcpy(_name, CFG_DEFAULT_NAME);
  _txbuf_sem = NULL;

  _conn_hdl = BLE_GATT_HANDLE_INVALID;

  _chars_count = 0;
  for(uint8_t i=0; i<BLE_MAX_CHARS; i++) _chars_list[i] = NULL;


  varclr(&_enc_key);
}

err_t AdafruitBluefruit::begin(void)
{
  // Configure Clock
  nrf_clock_lf_cfg_t clock_cfg =
  {
      .source        = NRF_CLOCK_LF_SRC_RC,
      .rc_ctiv       = 16,
      .rc_temp_ctiv  = 2,
      .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
  };

  VERIFY_STATUS( sd_softdevice_enable(&clock_cfg, NULL) );

  // Configure BLE params & ATTR Size
  ble_enable_params_t params =
  {
      .common_enable_params = { .vs_uuid_count = BLE_VENDOR_UUID_MAX },
      .gap_enable_params = {
          .periph_conn_count  = BLE_PRPH_MAX_CONN,
          .central_conn_count = BLE_CENTRAL_MAX_CONN,
          .central_sec_count  = BLE_CENTRAL_MAX_SECURE_CONN
      },
      .gatts_enable_params = {
          .service_changed = 1,
          .attr_tab_size = BLE_GATTS_ATTR_TABLE_SIZE
      }
  };

  extern uint32_t __data_start__; // defined in linker
  uint32_t app_ram_base = (uint32_t) &__data_start__;

  VERIFY_STATUS( sd_ble_enable(&params, &app_ram_base) );

  /*------------- Configure GAP  -------------*/

  // Connection Parameters
  ble_gap_conn_params_t   gap_conn_params =
  {
      .min_conn_interval = MS100TO125(20) , // in 1.25ms unit
      .max_conn_interval = MS100TO125(40) , // in 1.25ms unit
      .slave_latency     = CFG_GAP_CONNECTION_SLAVE_LATENCY,
      .conn_sup_timeout  = CFG_GAP_CONNECTION_SUPERVISION_TIMEOUT_MS / 10 // in 10ms unit
  };

  (void) sd_ble_gap_ppcp_set(&gap_conn_params);

  // Default device name
  ble_gap_conn_sec_mode_t sec_mode = BLE_SECMODE_OPEN;
  VERIFY_STATUS ( sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *) _name, strlen(_name)) );

  VERIFY_STATUS( sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN) );
  VERIFY_STATUS ( sd_ble_gap_tx_power_set( CFG_BLE_TX_POWER_LEVEL ) );

  /*------------- DFU OTA as built-in service -------------*/
  _dfu_svc.start();

  // Create RTOS Task for BLE Event
  TaskHandle_t task_hdl;
  xTaskCreate( adafruit_bluefruit_task, "ble handler", BLUEFRUIT_TASK_STACKSIZE, NULL, TASK_PRIO_HIGH, &task_hdl);

  return NRF_SUCCESS;
}

void AdafruitBluefruit::setName(const char* str)
{
  strncpy(_name, str, 32);
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

/*------------------------------------------------------------------*/
/* Advertising
 *------------------------------------------------------------------*/
bool AdafruitBluefruit::_addToAdv(bool scan_resp, uint8_t type, const void* data, uint8_t len)
{
  uint8_t* adv_data = (scan_resp ? &_scan_resp.data[_scan_resp.count] : &_adv.data[_adv.count]);
  uint8_t* count    = (scan_resp ? &_scan_resp.count : &_adv.count);

  VERIFY( (*count) + len + 2 <= BLE_GAP_ADV_MAX_SIZE );

  *adv_data++ = (len+1);
  *adv_data++ = type;
  memcpy(adv_data, data, len);

  (*count) = (*count) + len + 2;

  return true;
}

bool AdafruitBluefruit::addAdvData(uint8_t type, const void* data, uint8_t len)
{
  return _addToAdv(false, type, data, len);
}

bool AdafruitBluefruit::addAdvUuid(uint16_t uuid16)
{
  return addAdvData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, &uuid16, 2);
}

bool AdafruitBluefruit::addAdvUuid(uint8_t const  uuid128[])
{
  return addAdvData(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, uuid128, 16);
}

bool AdafruitBluefruit::addAdvService(BLEService& service)
{
  // UUID128
  if ( service.uuid._uuid128 )
  {
    return addAdvUuid(service.uuid._uuid128);
  }else
  {
    return addAdvUuid(service.uuid._uuid.uuid);
  }
}

// Add Name to Adv packet, use setName() to set
bool AdafruitBluefruit::addAdvName(void)
{
  uint8_t type = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
  uint8_t len  = strlen(_name);

  // not enough for full name, chop it
  if (_adv.count + len + 2 > BLE_GAP_ADV_MAX_SIZE)
  {
    type = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    len  = BLE_GAP_ADV_MAX_SIZE - (_adv.count+2);
  }

  return addAdvData(type, _name, len);
}

// tx power is set by setTxPower
bool AdafruitBluefruit::addAdvTxPower(void)
{
  return addAdvData(BLE_GAP_AD_TYPE_TX_POWER_LEVEL, &_tx_power, 1);
}

bool AdafruitBluefruit::addAdvFlags(uint8_t flags)
{
  return addAdvData(BLE_GAP_AD_TYPE_FLAGS, &flags, 1);
}

uint8_t AdafruitBluefruit::getAdvLen(void)
{
  return _adv.count;
}

uint8_t AdafruitBluefruit::getAdvData(uint8_t* buffer)
{
  memcpy(buffer, _adv.data, _adv.count);
  return _adv.count;
}

bool AdafruitBluefruit::setAdvData(uint8_t const * data, uint8_t count)
{
  VERIFY( data && count <= BLE_GAP_ADV_MAX_SIZE);

  memcpy(_adv.data, data, count);
  _adv.count = count;

  return true;
}

void AdafruitBluefruit::clearAdvData(void)
{
  varclr(&_adv);
}

/*------------------------------------------------------------------*/
/* Scan Response
 *------------------------------------------------------------------*/
bool AdafruitBluefruit::addScanData(uint8_t type, const void* data, uint8_t len)
{
  return _addToAdv(true, type, data, len);
}

bool AdafruitBluefruit::addScanUuid(uint16_t uuid16)
{
  return addScanData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, &uuid16, 2);
}

bool AdafruitBluefruit::addScanUuid(uint8_t const  uuid128[])
{
  return addScanData(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, uuid128, 16);
}

bool AdafruitBluefruit::addScanName(void)
{
  uint8_t type = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
  uint8_t len  = strlen(_name);

  // not enough for full name, chop it
  if (_scan_resp.count + len + 2 > BLE_GAP_ADV_MAX_SIZE)
  {
    type = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    len  = BLE_GAP_ADV_MAX_SIZE - (_scan_resp.count+2);
  }

  return addScanData(type, _name, len);
}

uint8_t AdafruitBluefruit::getScanLen(void)
{
  return _scan_resp.count;
}

uint8_t AdafruitBluefruit::getScanData(uint8_t* buffer)
{
  memcpy(buffer, _scan_resp.data, _scan_resp.count);
  return _scan_resp.count;
}

bool AdafruitBluefruit::setScanData(uint8_t const* data, uint8_t count)
{
  VERIFY (data && count <= BLE_GAP_ADV_MAX_SIZE);

  memcpy(_scan_resp.data, data, count);
  _scan_resp.count = count;

  return true;
}

void AdafruitBluefruit::clearScanData(void)
{
  varclr(&_scan_resp);
}

err_t AdafruitBluefruit::startAdvertising(void)
{
  VERIFY_STATUS( sd_ble_gap_adv_data_set(_adv.data, _adv.count, _scan_resp.data, _scan_resp.count) );

  // ADV Params
  ble_gap_adv_params_t adv_para =
  {
      .type        = BLE_GAP_ADV_TYPE_ADV_IND,
      .p_peer_addr = NULL                            , // Undirected advertisement
      .fp          = BLE_GAP_ADV_FP_ANY              ,
      .p_whitelist = NULL                            ,
      .interval    = MS1000TO625(CFG_GAP_ADV_INTERVAL_MS), // advertising interval (in units of 0.625 ms)
      .timeout     = CFG_GAP_ADV_TIMEOUT_S
  };

  VERIFY_STATUS( sd_ble_gap_adv_start(&adv_para) );

  return NRF_SUCCESS;
}

void AdafruitBluefruit::stopAdvertising(void)
{
  sd_ble_gap_adv_stop();
}

err_t AdafruitBluefruit::addService(uint16_t uuid16)
{
  BLEUuid uuid(uuid16);

  uint16_t svc_handle;
  return sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid._uuid, &svc_handle);
}

err_t AdafruitBluefruit::addService(uint8_t const uuid128[])
{
  BLEUuid uuid;
  uuid.set(uuid128);

  uint16_t svc_handle;
  return sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid._uuid, &svc_handle);
}

err_t AdafruitBluefruit::_registerCharacteristic(BLECharacteristic* chars)
{
  if ( _chars_count == BLE_MAX_CHARS ) return NRF_ERROR_NO_MEM;
  _chars_list[ _chars_count++ ] = chars;

  return NRF_SUCCESS;
}

uint16_t AdafruitBluefruit::connHandle(void)
{
  return _conn_hdl;
}

ble_gap_addr_t AdafruitBluefruit::peerAddr(void)
{
  return _peer_addr;
}

bool AdafruitBluefruit::txbuf_get(uint32_t ms)
{
  return xSemaphoreTake(_txbuf_sem, ms2tick(ms));
}

/**
 *
 */

enum {
  SEC_PARAM_TIMEOUT         = 30 , /**< Timeout for Pairing Request or Security Request (in seconds). */
  SEC_PARAM_BOND            = 1  , /**< Perform bonding. */
  SEC_PARAM_OOB             = 0  , /**< Out Of Band data not available. */
  SEC_PARAM_MIN_KEY_SIZE    = 7  , /**< Minimum encryption key size. */
  SEC_PARAM_MAX_KEY_SIZE    = 16
};

static void adafruit_bluefruit_task(void* arg)
{
  (void) arg;

  while (1)
  {
    Bluefruit.poll();

    vTaskDelay(1);
    //sd_app_evt_wait();
  }
}

void AdafruitBluefruit::poll(void)
{
  enum { BLE_STACK_EVT_MSG_BUF_SIZE = (sizeof(ble_evt_t) + (GATT_MTU_SIZE_DEFAULT)) };

  uint32_t ev_buf[BLE_STACK_EVT_MSG_BUF_SIZE/4 + 4];
  uint16_t ev_len = sizeof(ev_buf);
  ble_evt_t* evt = (ble_evt_t*) ev_buf;

  if( NRF_SUCCESS == sd_ble_evt_get((uint8_t*)ev_buf, &ev_len))
  {
    switch ( evt->header.evt_id  )
    {
      case BLE_GAP_EVT_CONNECTED:
        digitalWrite(LED_CONN, HIGH);

        _conn_hdl = evt->evt.gap_evt.conn_handle;
        _peer_addr = evt->evt.gap_evt.params.connected.peer_addr;

        uint8_t txbuf_max;
        (void) sd_ble_tx_packet_count_get(_conn_hdl, &txbuf_max);
        _txbuf_sem = xSemaphoreCreateCounting(txbuf_max, txbuf_max);
      break;

      case BLE_GAP_EVT_DISCONNECTED:
        _conn_hdl = BLE_GATT_HANDLE_INVALID;
        digitalWrite(LED_CONN, LOW);

        vSemaphoreDelete(_txbuf_sem);

        startAdvertising();
      break;

      case BLE_GAP_EVT_TIMEOUT:
        if (evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
        {
          // Restart Advertising
          startAdvertising();
        }
      break;

      case BLE_EVT_TX_COMPLETE:
        for(uint8_t i=0; i<evt->evt.common_evt.params.tx_complete.count; i++)
        {
          xSemaphoreGive(_txbuf_sem);
        }
      break;

#if 0
      case BLE_GAP_EVT_SEC_INFO_REQUEST:
        if (_enc_key.master_id.ediv == evt->evt.gap_evt.params.sec_info_request.master_id.ediv)
        {
          sd_ble_gap_sec_info_reply(evt->evt.gap_evt.conn_handle, &_enc_key.enc_info, NULL, NULL);
        } else
        {
          sd_ble_gap_sec_info_reply(evt->evt.gap_evt.conn_handle, NULL, NULL, NULL);
        }
      break;

      case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      {
        ble_gap_sec_params_t sec_para =
        {
            .bond         = SEC_PARAM_BOND            ,
            .mitm         = 0, //CFG_PIN_ENABLED ? nvm_data.core.passkey_enable : 0,
            .lesc         = 0,
            .keypress     = 0,
            .io_caps      = BLE_GAP_IO_CAPS_NONE, // (CFG_PIN_ENABLED && nvm_data.core.passkey_enable) ? BLE_GAP_IO_CAPS_DISPLAY_ONLY : BLE_GAP_IO_CAPS_NONE ,
            .oob          = SEC_PARAM_OOB             ,
            .min_key_size = SEC_PARAM_MIN_KEY_SIZE    ,
            .max_key_size = SEC_PARAM_MAX_KEY_SIZE
        };

        ble_gap_sec_keyset_t keyset =
        {
            .keys_own = {
                .p_enc_key  = &_enc_key,
                .p_id_key   = NULL,
                .p_sign_key = NULL,
                .p_pk       = NULL
            },

            .keys_peer = { NULL, NULL, NULL, NULL }
        };

        VERIFY_STATUS(sd_ble_gap_sec_params_reply(evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &sec_para, &keyset),
                      RETURN_VOID);
      }
      break;

      case BLE_GAP_EVT_AUTH_STATUS:
        // Bonding succeeded --> save encryption keys
        if (BLE_GAP_SEC_STATUS_SUCCESS == evt->evt.gap_evt.params.auth_status.auth_status)
        {

        }
      break;

      case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
      break;
#endif

      case BLE_GAP_EVT_CONN_SEC_UPDATE:
      break;

      default: break;
    }

    // GATTs characteristics event handler
    for(int i=0; i<_chars_count; i++)
    {
      _chars_list[i]->eventHandler(evt);
    }
  }
}
