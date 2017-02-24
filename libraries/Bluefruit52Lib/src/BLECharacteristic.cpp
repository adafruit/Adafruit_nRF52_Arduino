/**************************************************************************/
/*!
    @file     BLECharacteristic.cpp
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

void BLECharacteristic::init(void)
{
  _is_temp    = false;

  varclr(&_properties);
  _descriptor = NULL;
  _max_len    = BLE_GATTS_VAR_ATTR_LEN_MAX;

  varclr(&_report_ref_desc);

  _service = NULL;

  _handles.value_handle     = BLE_GATT_HANDLE_INVALID;
  _handles.user_desc_handle = BLE_GATT_HANDLE_INVALID;
  _handles.sccd_handle      = BLE_GATT_HANDLE_INVALID;
  _handles.cccd_handle      = BLE_GATT_HANDLE_INVALID;

  varclr(&_attr_meta);
  // default permission is OPEN
  _attr_meta.read_perm = _attr_meta.write_perm = BLE_SECMODE_OPEN;
  _attr_meta.vlen = 1;
  _attr_meta.vloc = BLE_GATTS_VLOC_STACK;

  _rd_authorize_cb = NULL;
  _wr_authorize_cb = NULL;
  _wr_cb           = NULL;
  _cccd_wr_cb      = NULL;
}

BLECharacteristic::BLECharacteristic(void)
  : uuid()
{
  init();
}

BLECharacteristic::BLECharacteristic(uint16_t uuid16)
  : uuid(uuid16)
{
  init();
}    

BLECharacteristic::BLECharacteristic(uint8_t const  uuid128[])
  : uuid(uuid128)
{
  init();
}

void BLECharacteristic::setUuid(uint16_t uuid16)
{
  uuid.set(uuid16);
}

void BLECharacteristic::setUuid(uint8_t const  uuid128[])
{
  uuid.set(uuid128);
}

/**
 * Must be set when Charactertistic is declared locally (e.g insdie function)
 * and is not last throughout programs. Useful for one-shot set-and-forget
 * Characteristics such as read-only one. Where there is no need for interactions
 * later on. This call will prevent the Characteristics to be hooked into
 * managing chars list of AdafruitBluefruit
 */
void BLECharacteristic::setTempMemory(void)
{
  _is_temp = true;
}

void BLECharacteristic::setProperties(uint8_t prop)
{
  memcpy(&_properties, &prop, 1);
}

void BLECharacteristic::setMaxLen(uint16_t max_len)
{
  _max_len = max_len;
}

void BLECharacteristic::setFixedLen(uint16_t fixed_len)
{
  if ( fixed_len )
  {
    _max_len = fixed_len;
    _attr_meta.vlen = 0;
  }else
  {
    _attr_meta.vlen = 1;
  }
}

void BLECharacteristic::setPermission(BleSecurityMode read_perm, BleSecurityMode write_perm)
{
  memcpy(&_attr_meta.read_perm , &read_perm, 1);
  memcpy(&_attr_meta.write_perm, &write_perm, 1);
}

void BLECharacteristic::setWriteCallback(write_cb_t fp)
{
  _wr_cb = fp;
}

void BLECharacteristic::setCccdWriteCallback(write_cb_t fp)
{
  _cccd_wr_cb = fp;
}

void BLECharacteristic::setReadAuthorizeCallback(read_authorize_cb_t fp)
{
  _attr_meta.rd_auth = (fp  ? 1 : 0);
  _rd_authorize_cb = fp;
}

void BLECharacteristic::setWriteAuthorizeCallbak(write_authorize_cb_t fp)
{
  _attr_meta.wr_auth = (fp ? 1 : 0);
  _wr_authorize_cb = fp;
}

void BLECharacteristic::setStringDescriptor(const char* descriptor)
{
  _descriptor = descriptor;
}

void BLECharacteristic::setReportRefDescriptor(uint8_t id, uint8_t type)
{
  _report_ref_desc.id   = id;
  _report_ref_desc.type = type;
}

ble_gatts_char_handles_t BLECharacteristic::handles(void)
{
  return _handles;
}

err_t BLECharacteristic::start(void)
{
  _service = BLEService::lastService;

  // Add UUID128 if needed
  uuid.add();

  // Permission is OPEN if passkey is disabled.
//  if (!nvm_data.core.passkey_enable) BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_char_def->permission);

  // Correct Read/Write permission according to properties
  if ( !(_properties.read || _properties.notify || _properties.indicate ) )
  {
    _attr_meta.read_perm = BLE_SECMODE_NO_ACCESS;
  }

  if ( !(_properties.write || _properties.write_wo_resp ) )
  {
    _attr_meta.write_perm = BLE_SECMODE_NO_ACCESS;
  }

  /* CCCD attribute metadata */
  ble_gatts_attr_md_t cccd_md;

  if ( _properties.notify || _properties.indicate )
  {
    /* Notification & Indication require CCCD */
    memclr( &cccd_md, sizeof(ble_gatts_attr_md_t) );
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    cccd_md.read_perm  = BLE_SECMODE_OPEN;
    cccd_md.write_perm = _attr_meta.read_perm;
  }

  /* Characteristic metadata */
  ble_gatts_char_md_t char_md;
  varclr(&char_md);

  char_md.char_props = _properties;
  char_md.p_cccd_md  = (_properties.notify || _properties.indicate) ? &cccd_md : NULL;

  /* Characteristic extended properties (for user description) */
  ble_gatts_attr_md_t desc_md =
  {
      .read_perm  = _attr_meta.read_perm,
      .write_perm = BLE_SECMODE_NO_ACCESS,
      .vlen       = 0,
      .vloc       = BLE_GATTS_VLOC_STACK,
  };

  if (_descriptor != NULL && _descriptor[0] != 0)
  {
    char_md.p_char_user_desc    = (uint8_t*) _descriptor;
    char_md.char_user_desc_size = char_md.char_user_desc_max_size = strlen(_descriptor);
    char_md.p_user_desc_md      = &desc_md;
    //char_md.char_ext_props    = ext_props,
  }

  /* Presentation format
   * https://developer.bluetooth.org/gatt/descriptors/Pages/DescriptorViewer.aspx?u=org.bluetooth.descriptor.gatt.characteristic_presentation_format.xml
   */
//   if ( p_char_def->presentation.format )
//   {
//     char_md.p_char_pf = &p_char_def->presentation;
//   }

  /* GATT attribute declaration */
  ble_gatts_attr_t attr_char_value =
  {
    .p_uuid    = &uuid._uuid,
    .p_attr_md = &_attr_meta,
    .init_len  = (_attr_meta.vlen == 1) ? (uint16_t) 0 : _max_len,
    .init_offs = 0,
    .max_len   = _max_len,
    .p_value   = NULL
  };

  VERIFY_STATUS( sd_ble_gatts_characteristic_add(BLE_GATT_HANDLE_INVALID, &char_md, &attr_char_value, &_handles) );

  // Report Reference Descriptor if any (required by HID)
  if ( _report_ref_desc.type )
  {
    // Reference Descriptor
    ble_gatts_attr_md_t ref_md =
    {
        .read_perm  = _attr_meta.read_perm,
        .write_perm = BLE_SECMODE_NO_ACCESS,
        .vlen       = 0,
        .vloc       = BLE_GATTS_VLOC_STACK
    };

    ble_uuid_t ref_uuid = { .uuid = UUID16_REPORT_REF_DESCR, .type = BLE_UUID_TYPE_BLE };
    ble_gatts_attr_t ref_desc =
    {
        .p_uuid    = &ref_uuid,
        .p_attr_md = &ref_md,
        .init_len  = sizeof(_report_ref_desc),
        .init_offs = 0,
        .max_len   = sizeof(_report_ref_desc),
        .p_value   = (uint8_t*) &_report_ref_desc
    };

    uint16_t ref_hdl;
    VERIFY_STATUS ( sd_ble_gatts_descriptor_add(BLE_GATT_HANDLE_INVALID, &ref_desc, &ref_hdl) );

    (void) ref_hdl; // not used
  }

  // Currently Only register to Bluefruit when having callback support
  // And The Characteristic must not be temporary memory aka local variable
  if ( !_is_temp &&
       (_rd_authorize_cb || _wr_authorize_cb || _wr_cb || _cccd_wr_cb) )
  {
    (void) Bluefruit._registerCharacteristic(this);
  }

  return ERROR_NONE;
}

/**
 * Call by Bluefruit event poll()
 * @param event
 */
void BLECharacteristic::eventHandler(ble_evt_t* event)
{
  switch(event->header.evt_id)
  {
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
      ble_gatts_evt_rw_authorize_request_t * authorize_request = &event->evt.gatts_evt.params.authorize_request;

      ble_uuid_t request_uuid = (authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) ?
          authorize_request->request.write.uuid : authorize_request->request.read.uuid;

      // skip if not our event
      if ( uuid == request_uuid )
      {
        if ( (authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) && (_wr_authorize_cb != NULL))
        {
          this->_wr_authorize_cb(*this, &authorize_request->request.write);
        }

        if ( (authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_READ) && (_rd_authorize_cb != NULL))
        {
          _rd_authorize_cb(*this, &authorize_request->request.read);
        }
      }
    }
    break;

    case BLE_GATTS_EVT_WRITE:
    {
      ble_gatts_evt_write_t* request = (ble_gatts_evt_write_t*) &event->evt.gatts_evt.params.write;

      // Value write
      if ( _wr_cb && (request->handle == _handles.value_handle))
      {
        _wr_cb(*this, request);
      }

      // CCCD write
      if ( _cccd_wr_cb && (request->handle == _handles.cccd_handle) )
      {
        _cccd_wr_cb(*this, request);
      }
    }
    break;

    default: break;
  }
}

/*------------------------------------------------------------------*/
/* WRITE
 *------------------------------------------------------------------*/

err_t BLECharacteristic::write(const uint8_t* data, int len)
{
  return write(data, len, 0);
}

err_t BLECharacteristic::write(const char   * str)
{
  return write((const uint8_t*) str, strlen(str));
}

err_t BLECharacteristic::write(const uint8_t* data, int len, uint16_t offset)
{
  // could not exceed max len
  uint16_t actual_len = min16(len, _max_len);

  ble_gatts_value_t value =
  {
      .len     = (uint16_t) actual_len,
      .offset  = offset,
      .p_value = (uint8_t*) data
  };

  // conn handle only needed for system attribute
  return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, _handles.value_handle, &value);
}

err_t BLECharacteristic::write(int num)
{
  return write( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::write(uint32_t num)
{
  return write( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::write(uint16_t num)
{
  return write( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::write(uint8_t  num)
{
  return write( (uint8_t*) &num, sizeof(num));
}

/*------------------------------------------------------------------*/
/* NOTIFY
 *------------------------------------------------------------------*/
bool BLECharacteristic::notifyEnabled(void)
{
  VERIFY( _properties.notify && Bluefruit.connected() );

  uint16_t cccd;
  ble_gatts_value_t value =
  {
      .len     = 2,
      .offset  = 0,
      .p_value = (uint8_t*) &cccd
  };

  VERIFY_STATUS( sd_ble_gatts_value_get(Bluefruit.connHandle(), _handles.cccd_handle, &value), false );

  return (cccd & BLE_GATT_HVX_NOTIFICATION);
}

err_t BLECharacteristic::notify(const uint8_t* data, int len, uint16_t offset)
{
  VERIFY( _properties.notify, NRF_ERROR_INVALID_PARAM);

  // Must already connected for enabled to be tru
  if ( notifyEnabled() )
  {
    // Whether Txbuf available or not we still update the chars'value
    (void) Bluefruit.txbuf_get(100);

    // could not exceed max len
    uint16_t actual_len = min16(len, _max_len);

    ble_gatts_hvx_params_t hvx_params =
    {
        .handle = _handles.value_handle,
        .type   = BLE_GATT_HVX_NOTIFICATION,
        .offset = offset,
        .p_len  = &actual_len,
        .p_data =  (uint8_t*) data,
    };

    VERIFY_STATUS( sd_ble_gatts_hvx(Bluefruit.connHandle(), &hvx_params) );
  }
  else
  {
    write(data, len);
  }

  return ERROR_NONE;
}

err_t BLECharacteristic::notify(const uint8_t* data, int len)
{
  return notify(data, len, 0);
}

err_t BLECharacteristic::notify(const char * str)
{
  return notify( (const uint8_t*) str, strlen(str) );
}

err_t BLECharacteristic::notify(int num)
{
  return notify( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::notify(uint32_t num)
{
  return notify( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::notify(uint16_t num)
{
  return notify( (uint8_t*) &num, sizeof(num));
}

err_t BLECharacteristic::notify(uint8_t  num)
{
  return notify( (uint8_t*) &num, sizeof(num));
}
