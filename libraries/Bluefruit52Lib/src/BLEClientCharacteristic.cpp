/**************************************************************************/
/*!
    @file     BLEClientCharacteristic.cpp
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

void BLEClientCharacteristic::_init(void)
{
  varclr(&_chr);
  _chr.handle_value = BLE_GATT_HANDLE_INVALID;
  _cccd_handle      = BLE_GATT_HANDLE_INVALID;

  _notify_cb       = NULL;
  _indicate_cb     = NULL;

  varclr(&_use_ada_cb);
}

BLEClientCharacteristic::BLEClientCharacteristic(void)
  : uuid(), _adamsg()
{
  _init();
}

BLEClientCharacteristic::BLEClientCharacteristic(BLEUuid bleuuid)
  : uuid(bleuuid), _adamsg()
{
  _init();
}

/**
 * Destructor
 */
BLEClientCharacteristic::~BLEClientCharacteristic()
{
  _adamsg.stop();
  Bluefruit.Gatt._removeCharacteristic(this);
}

void BLEClientCharacteristic::begin(BLEClientService* parent_svc)
{
  // Add UUID128 if needed
  (void) uuid.begin();

  // Use the last (discovered) service as parent if not provided
  _service = ( parent_svc == NULL ) ? BLEClientService::lastService : parent_svc;

  // Register to Bluefruit (required for callback and write response)
  (void) Bluefruit.Gatt._addCharacteristic(this);

  _adamsg.begin(true);
}

void BLEClientCharacteristic::_assign(ble_gattc_char_t* gattc_chr)
{
  _chr = *gattc_chr;
}

void BLEClientCharacteristic::disconnect(void)
{
  _chr.handle_value = BLE_GATT_HANDLE_INVALID;
}


bool BLEClientCharacteristic::discover(void)
{
  ble_gattc_handle_range_t bck_range = Bluefruit.Discovery.getHandleRange();

  // Set discovery handle to parent's service
  Bluefruit.Discovery.setHandleRange( _service->getHandleRange() );

  bool result = Bluefruit.Discovery.discoverCharacteristic( _service->connHandle(), *this) > 0;

  // Set back to previous
  Bluefruit.Discovery.setHandleRange(bck_range);

  return result;
}

bool BLEClientCharacteristic::discovered(void)
{
  return _chr.handle_value != BLE_GATT_HANDLE_INVALID;
}

uint16_t BLEClientCharacteristic::connHandle(void)
{
  return _service->connHandle();
}

uint16_t BLEClientCharacteristic::valueHandle(void)
{
  return _chr.handle_value;
}

uint8_t BLEClientCharacteristic::properties(void)
{
  uint8_t u8;
  memcpy(&u8, &_chr.char_props, 1);
  return u8;
}

BLEClientService& BLEClientCharacteristic::parentService (void)
{
  return *_service;
}

bool BLEClientCharacteristic::_discoverDescriptor(uint16_t conn_handle, ble_gattc_handle_range_t hdl_range)
{
  enum { MAX_DESCIRPTORS = 8 };

  struct {
    uint16_t count;
    ble_gattc_desc_t descs[MAX_DESCIRPTORS];
  }disc_rsp;

  uint16_t count = Bluefruit.Discovery._discoverDescriptor(conn_handle, (ble_gattc_evt_desc_disc_rsp_t*) &disc_rsp, sizeof(disc_rsp), hdl_range);

  // only care CCCD for now
  for(uint16_t i=0; i<count; i++)
  {
    if ( disc_rsp.descs[i].uuid.type == BLE_UUID_TYPE_BLE &&
         disc_rsp.descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG )
    {
      LOG_LV2("DISC", "Found CCCD: handle = %d", disc_rsp.descs[i].handle);
      _cccd_handle = disc_rsp.descs[i].handle;

      break;
    }
  }

  return true;
}

/*------------------------------------------------------------------*/
/* READ
 *------------------------------------------------------------------*/
uint16_t BLEClientCharacteristic::read(void* buffer, uint16_t bufsize)
{
  VERIFY( _chr.char_props.read, 0 );

  BLEConnection* conn = Bluefruit.Connection( _service->connHandle() );
  VERIFY(conn, 0);

  uint16_t const max_payload = conn->getMtu() - 3;

  _adamsg.prepare(buffer, bufsize);
  VERIFY_STATUS( sd_ble_gattc_read(_service->connHandle(), _chr.handle_value, 0), 0);
  int32_t rxlen = _adamsg.waitUntilComplete( (bufsize/(max_payload-2) + 1) * BLE_GENERIC_TIMEOUT );

  return (rxlen < 0) ? 0 : rxlen;
}

uint8_t BLEClientCharacteristic::read8 (void)
{
  uint8_t num;
  return read(&num, sizeof(num)) ? num : 0;
}

uint16_t BLEClientCharacteristic::read16(void)
{
  uint16_t num;
  return read(&num, sizeof(num)) ? num : 0;
}

uint32_t BLEClientCharacteristic::read32(void)
{
  uint32_t num;
  return read(&num, sizeof(num)) ? num : 0;
}

/*------------------------------------------------------------------*/
/* WRITE
 *------------------------------------------------------------------*/
uint16_t BLEClientCharacteristic::write_resp(const void* data, uint16_t len)
{
  VERIFY( _chr.char_props.write, 0 );

  BLEConnection* conn = Bluefruit.Connection( _service->connHandle() );
  VERIFY(conn, 0);

  uint16_t const max_payload = conn->getMtu() - 3;

  const bool long_write = (len > max_payload);
  int32_t count = 0;

  // CMD WRITE_REQUEST for single transaction
  if ( !long_write )
  {
    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = 0,
        .handle   = _chr.handle_value,
        .offset   = 0,
        .len      = len,
        .p_value  = (uint8_t*) data
    };

    _adamsg.prepare( (void*) data, len);
    VERIFY_STATUS(sd_ble_gattc_write(_service->connHandle(), &param), 0);

    // len is always 0 in BLE_GATTC_EVT_WRITE_RSP for BLE_GATT_OP_WRITE_REQ
    count = (_adamsg.waitUntilComplete(BLE_GENERIC_TIMEOUT) < 0 ? 0 : len);
  }
  else
  {
    /*------------- Long Write Sequence -------------*/
    // For BLE_GATT_OP_PREP_WRITE_REQ, 2 bytes are used for offset
    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_PREP_WRITE_REQ,
        .flags    = 0,
        .handle   = _chr.handle_value,
        .offset   = 0,
        .len      = min16(len, max_payload-2),
        .p_value  = (uint8_t*) data
    };

    _adamsg.prepare( (void*) data, len);
    VERIFY_STATUS(sd_ble_gattc_write(_service->connHandle(), &param), 0);
    count = _adamsg.waitUntilComplete( (len/(max_payload-2) + 1) * BLE_GENERIC_TIMEOUT );

    // delay to swallow last WRITE RESPONSE
    // delay(20);
  }

  return (count < 0) ? 0 : count;
}

uint16_t BLEClientCharacteristic::write8_resp(uint8_t value)
{
  return write_resp(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write16_resp(uint16_t value)
{
  return write_resp(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write32_resp(uint32_t value)
{
  return write_resp(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write32_resp(int value)
{
  return write32_resp((uint32_t) value);
}

uint16_t BLEClientCharacteristic::write(const void* data, uint16_t len)
{
//  VERIFY( _chr.char_props.write_wo_resp, 0 );

  BLEConnection* conn = Bluefruit.Connection( _service->connHandle() );
  VERIFY(conn, 0);

  uint16_t const max_payload = conn->getMtu() - 3;
  const uint8_t* u8data = (const uint8_t*) data;

  // Break into multiple packet if needed
  uint16_t remaining = len;
  while( remaining )
  {
    // TODO only Write without response consume a TX buffer
    if ( !conn->getWriteCmdPacket() ) break;

    uint16_t packet_len = min16(max_payload, remaining);

    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD ,
        .flags    = 0                     , // not used with BLE_GATT_OP_WRITE_CMD
        .handle   = _chr.handle_value     ,
        .offset   = 0                     , // not used with BLE_GATT_OP_WRITE_CMD
        .len      = packet_len            ,
        .p_value  = (uint8_t* ) u8data
    };

    VERIFY_STATUS( sd_ble_gattc_write(_service->connHandle(), &param), len-remaining);

    remaining -= packet_len;
    u8data    += packet_len;
  }

  return len-remaining;
}

uint16_t BLEClientCharacteristic::write8(uint8_t value)
{
  return write(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write16(uint16_t value)
{
  return write(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write32(uint32_t value)
{
  return write(&value, sizeof(value));
}

uint16_t BLEClientCharacteristic::write32(int value)
{
  return write32( (uint32_t) value);
}

void BLEClientCharacteristic::setNotifyCallback(notify_cb_t fp, bool useAdaCallback)
{
  _notify_cb = fp;
  _use_ada_cb.notify = useAdaCallback;
}

void BLEClientCharacteristic::setIndicateCallback(indicate_cb_t fp, bool useAdaCallback)
{
  _indicate_cb = fp;
  _use_ada_cb.indicate = useAdaCallback;
}

bool BLEClientCharacteristic::writeCCCD(uint16_t value)
{
  const uint16_t conn_handle = _service->connHandle();

  ble_gattc_write_params_t param =
  {
      .write_op = BLE_GATT_OP_WRITE_CMD,
      .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
      .handle   = _cccd_handle,
      .offset   = 0,
      .len      = 2,
      .p_value  = (uint8_t*) &value
  };

  // TODO only Write without response consume a TX buffer
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  VERIFY( conn && conn->getWriteCmdPacket() );

  VERIFY_STATUS( sd_ble_gattc_write(conn_handle, &param), false );

  return true;
}

bool BLEClientCharacteristic::enableNotify(void)
{
  VERIFY( _chr.char_props.notify );
  return writeCCCD(0x0001);
}

bool BLEClientCharacteristic::disableNotify(void)
{
  VERIFY( _chr.char_props.notify );
  return writeCCCD(0x0000);
}

bool BLEClientCharacteristic::enableIndicate  (void)
{
  VERIFY( _chr.char_props.indicate );
  return writeCCCD(0x0002);
}

bool BLEClientCharacteristic::disableIndicate (void)
{
  VERIFY( _chr.char_props.indicate );
  return writeCCCD(0x0000);
}

void BLEClientCharacteristic::_eventHandler(ble_evt_t* evt)
{
  const uint16_t evt_conn_hdl = evt->evt.common_evt.conn_handle;
  uint16_t gatt_status = evt->evt.gattc_evt.gatt_status;

  switch(evt->header.evt_id)
  {
    case BLE_GATTC_EVT_HVX:
    {
      ble_gattc_evt_hvx_t* hvx = &evt->evt.gattc_evt.params.hvx;

      switch ( hvx->type )
      {
        case BLE_GATT_HVX_NOTIFICATION:
          if (_notify_cb)
          {
            // use AdaCallback or invoke directly
            if ( !(_use_ada_cb.notify &&
                   ada_callback(hvx->data, hvx->len, _notify_cb, this, hvx->data, hvx->len)) )
            {
              _notify_cb(this, hvx->data, hvx->len);
            }
          }
        break;

        case BLE_GATT_HVX_INDICATION:
          if (_indicate_cb)
          {
            // use AdaCallback or invoke directly
            if ( !(_use_ada_cb.indicate &&
                   ada_callback(hvx->data, hvx->len, _indicate_cb, this, hvx->data, hvx->len)) )
            {
              _indicate_cb(this, hvx->data, hvx->len);
            }

            // Send confirmation to server
            VERIFY_STATUS( sd_ble_gattc_hv_confirm(evt_conn_hdl, hvx->handle), );
          }
        break;

        default : break;
      }
    }
    break;

    case BLE_GATTC_EVT_WRITE_RSP:
    {
      ble_gattc_evt_write_rsp_t* wr_rsp = (ble_gattc_evt_write_rsp_t*) &evt->evt.gattc_evt.params.write_rsp;

      // Give up if failed
      if ( gatt_status != BLE_GATT_STATUS_SUCCESS )
      {
        _adamsg.complete();
        break;
      }

      _adamsg.feed(NULL, wr_rsp->len);

      if ( wr_rsp->write_op == BLE_GATT_OP_WRITE_REQ)
      {
        // len is known to be zero for WRITE_REQ
        _adamsg.complete();
      }
      else if ( wr_rsp->write_op == BLE_GATT_OP_PREP_WRITE_REQ)
      {
        BLEConnection* conn = Bluefruit.Connection( _service->connHandle() );

        uint16_t const max_payload = conn->getMtu() - 3;
        uint16_t packet_len = min16(_adamsg.remaining, max_payload-2);

        if ( packet_len )
        {
          // still has data, continue to prepare Long Write sequence
          ble_gattc_write_params_t param =
          {
              .write_op = BLE_GATT_OP_PREP_WRITE_REQ,
              .flags    = 0,
              .handle   = _chr.handle_value,
              .offset   = (uint16_t) (wr_rsp->offset + wr_rsp->len),
              .len      = packet_len,
              .p_value  = (uint8_t*) _adamsg.buffer
          };

          // give up if cannot write
          if ( ERROR_NONE != sd_ble_gattc_write(_service->connHandle(), &param) )
          {
            _adamsg.complete();
          }
        }else
        {
          // All data is prepared, execute Long Write
          ble_gattc_write_params_t param =
          {
              .write_op = BLE_GATT_OP_EXEC_WRITE_REQ,
              .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
              .handle   = _chr.handle_value
          };

          sd_ble_gattc_write(_service->connHandle(), &param);

          // Last BLE_GATTC_EVT_WRITE_RSP for BLE_GATT_OP_EXEC_WRITE_REQ does not
          // contain characteristic's handle. Therefore BLEGatt couldn't forward the
          // event to us. Just skip the wait for now
          _adamsg.complete();
        }
      }else
      {
        // BLE_GATT_OP_EXEC_WRITE_REQ wont reach here due to the handle = 0 issue
      }
    }
    break;

    case BLE_GATTC_EVT_READ_RSP:
    {
      BLEConnection* conn = Bluefruit.Connection( _service->connHandle() );

      uint16_t const max_payload = conn->getMtu() - 3;
      ble_gattc_evt_read_rsp_t* rd_rsp = (ble_gattc_evt_read_rsp_t*) &evt->evt.gattc_evt.params.read_rsp;

      // Give up if failed (BLE_GATT_STATUS_ATTERR_INVALID_OFFSET usually)
      if ( gatt_status != BLE_GATT_STATUS_SUCCESS )
      {
        _adamsg.complete();
        break;
      }

      _adamsg.feed(rd_rsp->data, rd_rsp->len);

      /* Complete condition is one of follows
       * - Running out of buffer
       * - Receive data less than MPS - 1
       * - Couldn't perform GATTC Read
       */
      if (( _adamsg.remaining == 0)    ||
          (rd_rsp->len < (max_payload-1) ) ||
          (ERROR_NONE != sd_ble_gattc_read(_service->connHandle(), _chr.handle_value, _adamsg.xferlen)) )
      {
        _adamsg.complete();
      }
    }
    break;

    default: break;
  }
}

