/**************************************************************************/
/*!
    @file     BLEClientCharacteristic.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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
#include "AdaCallback.h"

#define MAX_DESCIRPTORS         8

void BLEClientCharacteristic::_init(void)
{
  varclr(&_chr);
  _cccd_handle     = 0;

  _notify_cb       = NULL;
  _use_AdaCallback = true;
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
  uuid.begin();

  // Use the last (discovered) service as parent if not provided
  _service = ( parent_svc == NULL ) ? BLEClientService::lastService : parent_svc;

  // Register to Bluefruit (required for callback and write response)
  (void) Bluefruit.Gatt._addCharacteristic(this);

  _adamsg.begin(true);
}

void BLEClientCharacteristic::assign(ble_gattc_char_t* gattc_chr)
{
  _chr = *gattc_chr;
}

void BLEClientCharacteristic::useAdaCallback(bool enabled)
{
  _use_AdaCallback = enabled;
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

bool BLEClientCharacteristic::discoverDescriptor(uint16_t conn_handle)
{
  struct {
    uint16_t count;
    ble_gattc_desc_t descs[MAX_DESCIRPTORS];
  }disc_rsp;

  uint16_t count = Bluefruit.Discovery._discoverDescriptor(conn_handle, (ble_gattc_evt_desc_disc_rsp_t*) &disc_rsp, MAX_DESCIRPTORS);

  // only care CCCD for now
  for(uint16_t i=0; i<count; i++)
  {
    if ( disc_rsp.descs[i].uuid.type == BLE_UUID_TYPE_BLE &&
         disc_rsp.descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG )
    {
      LOG_LV2(Discovery, "Found CCDD: handle = %d", disc_rsp.descs[i].handle);
      _cccd_handle = disc_rsp.descs[i].handle;
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

  _adamsg.prepare(buffer, bufsize);
  VERIFY_STATUS( sd_ble_gattc_read(_service->connHandle(), _chr.handle_value, 0), 0);
  int32_t rxlen = _adamsg.waitUntilComplete( (bufsize/(MTU_MPS-2)+1) * BLE_GENERIC_TIMEOUT );

  return (rxlen < 0) ? 0 : rxlen;
}

/*------------------------------------------------------------------*/
/* WRITE
 *------------------------------------------------------------------*/
uint16_t BLEClientCharacteristic::write_resp(const void* data, uint16_t len)
{
  VERIFY( _chr.char_props.write, 0 );

  // TODO Currently SD132 v2.0 MTU is fixed with max payload = 20
  // SD132 v3.0 could negotiate MTU to higher number
  // For BLE_GATT_OP_PREP_WRITE_REQ, max data is 18 ( 2 byte is used for offset)
  const bool long_write = (len > MTU_MPS);
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
    VERIFY_STATUS ( sd_ble_gattc_write(_service->connHandle(), &param) );

    // len is alwasy 0 in BLE_GATTC_EVT_WRITE_RSP for BLE_GATT_OP_WRITE_REQ
    count = (_adamsg.waitUntilComplete(BLE_GENERIC_TIMEOUT) < 0 ? 0 : len);
  }
  else
  {
    /*------------- Long Write Sequence -------------*/
    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_PREP_WRITE_REQ,
        .flags    = 0,
        .handle   = _chr.handle_value,
        .offset   = 0,
        .len      = min16(len, MTU_MPS-2),
        .p_value  = (uint8_t*) data
    };

    _adamsg.prepare( (void*) data, len);
    VERIFY_STATUS ( sd_ble_gattc_write(_service->connHandle(), &param) );
    count = _adamsg.waitUntilComplete( (len/(MTU_MPS-2)+1) * BLE_GENERIC_TIMEOUT );

    // delay to swallow last WRITE RESPONSE
    // delay(20);
  }

  return (count < 0) ? 0 : count;
}

uint16_t BLEClientCharacteristic::write(const void* data, uint16_t len)
{
//  VERIFY( _chr.char_props.write_wo_resp, 0 );

  // Break into multiple MTU-3 packet
  // TODO Currently SD132 v2.0 MTU is fixed with max payload = 20
  // SD132 v3.0 could negotiate MTU to higher number
  const uint8_t* u8data = (const uint8_t*) data;

  uint16_t remaining = len;
  while( remaining )
  {
    // Write CMD consume a TX buffer
    if ( !Bluefruit.Gap.getTxPacket(_service->connHandle()) )  return BLE_ERROR_NO_TX_PACKETS;

    uint16_t packet_len = min16(MTU_MPS, remaining);

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

  return len;
}

void BLEClientCharacteristic::setNotifyCallback(notify_cb_t fp)
{
  _notify_cb = fp;
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

  // Write consume a TX buffer
  if ( !Bluefruit.Gap.getTxPacket(conn_handle) )  return BLE_ERROR_NO_TX_PACKETS;

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
  uint16_t gatt_status = evt->evt.gattc_evt.gatt_status;

  switch(evt->header.evt_id)
  {
    case BLE_GATTC_EVT_HVX:
    {
      ble_gattc_evt_hvx_t* hvx = &evt->evt.gattc_evt.params.hvx;

      if ( hvx->type == BLE_GATT_HVX_NOTIFICATION )
      {
        if (_notify_cb)
        {
          // use AdaCallback or invoke directly
          if (_use_AdaCallback)
          {
            uint8_t* data = (uint8_t*) rtos_malloc(hvx->len);
            if (!data) return;
            memcpy(data, hvx->data, hvx->len);

            ada_callback(data, _notify_cb, this, data, hvx->len);
          }else
          {
            _notify_cb(*this, hvx->data, hvx->len);
          }
        }
      }else
      {

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
      }else if ( wr_rsp->write_op == BLE_GATT_OP_PREP_WRITE_REQ)
      {
        uint16_t packet_len = min16(_adamsg.remaining, MTU_MPS-2);

        // still has data, continue to prepare
        if ( packet_len )
        {
          // Long Write Prepare
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
          // Long Write Execute
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
       * - Receive data less than MPS - 1 (19)
       * - Couldn't perform GATTC Read
       */
      if (( _adamsg.remaining == 0)    ||
          (rd_rsp->len < (MTU_MPS-1) ) ||
          (ERROR_NONE != sd_ble_gattc_read(_service->connHandle(), _chr.handle_value, _adamsg.xferlen)) )
      {
        _adamsg.complete();
      }
    }
    break;

    default: break;
  }
}

