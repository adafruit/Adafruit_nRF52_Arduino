/**************************************************************************/
/*!
    @file     BLECentralCharacteristic.cpp
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

#define MAX_DESCIRPTORS         8
#define CENTRAL_CHR_TIMEOUT     100

void BLECentralCharacteristic::_init(void)
{
  varclr(&_chr);
  _cccd_handle = 0;

  _notify_cb = NULL;
  _sem       = NULL;
}

BLECentralCharacteristic::BLECentralCharacteristic(void)
  : uuid()
{
  _init();
}

BLECentralCharacteristic::BLECentralCharacteristic(BLEUuid bleuuid)
  : uuid(bleuuid)
{
  _init();
}

void BLECentralCharacteristic::assign(ble_gattc_char_t* gattc_chr)
{
  _chr = *gattc_chr;
}

uint16_t BLECentralCharacteristic::valueHandle()
{
  return _chr.handle_value;
}

BLECentralService& BLECentralCharacteristic::parentService (void)
{
  return *_service;
}

bool BLECentralCharacteristic::discoverDescriptor(uint16_t conn_handle)
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
      LOG_LV1(Discovery, "Found CCDD: handle = %d", disc_rsp.descs[i].handle);
      _cccd_handle = disc_rsp.descs[i].handle;
    }
  }

  return true;
}

void BLECentralCharacteristic::begin(void)
{
  // Add UUID128 if needed
  uuid.begin();

  _service = BLECentralService::lastService;

  // Register to Bluefruit (required for callback and write response)
  (void) Bluefruit.Gatt._addCharacteristic(this);
}

/*------------------------------------------------------------------*/
/* READ
 *------------------------------------------------------------------*/
uint16_t BLECentralCharacteristic::read(void* buffer, int bufsize)
{
//  VERIFY_STATUS( sd_ble_gattc_read(Bluefruit.Central.connHandle(), _chr.handle_value, offset), 0 );

//  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* WRITE
 *------------------------------------------------------------------*/
uint16_t BLECentralCharacteristic::write_resp(const void* data, int len)
{
  // Break into multiple MTU-3 packet
  // TODO Currently SD132 v2.0 MTU is fixed with max payload = 20
  // SD132 v3.0 could negotiate MTU to higher number
  const uint16_t MTU_MPS = 20;

  const uint8_t* u8data = (const uint8_t*) data;

  // Write Response requires to wait for BLE_GATTC_EVT_WRITE_RSP event
  _sem = xSemaphoreCreateBinary();

  int remaining = len;
  while( remaining )
  {
    // Write Req need to wait for response
    uint16_t packet_len = min16(MTU_MPS, remaining);

    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = _chr.handle_value,
        .offset   = 0, // For WRITE_CMD and WRITE_REQ, offset must be 0.
        .len      = packet_len,
        .p_value  = (uint8_t* ) u8data
    };

    if ( ERROR_NONE != sd_ble_gattc_write(Bluefruit.Central.connHandle(), &param) ) break;

    if ( !xSemaphoreTake(_sem, CENTRAL_CHR_TIMEOUT) ) break;

    remaining -= packet_len;
    u8data    += packet_len;
  }

  vSemaphoreDelete(_sem);
  _sem = NULL;

  return len - remaining;
}

uint16_t BLECentralCharacteristic::write(const void* data, int len)
{
  // Break into multiple MTU-3 packet
  // TODO Currently SD132 v2.0 MTU is fixed with max payload = 20
  // SD132 v3.0 could negotiate MTU to higher number
  const uint16_t MTU_MPS = 20;

  const uint16_t conn_handle = _service->connHandle();

  const uint8_t* u8data = (const uint8_t*) data;

  int remaining = len;
  while( remaining )
  {
    // Write CMD consume a TX buffer
    if ( !Bluefruit.Gap.getTxPacket(conn_handle) )  return BLE_ERROR_NO_TX_PACKETS;

    uint16_t packet_len = min16(MTU_MPS, remaining);

    ble_gattc_write_params_t param =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = _chr.handle_value,
        .offset   = 0, // For WRITE_CMD and WRITE_REQ, offset must be 0.
        .len      = packet_len,
        .p_value  = (uint8_t* ) u8data
    };

    VERIFY_STATUS( sd_ble_gattc_write(conn_handle, &param), len - remaining );

    remaining -= packet_len;
    u8data    += packet_len;
  }

  return len;
}

void BLECentralCharacteristic::setNotifyCallback(notify_cb_t fp)
{
  _notify_cb = fp;
}

bool BLECentralCharacteristic::writeCCCD(uint16_t value)
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

bool BLECentralCharacteristic::enableNotify(void)
{
  VERIFY( _chr.char_props.notify );
  return writeCCCD(0x0001);
}

bool BLECentralCharacteristic::disableNotify(void)
{
  return writeCCCD(0x0000);
}

bool BLECentralCharacteristic::enableIndicate  (void)
{
  VERIFY( _chr.char_props.indicate );
  return writeCCCD(0x0002);
}

bool BLECentralCharacteristic::disableIndicate (void)
{
  return writeCCCD(0x0000);
}

void BLECentralCharacteristic::_eventHandler(ble_evt_t* evt)
{
  switch(evt->header.evt_id)
  {
    case BLE_GATTC_EVT_HVX:
    {
      ble_gattc_evt_hvx_t* hvx = &evt->evt.gattc_evt.params.hvx;

      if ( hvx->type == BLE_GATT_HVX_NOTIFICATION )
      {
        if (_notify_cb) _notify_cb(*this, hvx->data, hvx->len);
      }else
      {

      }
    }
    break;

    case BLE_GATTC_EVT_WRITE_RSP:
    {
      ble_gattc_evt_write_rsp_t* wr_rsp = (ble_gattc_evt_write_rsp_t*) &evt->evt.gattc_evt.params.write_rsp;

      if (wr_rsp->write_op == BLE_GATT_OP_WRITE_REQ &&_sem)
      {
        // TODO check evt->evt.gattc_evt.gatt_status
        xSemaphoreGive(_sem);
      }
    }
    break;

    default: break;
  }
}

