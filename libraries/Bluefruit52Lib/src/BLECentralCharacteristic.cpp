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

#define MAX_DESCIRPTORS   8

void BLECentralCharacteristic::_init(void)
{
  _cccd_handle = 0;

  _notify_cb = NULL;
}

BLECentralCharacteristic::BLECentralCharacteristic(void)
  : uuid()
{
  varclr(&_chr);
  _init();
}

BLECentralCharacteristic::BLECentralCharacteristic(ble_gattc_char_t* gattc_char)
  : uuid(gattc_char->uuid)
{
  _chr = (*gattc_char);
  _init();
}

uint16_t BLECentralCharacteristic::valueHandle()
{
  return _chr.handle_value;
}

BLECentralService& BLECentralCharacteristic::parentService (void)
{
  return *_service;
}

bool BLECentralCharacteristic::discoverDescriptor(void)
{
  struct {
    uint16_t count;
    ble_gattc_desc_t descs[MAX_DESCIRPTORS];
  }disc_rsp;

  uint16_t count = Bluefruit.Central._discoverDescriptor((ble_gattc_evt_desc_disc_rsp_t*) &disc_rsp, MAX_DESCIRPTORS);

  // only care CCCD for now
  for(uint16_t i=0; i<count; i++)
  {
    if ( disc_rsp.descs[i].uuid.type == BLE_UUID_TYPE_BLE &&
         disc_rsp.descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG )
    {
      LOG_LV1(BLECentralUart, "Found CCDD: handle = %d", disc_rsp.descs[i].handle);
      _cccd_handle = disc_rsp.descs[i].handle;
    }
  }

  return true;
}

void BLECentralCharacteristic::begin(void)
{
  _service = BLECentralService::lastService;

  // Currently Only register to Bluefruit if callback is installed
  if ( _notify_cb )
  {
    (void) Bluefruit.Central._registerCharacteristic(this);
  }
}

uint16_t BLECentralCharacteristic::read(void* buffer, int bufsize, uint16_t offset)
{
  VERIFY_STATUS( sd_ble_gattc_read(Bluefruit.Central.connHandle(), _chr.handle_value, offset), 0 );

  return ERROR_NONE;
}

void BLECentralCharacteristic::setNotifyCallback(notify_cb_t fp)
{
  _notify_cb = fp;
}

bool BLECentralCharacteristic::enableNotify(void)
{
  uint16_t value = 0x0001;
  ble_gattc_write_params_t param =
  {
      .write_op = BLE_GATT_OP_WRITE_CMD,
      .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
      .handle   = _cccd_handle,
      .offset   = 0,
      .len      = 2,
      .p_value  = (uint8_t*) &value
  };

  // TODO consume a TX buffer
  VERIFY_STATUS( sd_ble_gattc_write(Bluefruit.Central.connHandle(), &param), false );

  return true;
}

void BLECentralCharacteristic::_eventHandler(ble_evt_t* event)
{
  switch(event->header.evt_id)
  {
    case BLE_GATTC_EVT_HVX:
    {
      ble_gattc_evt_hvx_t* hvx = &event->evt.gattc_evt.params.hvx;

      if ( hvx->type == BLE_GATT_HVX_NOTIFICATION )
      {
        if (_notify_cb) _notify_cb(*this, hvx->data, hvx->len);
      }else
      {

      }
    }
    break;

    default: break;
  }
}
