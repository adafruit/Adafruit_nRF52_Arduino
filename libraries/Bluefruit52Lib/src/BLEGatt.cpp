/**************************************************************************/
/*!
    @file     BLEGatt.cpp
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


BLEGatt::BLEGatt(void)
  : _adamsg()
{
  varclr(&_server);
  varclr(&_client);
}

uint16_t BLEGatt::readCharByUuid(uint16_t conn_hdl, BLEUuid bleuuid, void* buffer, uint16_t bufsize, uint16_t start_hdl, uint16_t end_hdl)
{
  int32_t count = 0;
  ble_gattc_handle_range_t hdl_range = { .start_handle = start_hdl, .end_handle = end_hdl };

  _adamsg.begin(true);
  _adamsg.prepare(buffer, bufsize);

  err_t err = sd_ble_gattc_char_value_by_uuid_read(conn_hdl, &bleuuid._uuid, &hdl_range);

  if( NRF_SUCCESS == err )
  {
    // Read by uuid could take long if the uuid handle is far from start handle
    count = _adamsg.waitUntilComplete(5*BLE_GENERIC_TIMEOUT);
  }else
  {
    VERIFY_MESS(err);
  }
  _adamsg.stop();


  return (count < 0) ? 0 : count;
}

void BLEGatt::_eventHandler(ble_evt_t* evt)
{
  // Server Characteristics
  for(int i=0; i<_server.chr_count; i++)
  {
    _server.chr_list[i]->_eventHandler(evt);
  }

  // Client Characteristics
  for(int i=0; i<_client.chr_count; i++)
  {
    bool matched = false;

    switch(evt->header.evt_id)
    {
      case BLE_GATTC_EVT_HVX:
      case BLE_GATTC_EVT_WRITE_RSP:
      case BLE_GATTC_EVT_READ_RSP:
        // write & read & hvc response's handle has same offset from the struct
        matched = (_client.chr_list[i]->_chr.handle_value == evt->evt.gattc_evt.params.write_rsp.handle);
      break;

      default: break;
    }

    // invoke charactersistic handler if matched
    if ( matched ) _client.chr_list[i]->_eventHandler(evt);
  }

  // disconnect Client Service
  if ( evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED )
  {
    for(int i=0; i<_client.svc_count; i++) _client.svc_list[i]->disconnect();
  }

  // GATTC Read Characteristic by UUID procedure
  if ( evt->header.evt_id == BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP )
  {
    ble_gattc_evt_char_val_by_uuid_read_rsp_t* rd_rsp = &evt->evt.gattc_evt.params.char_val_by_uuid_read_rsp;

    if (rd_rsp->count)
    {
      _adamsg.feed(rd_rsp->handle_value[0].p_value, rd_rsp->value_len);
      _adamsg.complete();
    }
  }
}

/*------------------------------------------------------------------*/
/* Server
 *------------------------------------------------------------------*/
bool BLEGatt::_addCharacteristic(BLECharacteristic* chr)
{
  if ( _server.chr_count == BLE_GATT_MAX_SERVER_CHARS ) return false;
  _server.chr_list[ _server.chr_count++ ] = chr;

  return true;
}

/*------------------------------------------------------------------*/
/* Client
 *------------------------------------------------------------------*/
void BLEGatt::_removeCharacteristic(BLEClientCharacteristic* chr)
{
  for(int i=0; i<_client.chr_count; i++)
  {
    // found the char, swap with the last one
    if ( _client.chr_list[i] == chr )
    {
      vTaskSuspendAll();

      _client.chr_count--;

      _client.chr_list[i] = _client.chr_list[ _client.chr_count ];
      _client.chr_list[_client.chr_count] = NULL;

      ( void ) xTaskResumeAll();

      break;
    }
  }
}

bool BLEGatt::_addCharacteristic(BLEClientCharacteristic* chr)
{
  VERIFY( _client.chr_count < BLE_GATT_MAX_CLIENT_CHARS );
  _client.chr_list[ _client.chr_count++ ] = chr;

  return true;
}

bool BLEGatt::_addService(BLEClientService* svc)
{
  VERIFY( _client.svc_count < BLE_GATT_MAX_CLIENT_SERVICE );
  _client.svc_list[ _client.svc_count++ ] = svc;

  return true;
}
