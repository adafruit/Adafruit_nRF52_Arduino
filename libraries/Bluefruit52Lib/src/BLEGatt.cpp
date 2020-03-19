/**************************************************************************/
/*!
    @file     BLEGatt.cpp
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

  BLEConnection* conn = Bluefruit.Connection(conn_hdl);

  while( _adamsg.isWaiting() )
  {
    delay( conn->getConnectionInterval() );
  }

  _adamsg.begin(true);
  _adamsg.prepare(buffer, bufsize);

  err_t err = sd_ble_gattc_char_value_by_uuid_read(conn_hdl, &bleuuid._uuid, &hdl_range);

  if( NRF_SUCCESS == err )
  {
    // Read by uuid could take long if the uuid handle is far from start handle
    count = _adamsg.waitUntilComplete(5*BLE_GENERIC_TIMEOUT);
  }else
  {
    VERIFY_MESS(err, dbg_err_str);
  }
  _adamsg.stop();


  return (count < 0) ? 0 : count;
}

void BLEGatt::_eventHandler(ble_evt_t* evt)
{
  // conn handle has fixed offset regardless of event type
  const uint16_t evt_conn_hdl = evt->evt.common_evt.conn_handle;
  const uint16_t evt_id       = evt->header.evt_id;

  BLEConnection* conn = Bluefruit.Connection(evt_conn_hdl);

  /*------------- Server service -------------*/
  if ( evt_id == BLE_GAP_EVT_DISCONNECTED ||  evt_id == BLE_GAP_EVT_CONNECTED )
  {
    for(uint8_t i=0; i<_server.svc_count; i++)
    {
      if ( evt_id == BLE_GAP_EVT_DISCONNECTED )
      {
        _server.svc_list[i]->svc_disconnect_hdl(evt_conn_hdl);
      }else
      {
        _server.svc_list[i]->svc_connect_hdl(evt_conn_hdl);
      }
    }
  }

  /*------------- Server Characteristics -------------*/
  for(uint8_t i=0; i<_server.chr_count; i++)
  {
    BLECharacteristic* chr = _server.chr_list[i];
    uint16_t req_handle = BLE_GATT_HANDLE_INVALID;

    // BLE_GATTS_OP_EXEC_WRITE_REQ_NOW has no handle, uuid only command op
    bool exec_write_now = false;

    switch (evt_id)
    {
      case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
      {
        // Handle has the same offset for read & write request
        req_handle = evt->evt.gatts_evt.params.authorize_request.request.read.handle;

        if ( evt->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE )
        {
          ble_gatts_evt_write_t * wr_req = &evt->evt.gatts_evt.params.authorize_request.request.write;
          if ( wr_req->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW )
          {
            exec_write_now = true;
          }
        }
      }
      break;

      case BLE_GATTS_EVT_WRITE:
        req_handle = evt->evt.gatts_evt.params.write.handle;
        break;

      default: break;
    }

    // invoke characteristic handler if matched
    if ( exec_write_now ||
        ((req_handle != BLE_GATT_HANDLE_INVALID) && (req_handle == chr->handles().value_handle || req_handle == chr->handles().cccd_handle)) )
    {
      chr->_eventHandler(evt);

      // Save CCCD if paired
      if ( conn->secured() && (evt_id == BLE_GATTS_EVT_WRITE) && (req_handle == chr->handles().cccd_handle) )
      {
        conn->saveCccd();
      }
    }
  }

  /*------------- Client Characteristics -------------*/
  for(int i=0; i<_client.chr_count; i++)
  {
    if ( evt_conn_hdl == _client.chr_list[i]->connHandle() )
    {
      BLEClientCharacteristic* chr = _client.chr_list[i];
      uint16_t req_handle = BLE_GATT_HANDLE_INVALID;

      switch(evt_id)
      {
        case BLE_GATTC_EVT_HVX:
          req_handle = evt->evt.gattc_evt.params.hvx.handle;
        break;

        case BLE_GATTC_EVT_WRITE_RSP:
          req_handle = evt->evt.gattc_evt.params.write_rsp.handle;
        break;

        case BLE_GATTC_EVT_READ_RSP:
          req_handle = evt->evt.gattc_evt.params.read_rsp.handle;
        break;

        default: break;
      }

      // invoke characteristic handler if matched
      if ( (req_handle != BLE_GATT_HANDLE_INVALID) && (chr->valueHandle() == req_handle) )
      {
        chr->_eventHandler(evt);
      }
    }
  }

  // disconnect Client Services & Characteristics of the disconnected handle
  if ( evt_id == BLE_GAP_EVT_DISCONNECTED )
  {
    // Client Chr
    for(uint8_t i=0; i<_client.chr_count; i++)
    {
      if ( evt_conn_hdl == _client.chr_list[i]->connHandle() )
      {
        _client.chr_list[i]->disconnect();
      }
    }

    // Client Service TODO merge to above loop
    for(uint8_t i=0; i<_client.svc_count; i++)
    {
      if ( evt_conn_hdl == _client.svc_list[i]->_conn_hdl)
      {
        _client.svc_list[i]->disconnect();
      }
    }
  }

  // GATTC Read Characteristic by UUID procedure
  switch ( evt_id )
  {
    case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
    {
      ble_gattc_evt_char_val_by_uuid_read_rsp_t* rd_rsp = &evt->evt.gattc_evt.params.char_val_by_uuid_read_rsp;

      if (rd_rsp->count)
      {
        ble_gattc_handle_value_t hdl_value = { 0, nullptr };

        if ( ERROR_NONE == sd_ble_gattc_evt_char_val_by_uuid_read_rsp_iter(&evt->evt.gattc_evt, &hdl_value) )
        {
          _adamsg.feed(hdl_value.p_value, rd_rsp->value_len);
        }

        _adamsg.complete();
      }
    }
    break;

    default: break;
  }
}

/*------------------------------------------------------------------*/
/* Server
 *------------------------------------------------------------------*/
bool BLEGatt::_addCharacteristic(BLECharacteristic* chr)
{
  VERIFY( _server.chr_count < CFG_GATT_MAX_SERVER_CHARS );
  _server.chr_list[ _server.chr_count++ ] = chr;

  return true;
}

bool BLEGatt::_addService(BLEService* svc)
{
  VERIFY( _server.svc_count < CFG_GATT_MAX_SERVER_SERVICE );
  _server.svc_list[ _server.svc_count++ ] = svc;

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
  VERIFY( _client.chr_count < CFG_GATT_MAX_CLIENT_CHARS );
  _client.chr_list[ _client.chr_count++ ] = chr;

  return true;
}

bool BLEGatt::_addService(BLEClientService* svc)
{
  VERIFY( _client.svc_count < CFG_GATT_MAX_CLIENT_SERVICE );
  _client.svc_list[ _client.svc_count++ ] = svc;

  return true;
}
