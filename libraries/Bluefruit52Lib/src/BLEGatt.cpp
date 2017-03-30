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
{
  varclr(&server);
  varclr(&client);
}

void BLEGatt::_eventHandler(ble_evt_t* evt)
{
  // Server Characteristics
  for(int i=0; i<server._chars_count; i++)
  {
    server._chars_list[i]->_eventHandler(evt);
  }

  // Client Characteristics
  for(int i=0; i<client._chars_count; i++)
  {
    bool matched = false;

    switch(evt->header.evt_id)
    {
      case BLE_GATTC_EVT_HVX:
      case BLE_GATTC_EVT_WRITE_RSP:
      case BLE_GATTC_EVT_READ_RSP:
        // write & read & hvc response's handle has same offset from the struct
        matched = (client._chars_list[i]->_chr.handle_value == evt->evt.gattc_evt.params.write_rsp.handle);
      break;

      default: break;
    }

    // invoke charactersistic handler if matched
    if ( matched ) client._chars_list[i]->_eventHandler(evt);
  }

  // disconnect Client Service
  if ( evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED )
  {
    for(int i=0; i<client._svc_count; i++) client._svc_list[i]->disconnect();
  }
}

bool BLEGatt::_addCharacteristic(BLECharacteristic* chr)
{
  if ( server._chars_count == BLE_GATT_MAX_SERVER_CHARS ) return false;
  server._chars_list[ server._chars_count++ ] = chr;

  return true;
}



bool BLEGatt::_addCharacteristic(BLECentralCharacteristic* chr)
{
  VERIFY( client._chars_count < BLE_GATT_MAX_CLIENT_CHARS );
  client._chars_list[ client._chars_count++ ] = chr;

  return true;
}

bool BLEGatt::_addService(BLECentralService* svc)
{
  VERIFY( client._svc_count < BLE_GATT_MAX_CLIENT_SERVICE );
  client._svc_list[ client._svc_count++ ] = svc;

  return true;
}
