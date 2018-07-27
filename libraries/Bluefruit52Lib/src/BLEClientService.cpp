/**************************************************************************/
/*!
    @file     BLEClientService.cpp
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

// Last service that is discovered
BLEClientService* BLEClientService::lastService = NULL;

void BLEClientService::_init(void)
{
  _conn_hdl   = BLE_CONN_HANDLE_INVALID;

  _hdl_range.start_handle = 1;
  _hdl_range.end_handle   = 0xffff;
}

BLEClientService::BLEClientService(void)
  : uuid()
{
  _init();
}

BLEClientService::BLEClientService(BLEUuid bleuuid)
  : uuid(bleuuid)
{
  _init();
}


bool BLEClientService::begin(void)
{
  // Add UUID128 if needed
  (void) uuid.begin();

  lastService = this;
  (void) Bluefruit.Gatt._addService(this);

  return true;
}

bool BLEClientService::discover(uint16_t conn_handle)
{
  // Initialize Discovery module if needed
  if ( !Bluefruit.Discovery.begun() ) Bluefruit.Discovery.begin();

  VERIFY( Bluefruit.Discovery._discoverService(conn_handle, *this) );
  _conn_hdl = conn_handle;
  return true;
}

bool BLEClientService::discovered(void)
{
  return _conn_hdl != BLE_CONN_HANDLE_INVALID;
}

uint16_t BLEClientService::connHandle(void)
{
  return _conn_hdl;
}

void BLEClientService::setHandleRange(ble_gattc_handle_range_t handle_range)
{
  _hdl_range = handle_range;
}

ble_gattc_handle_range_t BLEClientService::getHandleRange(void)
{
  return _hdl_range;
}

void BLEClientService::disconnect(void)
{
  _conn_hdl   = BLE_CONN_HANDLE_INVALID;
  // inherited service may want to clean up its own characteristic
}
