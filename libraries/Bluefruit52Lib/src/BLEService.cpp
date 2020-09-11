/**************************************************************************/
/*!
    @file     BLEService.cpp
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

BLEService* BLEService::lastService = NULL;

void BLEService::_init(void)
{
  _handle = BLE_GATT_HANDLE_INVALID;
  _read_perm = SECMODE_OPEN;
  _write_perm = SECMODE_OPEN;
}

BLEService::BLEService(void)
  : uuid()
{
  _init();
}

BLEService::BLEService(BLEUuid bleuuid)
  : uuid(bleuuid)
{
  _init();
}

void BLEService::setUuid(BLEUuid bleuuid)
{
  uuid = bleuuid;
}

void BLEService::setPermission(SecureMode_t read_perm, SecureMode_t write_perm)
{
  _read_perm = read_perm;
  _write_perm = write_perm;
}

void BLEService::getPermission(SecureMode_t* read_perm, SecureMode_t* write_perm)
{
  *read_perm = _read_perm;
  *write_perm = _write_perm;
}

err_t BLEService::begin(void)
{
  // Add UUID128 if needed
  (void) uuid.begin();

  uint16_t handle;
  VERIFY_STATUS( sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid._uuid, &handle) );

  lastService = this;
  (void) Bluefruit.Gatt._addService(this);

  return ERROR_NONE;
}

void BLEService::svc_disconnect_hdl(uint16_t conn_hdl)
{
  // Template for inherited class
}

void BLEService::svc_connect_hdl(uint16_t conn_hdl)
{
  // Template for inherited class
}

