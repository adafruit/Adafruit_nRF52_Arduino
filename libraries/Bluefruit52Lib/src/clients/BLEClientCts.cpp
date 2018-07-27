/**************************************************************************/
/*!
    @file     BLEClientCts.cpp
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

void blects_central_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

BLEClientCts::BLEClientCts(void)
 : BLEClientService(UUID16_SVC_CURRENT_TIME), _cur_time(UUID16_CHR_CURRENT_TIME), _local_info(UUID16_CHR_LOCAL_TIME_INFORMATION)
{
  varclr(&Time);
  varclr(&LocalInfo);

  _adjust_cb = NULL;
}

bool BLEClientCts::begin(void)
{
  VERIFY_STATIC(sizeof(Time) == 10);
  VERIFY_STATIC(sizeof(LocalInfo) == 2);

  // Invoke base class begin()
  BLEClientService::begin();

  _cur_time.begin(this);
  _cur_time.setNotifyCallback(blects_central_notify_cb);

  _local_info.begin(this);

  return true;
}

bool BLEClientCts::discover(uint16_t conn_handle)
{
  // Call Base class discover
  VERIFY( BLEClientService::discover(conn_handle) );
  _conn_hdl = BLE_CONN_HANDLE_INVALID; // make as invalid until we found all chars

  // Discover characteristics
  Bluefruit.Discovery.discoverCharacteristic(conn_handle, _cur_time, _local_info);

  // Current Time chars is mandatory
  VERIFY( _cur_time.valueHandle() != BLE_GATT_HANDLE_INVALID, false);

  _conn_hdl = conn_handle;
  return true;
}

bool BLEClientCts::getCurrentTime(void)
{
  return _cur_time.read(&Time, sizeof(Time)) > 0;
}

bool BLEClientCts::getLocalTimeInfo(void)
{
  return _local_info.read(&LocalInfo, sizeof(LocalInfo)) > 0;
}

bool BLEClientCts::enableAdjust(void)
{
  return _cur_time.enableNotify();
}

void BLEClientCts::setAdjustCallback(adjust_callback_t fp)
{
  _adjust_cb = fp;
}

void BLEClientCts::_cur_time_notify_cb(uint8_t* data, uint16_t len)
{
  memcpy(&Time, data, len);

  // invoked callback if set
  if ( _adjust_cb ) _adjust_cb( Time.adjust_reason );
}

void blects_central_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  ((BLEClientCts&) chr->parentService())._cur_time_notify_cb(data, len);
}
