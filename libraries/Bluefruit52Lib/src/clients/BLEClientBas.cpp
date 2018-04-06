/**************************************************************************/
/*!
    @file     BLEClientBas.cpp
    @author   swilson

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

BLEClientBas::BLEClientBas(void)
  : BLEClientService(UUID16_SVC_BATTERY)
{

}

bool BLEClientBas::begin(void)
{
  // Invoke base class begin()
  BLEClientService::begin();

  return true;
}

bool BLEClientBas::discover(uint16_t conn_handle)
{
  // Call BLECentralService discover
  VERIFY( BLEClientService::discover(conn_handle) );

  return true;
}

uint8_t BLEClientBas::getLevel(void)
{
  uint8_t level = 0;
  
  ble_gattc_handle_range_t bck_range = Bluefruit.Discovery.getHandleRange();

  // Set discovery handle to BAS service
  Bluefruit.Discovery.setHandleRange(_hdl_range);

  BLEClientCharacteristic chr(UUID16_CHR_BATTERY_LEVEL);
  chr.begin(this);

  if ( Bluefruit.Discovery.discoverCharacteristic(_conn_hdl, chr) )
  {
    level = chr.read8();
  }

  // Set back
  Bluefruit.Discovery.setHandleRange(bck_range);

  return level;
}
