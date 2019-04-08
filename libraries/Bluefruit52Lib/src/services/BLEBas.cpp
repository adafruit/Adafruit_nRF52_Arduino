/**************************************************************************/
/*!
    @file     BLEBas.cpp
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

BLEBas::BLEBas(void) :
  BLEService(UUID16_SVC_BATTERY), _battery(UUID16_CHR_BATTERY_LEVEL)
{

}

err_t BLEBas::begin(void)
{
  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  _battery.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  _battery.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _battery.setFixedLen(1);
  VERIFY_STATUS( _battery.begin() );

  return ERROR_NONE;
}

bool BLEBas::write(uint8_t level)
{
  return _battery.write8(level) > 0;
}

bool BLEBas::notify(uint8_t level)
{
  return _battery.notify8(level);
}

bool BLEBas::notify(uint16_t conn_hdl, uint8_t level)
{
  return _battery.notify8(conn_hdl, level);
}
