/**************************************************************************/
/*!
    @file     BLEHid.cpp
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

BLEHid::BLEHid(void)
  : BLEService(UUID16_SVC_HUMAN_INTERFACE_DEVICE),
  _chr_protocol(), _chr_input()
{

}

err_t BLEHid::start(void)
{
  VERIFY_STATUS( this->addToGatt() );

  // Protocol Mode
  _chr_protocol.setUuid(UUID16_CHR_PROTOCOL_MODE);
  _chr_protocol.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
  _chr_protocol.setFixedLen(1);
  VERIFY_STATUS( _chr_protocol.start() );
  _chr_protocol.write( (uint8_t) 1);

  // Input reports
  {
    _chr_input.setUuid(UUID16_CHR_REPORT);
    _chr_input.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_input.setFixedLen(sizeof(hid_keyboard_report_t));
    //_chr_input.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    _chr_input.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    _chr_input.setReportRefDescriptor(0x01, 0x01);
    VERIFY_STATUS( _chr_input.start() );
  }

  return NRF_SUCCESS;
}
