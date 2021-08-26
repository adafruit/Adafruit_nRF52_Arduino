/**************************************************************************/
/*!
    @file     BLEHidAdafruit.h
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
#ifndef BLEHID_GAMEPAD_H_
#define BLEHID_GAMEPAD_H_

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEHidGeneric.h"
#include "BLEService.h"

class BLEHidGamepad : public BLEHidGeneric
{
  public:

    BLEHidGamepad(void);

    virtual err_t begin(void);

    // Single connection
    bool report(hid_gamepad_report_t const* report);
    bool reportButtons(uint32_t button_mask);
    bool reportHat(uint8_t hat);
    bool reportJoystick(int8_t x, int8_t y, int8_t z, int8_t rz, int8_t rx, int8_t ry);

    // Multiple connections
    bool report(uint16_t conn_hdl, hid_gamepad_report_t const* report);
    bool reportButtons(uint16_t conn_hdl, uint32_t button_mask);
    bool reportHat(uint16_t conn_hdl, uint8_t hat);
    bool reportJoystick(uint16_t conn_hdl, int8_t x, int8_t y, int8_t z, int8_t rz, int8_t rx, int8_t ry);

  protected:

};

#endif /* BLEHID_GAMEPAD_H_ */
