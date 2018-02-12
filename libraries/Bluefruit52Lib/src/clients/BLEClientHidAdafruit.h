/**************************************************************************/
/*!
    @file     BLEClientHidAdafruit.h
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
#ifndef BLECLIENTHIDADAFRUIT_H_
#define BLECLIENTHIDADAFRUIT_H_

#include "bluefruit_common.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"

#include "services/BLEHidGeneric.h"

// Only support Boot Keyboard and/or Boot Mouse, there is no Consumer Control support
class BLEClientHidAdafruit : public BLEClientService
{
  public:
    BLEClientHidAdafruit(void);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    bool keyboardPresent(void);
    bool mousePresent(void);

    bool enableKeyboard(void);
    bool disableKeyboard(void);

    bool enableMouse(void);
    bool disableMouse(void);

  private:
    uint8_t _country;

    // Only support Boot protocol for keyboard and Mouse
    BLEClientCharacteristic _protcol_mode;
    BLEClientCharacteristic _hid_info;
    BLEClientCharacteristic _hid_control;

    BLEClientCharacteristic _kbd_boot_input;
    BLEClientCharacteristic _kbd_boot_output;

    BLEClientCharacteristic _mouse_boot_input;
};



#endif /* BLECLIENTHIDADAFRUIT_H_ */
