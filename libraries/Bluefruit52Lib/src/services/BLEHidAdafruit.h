/**************************************************************************/
/*!
    @file     BLEHidAdafruit.h
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
#ifndef BLEHIDADAFRUIT_H_
#define BLEHIDADAFRUIT_H_

#include "bluefruit_common.h"
#include "utility/adafruit_fifo.h"

#include "BLECharacteristic.h"
#include "BLEHidGeneric.h"
#include "BLEService.h"


class BLEHidAdafruit : public BLEHidGeneric
{
  protected:

  public:
    BLEHidAdafruit(void);

    virtual err_t start(void);

    // Keyboard
    err_t keyboardReport(hid_keyboard_report_t* report);
    err_t keyboardReport(uint8_t modifier, uint8_t keycode[6]);

    err_t keyHIDCode(uint8_t modifier, uint8_t keycode0, uint8_t keycode1=0, uint8_t keycode2=0, uint8_t keycode3=0, uint8_t keycode4=0, uint8_t keycode5=0);

    err_t keyPress(char ch);
    err_t keyRelease(void);
    err_t keySequence(const char* str, int interal=5);

    // Consumer Media Keys
    err_t consumerReport(uint16_t usage_code);
    err_t consumerKeyPress(uint16_t usage_code);
    err_t consumerKeyRelease(void);
};

#endif /* BLEHIDADAFRUIT_H_ */
