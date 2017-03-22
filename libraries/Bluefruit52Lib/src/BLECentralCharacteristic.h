/**************************************************************************/
/*!
    @file     BLECentralCharacteristic.h
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
#ifndef BLECENTRALCHARACTERISTIC_H_
#define BLECENTRALCHARACTERISTIC_H_

#include "bluefruit_common.h"
#include "BLEUuid.h"
#include "BLECharacteristic.h"

class BLECentralCharacteristic
{
  public:
    BLEUuid uuid;

    BLECentralCharacteristic(void);
    BLECentralCharacteristic(ble_gattc_char_t* gattc_char);

    bool discoverDescriptor(void);
    void begin(void);

    uint16_t valueHandle();

    /*------------- Read -------------*/
    uint16_t read(void* buffer, int bufsize, uint16_t offset = 0);

    /*------------- Write -------------*/

    /*------------- Notify -------------*/
    bool enableNotify(void);


    /*------------- Callbacks -------------*/
    typedef void (*notify_cb_t  ) (BLECentralCharacteristic& chr, uint8_t* data, uint16_t len);
    typedef void (*indicate_cb_t) (BLECentralCharacteristic& chr, uint8_t* data, uint16_t len);

    void setNotifyCallback(notify_cb_t fp);

  private:
    ble_gattc_char_t _chr;
    uint16_t         _cccd_handle;

    notify_cb_t      _notify_cb;

    void _init(void);
    void _eventHandler(ble_evt_t* event);

    friend class BLECentral;
};

#endif /* BLECENTRALCHARACTERISTIC_H_ */
