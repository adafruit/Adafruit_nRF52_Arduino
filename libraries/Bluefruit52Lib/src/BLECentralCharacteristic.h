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

// Forward declaration
class BLEClientService;

class BLECentralCharacteristic
{
  public:
    typedef void (*notify_cb_t  ) (BLECentralCharacteristic& chr, uint8_t* data, uint16_t len);
    typedef void (*indicate_cb_t) (BLECentralCharacteristic& chr, uint8_t* data, uint16_t len);

    BLEUuid uuid;

    BLECentralCharacteristic(void);
    BLECentralCharacteristic(BLEUuid bleuuid);

    void assign(ble_gattc_char_t* gattc_chr);
    bool discoverDescriptor(uint16_t conn_handle);
    void begin(void);

    uint16_t valueHandle();
    BLEClientService& parentService(void);

    /*------------- Read -------------*/
    uint16_t read(void* buffer, int bufsize);

    /*------------- Write -------------*/
    uint16_t write     (const void* data, int len);
    uint16_t write_resp(const void* data, int len);

    /*------------- Notify -------------*/
    bool writeCCCD       (uint16_t value);

    bool enableNotify    (void);
    bool disableNotify   (void);

    bool enableIndicate  (void);
    bool disableIndicate (void);

    /*------------- Callbacks -------------*/
    void setNotifyCallback(notify_cb_t fp);
    void useAdaCallback(bool enabled);

  private:
    ble_gattc_char_t   _chr;
    uint16_t           _cccd_handle;

    BLEClientService* _service;
    notify_cb_t        _notify_cb;
    bool               _use_AdaCallback; // whether callback is invoked in seperated task with AdaCallback

    SemaphoreHandle_t  _sem;

    void     _init         (void);
    void     _eventHandler (ble_evt_t* event);

    friend class BLEGatt;
};

#endif /* BLECENTRALCHARACTERISTIC_H_ */
