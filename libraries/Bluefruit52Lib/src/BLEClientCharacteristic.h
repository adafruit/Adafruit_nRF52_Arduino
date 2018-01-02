/**************************************************************************/
/*!
    @file     BLEClientCharacteristic.h
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
#ifndef BLECLIENTCHARACTERISTIC_H_
#define BLECLIENTCHARACTERISTIC_H_

#include "bluefruit_common.h"
#include "BLEUuid.h"
#include "BLECharacteristic.h"

// Forward declaration
class BLEClientService;

class BLEClientCharacteristic
{
  public:
    /*--------- Callback Signatures ----------*/
    typedef void (*notify_cb_t  ) (BLEClientCharacteristic& chr, uint8_t* data, uint16_t len);
    typedef void (*indicate_cb_t) (BLEClientCharacteristic& chr, uint8_t* data, uint16_t len);

    BLEUuid uuid;

    BLEClientCharacteristic(void);
    BLEClientCharacteristic(BLEUuid bleuuid);

    virtual ~BLEClientCharacteristic();

    void assign(ble_gattc_char_t* gattc_chr);
    bool discoverDescriptor(uint16_t conn_handle, ble_gattc_handle_range_t hdl_range);

    void begin(BLEClientService* parent_svc = NULL);

    uint16_t connHandle(void);
    uint16_t valueHandle(void);
    uint8_t  properties(void);
    BLEClientService& parentService(void);

    /*------------- Read -------------*/
    uint16_t read(void* buffer, uint16_t bufsize);

    /*------------- Write -------------*/
    uint16_t write     (const void* data, uint16_t len);
    uint16_t write_resp(const void* data, uint16_t len);

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
    bool               _use_AdaCallback; // whether callback is invoked in separated task with AdaCallback

    AdaMsg             _adamsg;

    void  _init         (void);
    void  _eventHandler (ble_evt_t* event);

    void disconnect(void);

    friend class BLEGatt;
};

#endif /* BLECLIENTCHARACTERISTIC_H_ */
