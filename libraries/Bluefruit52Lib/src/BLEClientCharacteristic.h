/**************************************************************************/
/*!
    @file     BLEClientCharacteristic.h
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
    typedef void (*notify_cb_t  ) (BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
    typedef void (*indicate_cb_t) (BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

    BLEUuid uuid;

    // Constructors
    BLEClientCharacteristic(void);
    BLEClientCharacteristic(BLEUuid bleuuid);

    // Destructor
    virtual ~BLEClientCharacteristic();

    void     begin(BLEClientService* parent_svc = NULL);

    bool     discover(void);
    bool     discovered(void);

    uint16_t connHandle(void);
    uint16_t valueHandle(void);
    uint8_t  properties(void);

    BLEClientService& parentService(void);

    /*------------- Read -------------*/
    uint16_t read(void* buffer, uint16_t bufsize);
    uint8_t  read8 (void);
    uint16_t read16(void);
    uint32_t read32(void);

    /*------------- Write without Response-------------*/
    uint16_t write     (const void* data, uint16_t len);
    uint16_t write8    (uint8_t value);
    uint16_t write16   (uint16_t value);
    uint16_t write32   (uint32_t value);
    uint16_t write32   (int      value);

    /*------------- Write with Response-------------*/
    uint16_t write_resp(const void* data, uint16_t len);
    uint16_t write8_resp    (uint8_t value);
    uint16_t write16_resp   (uint16_t value);
    uint16_t write32_resp   (uint32_t value);
    uint16_t write32_resp   (int      value);

    /*------------- Notify -------------*/
    bool writeCCCD       (uint16_t value);

    bool enableNotify    (void);
    bool disableNotify   (void);

    bool enableIndicate  (void);
    bool disableIndicate (void);

    /*------------- Callbacks -------------*/
    void setNotifyCallback(notify_cb_t fp, bool useAdaCallback = true);
    void setIndicateCallback(indicate_cb_t fp, bool useAdaCallback = true);

    /*------------- Internal usage -------------*/
    void _assign(ble_gattc_char_t* gattc_chr);
    bool _discoverDescriptor(uint16_t conn_handle, ble_gattc_handle_range_t hdl_range);

  private:
    ble_gattc_char_t  _chr;
    uint16_t          _cccd_handle;

    BLEClientService* _service;
    AdaMsg             _adamsg;

    /*------------- Callbacks -------------*/
    notify_cb_t       _notify_cb;
    indicate_cb_t     _indicate_cb;

    struct ATTR_PACKED {
      uint8_t notify    : 1;
      uint8_t indicate : 1;
    } _use_ada_cb;

    void  _init         (void);
    void  _eventHandler (ble_evt_t* event);

    void disconnect(void);

    friend class BLEGatt;
};

#endif /* BLECLIENTCHARACTERISTIC_H_ */
