/**************************************************************************/
/*!
    @file     BLEAncs.h
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
#ifndef BLEANCSLOOPBACK_H_
#define BLEANCSLOOPBACK_H_

#include "../BLEClientCharacteristic.h"
#include "bluefruit_common.h"

#include "BLEClientService.h"

extern const uint8_t BLEANCS_UUID_SERVICE[];
extern const uint8_t BLEANCS_UUID_CHR_CONTROL[];
extern const uint8_t BLEANCS_UUID_CHR_NOTIFICATION[];
extern const uint8_t BLEANCS_UUID_CHR_DATA[];

class BLEAncsLoopback : public BLEClientService
{
  public:
    typedef void (*notification_callback_t) (uint8_t* data, uint16_t len);
    typedef void (*data_callback_t) (uint8_t* data, uint16_t len);

    BLEAncsLoopback(void);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    void setNotificationCallback(notification_callback_t fp);
    void setDataCallback(data_callback_t fp);

    void requestNotificationData(uint8_t* data, uint16_t len);
    
    bool enableNotification(void);
    bool disableNotification(void);

  private:
    BLEClientCharacteristic _control;
    BLEClientCharacteristic _notification;
    BLEClientCharacteristic _data;

    notification_callback_t _notif_cb;
    data_callback_t _data_cb;

    void _handleNotification(uint8_t* data, uint16_t len);
    void _handleData(uint8_t* data, uint16_t len);

    friend void bleancs_loopback_notification_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
    friend void bleancs_loopback_data_cb        (BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
};


#endif /* BLEANCSLOOPBACK_H_ */
