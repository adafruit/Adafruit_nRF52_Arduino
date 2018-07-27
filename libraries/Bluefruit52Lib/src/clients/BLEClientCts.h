/**************************************************************************/
/*!
    @file     BLEClientCts.h
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
#ifndef BLECLIENTCTS_H_
#define BLECLIENTCTS_H_

#include "bluefruit_common.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"

class BLEClientCts : public BLEClientService
{
  public:
    // Callback Signatures
    typedef void (*adjust_callback_t) (uint8_t reason);

    BLEClientCts(void);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    bool getCurrentTime(void);
    bool getLocalTimeInfo(void);

    bool enableAdjust(void);
    void setAdjustCallback(adjust_callback_t fp);

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.current_time.xml
    struct ATTR_PACKED {
      uint16_t year;
      uint8_t  month;
      uint8_t  day;
      uint8_t  hour;
      uint8_t  minute;
      uint8_t  second;
      uint8_t  weekday;
      uint8_t  subsecond;
      uint8_t  adjust_reason;
    } Time;

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.local_time_information.xml
    struct ATTR_PACKED {
      int8_t  timezone;
      uint8_t dst_offset;
    }LocalInfo;

    /*------------------------------------------------------------------*/
    /* INTERNAL USAGE ONLY
     * Although declare as public, it is meant to be invoked by internal
     * code. User should not call these directly
     *------------------------------------------------------------------*/
    void _cur_time_notify_cb(uint8_t* data, uint16_t len);

  private:
    BLEClientCharacteristic _cur_time;
    BLEClientCharacteristic _local_info;

    adjust_callback_t _adjust_cb;
};

#endif /* BLECLIENTCTS_H_ */
