/**************************************************************************/
/*!
    @file     BLEClientDis.h
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

#ifndef BLECLIENTDIS_H_
#define BLECLIENTDIS_H_

#include "bluefruit_common.h"
#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"

class BLEClientDis : public BLEClientService
{
  public:
    BLEClientDis(void);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    uint16_t getChars(uint16_t uuid, char* buffer, uint16_t bufsize);

    uint16_t getModel       (char* buffer, uint16_t bufsize);
    uint16_t getSerial      (char* buffer, uint16_t bufsize);
    uint16_t getFirmwareRev (char* buffer, uint16_t bufsize);
    uint16_t getHardwareRev (char* buffer, uint16_t bufsize);
    uint16_t getSoftwareRev (char* buffer, uint16_t bufsize);
    uint16_t getManufacturer(char* buffer, uint16_t bufsize);

  private:

    // BLE DIS has several characteristics but is often used one or two times
    // It is better to implement get() with on-the-fly BLEClientCharacteristic
};



#endif /* BLECLIENTDIS_H_ */
