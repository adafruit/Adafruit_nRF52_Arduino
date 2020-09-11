/**************************************************************************/
/*!
    @file     BLEService.h
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
#ifndef BLESERVICE_H_
#define BLESERVICE_H_

#include "bluefruit_common.h"
#include "BLEUuid.h"

class BLEService
{
  protected:
    uint16_t _handle; // service gatt handle
    SecureMode_t _read_perm;
    SecureMode_t _write_perm;

    void  _init(void);

    virtual void svc_disconnect_hdl(uint16_t conn_hdl);
    virtual void svc_connect_hdl(uint16_t conn_hdl);
  
  public:
    static BLEService* lastService;

    BLEUuid uuid;

    BLEService(void);
    BLEService(BLEUuid bleuuid);

    void setUuid(BLEUuid bleuuid);

    void setPermission(SecureMode_t read_perm, SecureMode_t write_perm);
    void getPermission(SecureMode_t* read_perm, SecureMode_t* write_perm);

    virtual err_t begin(void);

    friend class BLEGatt;
};

#endif /* BLESERVICE_H_ */
