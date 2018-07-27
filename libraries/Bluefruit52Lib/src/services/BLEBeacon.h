/**************************************************************************/
/*!
    @file     BLEBeacon.h
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
#ifndef BLEBEACON_H_
#define BLEBEACON_H_

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

class BLEAdvertising; // forward declare

class BLEBeacon
{
  private:
    uint16_t _manufacturer_id;
    uint8_t const* _uuid128;
    uint16_t _major_be; // Big Endian
    uint16_t _minor_be; // Big Endian
    int8_t   _rssi_at_1m;

    void _init(void);

  public:
    BLEBeacon(void);
    BLEBeacon(uint8_t const uuid128[16]);
    BLEBeacon(uint8_t const uuid128[16], uint16_t major, uint16_t minor, int8_t rssi);

    void setManufacturer(uint16_t manfacturer);
    void setUuid(uint8_t const uuid128[16]);
    void setMajorMinor(uint16_t major, uint16_t minor);
    void setRssiAt1m(int8_t rssi);

    bool start(void);
    bool start(BLEAdvertising& adv);
};



#endif /* BLEBEACON_H_ */
