/**************************************************************************/
/*!
    @file     BLEAdvertising.h
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
#ifndef BLEADVERTISING_H_
#define BLEADVERTISING_H_

#include <Arduino.h>
#include "bluefruit_common.h"

#include "BLEUuid.h"
#include "BLEService.h"
#include "BLECentralService.h"

#include "services/BLEBeacon.h"

//enum
//{
//  BLE_ADV_MODE
//};

class BLEAdvertising
{
private:
  uint8_t _data[BLE_GAP_ADV_MAX_SIZE];
  uint8_t _count;

public:
  BLEAdvertising(void);

  bool start(uint8_t mode = 0);
  bool stop (void);

  bool addData(uint8_t type, const void* data, uint8_t len);
  bool addFlags(uint8_t flags);
  bool addTxPower(void);
  bool addName(void);
  bool addAppearance(uint16_t appearance);

  bool addUuid(BLEUuid bleuuid);
  bool addService(BLEService& service);
  bool addService(BLECentralService& service);

  bool setBeacon(BLEBeacon& beacon);

  // Custom API
  uint8_t count(void);
  char*   getData(void);
  bool    setData(const uint8_t* data, uint8_t count);
  void    clearData(void);

};

#endif /* BLEADVERTISING_H_ */
