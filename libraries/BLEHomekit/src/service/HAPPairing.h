/**************************************************************************/
/*!
    @file     HAPPairing.h
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
#ifndef HAPPAIRING_H_
#define HAPPAIRING_H_

#include "HAPCharacteristic.h"
#include "HAPService.h"

class HAPPairing : public HAPService
{
  public:
    HAPPairing(void);
    virtual err_t begin(void);

  private:
    HAPCharacteristic _setup;
    HAPCharacteristic _verify;
    HAPCharacteristic _features;
    HAPCharacteristic _pairing;

    uint8_t _pair_id[17]; // Device ID in string format e.g "aa:bb:cc:dd:ee"

    void setDeviceID(uint8_t dev_id[6]);

    void createSrpResponse(uint16_t conn_hdl, uint8_t status, TLV8_t ktlv[], uint8_t count);

    void pair_setup_m1(uint16_t conn_hdl, HAPRequest_t const* hap_req);
    void pair_setup_m3(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t pubkey, TLV8_t proof);
    void pair_setup_m5(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t encrypted);


    void pair_verify_m1(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t pubkey);
    void pair_verify_m3(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t encrypted);

    friend void _pair_setup_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req);
    friend void _pair_verify_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req);

#if CFG_DEBUG
    friend void test_homekit(void);
#endif
};



#endif /* HAPPAIRING_H_ */
