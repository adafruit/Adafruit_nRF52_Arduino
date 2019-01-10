/**************************************************************************/
/*!
    @file     HAPCharacteristic.h
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
#ifndef HAPCHARACTERISTIC_H_
#define HAPCHARACTERISTIC_H_

#include <BLECharacteristic.h>
#include "HAPProcedure.h"

enum HAPChrProperties_t
{
  HAP_CHR_PROPS_READ                  = bit(0),
  HAP_CHR_PROPS_WRITE                 = bit(1),
  HAP_CHR_PROPS_ADDITIONAL_AUTH_DATA  = bit(2),
  HAP_CHR_PROPS_TIMED_WRITE_PROCEDURE = bit(3),
  HAP_CHR_PROPS_SECURE_READ           = bit(4),
  HAP_CHR_PROPS_SECURE_WRITE          = bit(5),
  HAP_CHR_PROPS_HIDDEN                = bit(6), // from user
  HAP_CHR_PROPS_NOTIFY                = bit(7),
  HAP_CHR_PROPS_NOTIFY_DISCONNECTED   = bit(8),
};

class HAPCharacteristic : public BLECharacteristic
{
  public:
    typedef void (*hap_write_cb_t) (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req);
    typedef void (*hap_read_cb_t ) (uint16_t conn_hdl, HAPCharacteristic* chr);
    static BLEUuid _g_uuid_cid;

    HAPCharacteristic(BLEUuid bleuuid, uint8_t format, uint16_t unit = UUID16_UNIT_UNITLESS);
    virtual err_t begin(void);

    void setHapProperties(uint16_t prop);

    // Write to Hap Value
    uint16_t writeHapValue(const void* data, uint16_t len);
    uint16_t writeHapValue(const char* str);
    uint16_t writeHapValue(uint32_t num);

    // Callbacks
    void setHapWriteCallback(hap_write_cb_t fp);
    void setHapReadCallback(hap_read_cb_t fp);

    /*------------- Internal Functions -------------*/
    virtual void _eventHandler(ble_evt_t* event);

    void createHapResponse(uint16_t conn_hdl, uint8_t status, TLV8_t tlv_para[] = NULL, uint8_t count = 0);

  private:
    uint16_t _cid;
    uint16_t _hap_props;

    // HAP request & response
    HAPRequest_t* _hap_req;
    uint16_t      _hap_reqlen;
    uint8_t       _tid;

    HAPResponse_t* _hap_resp;
    uint16_t       _hap_resplen;
    uint16_t       _hap_resplen_sent;

    // Char value is read by HAP procedure, not exposed via GATT
    void*    _value;
    uint16_t _vallen;

    // Callbacks
    hap_write_cb_t _hap_wr_cb;
    hap_read_cb_t  _hap_rd_cb;

    err_t _addChrIdDescriptor(void);

    void processGattWrite(uint16_t conn_hdl, ble_gatts_evt_write_t* gatt_req);
    void processGattRead(uint16_t conn_hdl, ble_gatts_evt_read_t* gatt_req);

    void processHapRequest(uint16_t conn_hdl, HAPRequest_t* hap_req);
    void processChrSignatureRead(uint16_t conn_hdl, HAPRequest_t* hap_req);
    void processChrRead(uint16_t conn_hdl, HAPRequest_t* hap_req);

    friend void test_homekit(void);
};

#endif /* HAPCHARACTERISTIC_H_ */
