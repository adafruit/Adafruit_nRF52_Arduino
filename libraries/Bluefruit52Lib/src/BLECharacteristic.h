/**************************************************************************/
/*!
    @file     BLECharacteristic.h
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
#ifndef BLECHARACTERISTIC_H_
#define BLECHARACTERISTIC_H_

#include "bluefruit_common.h"
#include "BLEUuid.h"

// Forward declaration
class AdafruitBluefruit;
class BLEService;

enum CharsProperties
{
  CHR_PROPS_BROADCAST       = bit(0),
  CHR_PROPS_READ            = bit(1),
  CHR_PROPS_WRITE_WO_RESP   = bit(2),
  CHR_PROPS_WRITE           = bit(3),
  CHR_PROPS_NOTIFY          = bit(4),
  CHR_PROPS_INDICATE        = bit(5)
};

// same as CharsProperties, but make it easier to migrate from ArduinoBLE
enum BLECharsProperties
{
  BLEBroadcast            = 0x01,
  BLERead                 = 0x02,
  BLEWriteWithoutResponse = 0x04,
  BLEWrite                = 0x08,
  BLENotify               = 0x10,
  BLEIndicate             = 0x20
};

class BLECharacteristic
{
  public:
    /*--------- Callback Signatures ----------*/
    typedef void (*read_authorize_cb_t)  (uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t * request);
    typedef void (*write_authorize_cb_t) (uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_write_t* request);
    typedef void (*write_cb_t)           (uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
    typedef void (*write_cccd_cb_t)      (uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value);

    BLEUuid uuid;

    // Constructors
    BLECharacteristic(void);
    BLECharacteristic(BLEUuid bleuuid);
    BLECharacteristic(BLEUuid bleuuid, uint8_t properties);
    BLECharacteristic(BLEUuid bleuuid, uint8_t properties, int max_len, bool fixed_len = false);

    // Destructor
    virtual ~BLECharacteristic();

    BLEService& parentService(void);

    void setTempMemory(void);

    /*------------- Configure -------------*/
    void setUuid(BLEUuid bleuuid);
    void setProperties(uint8_t prop);
    void setPermission(SecureMode_t read_perm, SecureMode_t write_perm);
    void setMaxLen(uint16_t max_len);
    void setFixedLen(uint16_t fixed_len);
    void setBuffer(void* buf, uint16_t bufsize);

    uint16_t getMaxLen(void);
    bool isFixedLen(void);

    /*------------- Descriptors -------------*/
    void setUserDescriptor(const char* descriptor); // aka user descriptor
    void setReportRefDescriptor(uint8_t id, uint8_t type); // TODO refactor to use addDescriptor()
    void setPresentationFormatDescriptor(uint8_t type, int8_t exponent, uint16_t unit, uint8_t name_space = 1, uint16_t descritpor = 0);

    /*------------- Callbacks -------------*/
    void setWriteCallback        (write_cb_t            fp, bool useAdaCallback = true);
    void setCccdWriteCallback    (write_cccd_cb_t       fp, bool useAdaCallback = true);
    void setReadAuthorizeCallback(read_authorize_cb_t   fp, bool useAdaCallback = true);
    void setWriteAuthorizeCallback(write_authorize_cb_t fp, bool useAdaCallback = true);

    virtual err_t begin(void);

    // Add Descriptor function must be called right after begin()
    err_t addDescriptor(BLEUuid bleuuid, void const * content, uint16_t len, SecureMode_t read_perm = SECMODE_OPEN, SecureMode_t write_perm = SECMODE_NO_ACCESS);

    ble_gatts_char_handles_t handles(void);

    /*------------- Write -------------*/
    uint16_t write   (const void* data, uint16_t len);
    uint16_t write   (const char* str);

    uint16_t write8     (uint8_t  num);
    uint16_t write16    (uint16_t num);
    uint16_t write32    (uint32_t num);
    uint16_t write32    (int      num);
    uint16_t writeFloat (float    num);

    /*------------- Read -------------*/
    uint16_t read  (void* buffer, uint16_t bufsize, uint16_t offset = 0);

    uint8_t  read8     (void);
    uint16_t read16    (void);
    uint32_t read32    (void);
    float    readFloat (void);

    uint16_t getCccd(uint16_t conn_hdl);

    bool notifyEnabled(void);
    bool notifyEnabled(uint16_t conn_hdl);

    /*------------- Notify -------------*/
//    bool notify   (void);
    bool notify   (const void* data, uint16_t len);
    bool notify   (const char* str);

    bool notify8  (uint8_t  num);
    bool notify16 (uint16_t num);
    bool notify32 (uint32_t num);
    bool notify32 (int      num);
    bool notify32 (float    fnum);

    /*------------- Notify multiple connections -------------*/
//    bool notify   (uint16_t conn_hdl);
    bool notify   (uint16_t conn_hdl, const void* data, uint16_t len);
    bool notify   (uint16_t conn_hdl, const char* str);

    bool notify8  (uint16_t conn_hdl, uint8_t  num);
    bool notify16 (uint16_t conn_hdl, uint16_t num);
    bool notify32 (uint16_t conn_hdl, uint32_t num);
    bool notify32 (uint16_t conn_hdl, int      num);
    bool notify32 (uint16_t conn_hdl, float    num);

    /*------------- Indicate -------------*/
    bool indicateEnabled(void);
    bool indicateEnabled(uint16_t conn_hdl);

    bool indicate   (const void* data, uint16_t len);
    bool indicate   (const char* str);

    bool indicate8  (uint8_t  num);
    bool indicate16 (uint16_t num);
    bool indicate32 (uint32_t num);
    bool indicate32 (int      num);
    bool indicate32 (float    num);

    /*------------- Indicate multiple connections -------------*/
    bool indicate   (uint16_t conn_hdl, const void* data, uint16_t len);
    bool indicate   (uint16_t conn_hdl, const char* str);

    bool indicate8  (uint16_t conn_hdl, uint8_t  num);
    bool indicate16 (uint16_t conn_hdl, uint16_t num);
    bool indicate32 (uint16_t conn_hdl, uint32_t num);
    bool indicate32 (uint16_t conn_hdl, int      num);
    bool indicate32 (uint16_t conn_hdl, float    num);

    /*------------- Internal Functions -------------*/
    virtual void _eventHandler(ble_evt_t* event);

  protected:
    bool _is_temp;
    uint16_t _max_len;
    BLEService* _service; // pointer to parent's service

    uint8_t* _userbuf;

    /*------------- Descriptors -------------*/
    const char* _usr_descriptor;
    struct ATTR_PACKED {
      uint8_t id;
      uint8_t type;
    }_report_ref_desc;

    ble_gatts_char_pf_t       _format_desc;
    ble_gatt_char_props_t     _properties;
    ble_gatts_attr_md_t       _attr_meta;
    ble_gatts_char_handles_t  _handles;

    struct {
        uint8_t* buffer;
        uint16_t bufsize;
        uint16_t count;
    }_long_wr;

    /*------------- Callback pointers -------------*/
    struct ATTR_PACKED {
      uint8_t write           : 1;
      uint8_t cccd_write      : 1;
      uint8_t read_authorize  : 1;
      uint8_t write_authorize : 1;
    } _use_ada_cb;

    read_authorize_cb_t   _rd_authorize_cb;
    write_authorize_cb_t  _wr_authorize_cb;

    write_cb_t            _wr_cb;
    write_cccd_cb_t       _cccd_wr_cb;

    /*------------- Internal Functions -------------*/
    void _init(void);
};

#endif /* BLECHARACTERISTIC_H_ */
