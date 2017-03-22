/**************************************************************************/
/*!
    @file     BLECharacteristic.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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

class AdafruitBluefruit;
class BLEService;

enum BleSecurityMode
{
  SECMODE_NO_ACCESS        = 0x00,
  SECMODE_OPEN             = 0x11,
  SECMODE_ENC_NO_MITM      = 0x21,
  SECMODE_ENC_WITH_MITM    = 0x31,
  SECMODE_SIGNED_NO_MITM   = 0x12,
  SECMODE_SIGNED_WITH_MITM = 0x22
};

#define BLE_SECMODE_NO_ACCESS        ((ble_gap_conn_sec_mode_t) { .sm = 0, .lv = 0 })
#define BLE_SECMODE_OPEN             ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 1 })
#define BLE_SECMODE_ENC_NO_MITM      ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 2 })
#define BLE_SECMODE_ENC_WITH_MITM    ((ble_gap_conn_sec_mode_t) { .sm = 1, .lv = 3 })
#define BLE_SECMODE_SIGNED_NO_MITM   ((ble_gap_conn_sec_mode_t) { .sm = 2, .lv = 1 })
#define BLE_SECMODE_SIGNED_WITH_MITM ((ble_gap_conn_sec_mode_t) { .sm = 2, .lv = 2 })

enum CharsProperties
{
  CHR_PROPS_BROADCAST       = bit(0),
  CHR_PROPS_READ            = bit(1),
  CHR_PROPS_WRITE_WO_RESP   = bit(2),
  CHR_PROPS_WRITE           = bit(3),
  CHR_PROPS_NOTIFY          = bit(4),
  CHR_PROPS_INDICATE        = bit(5)
};

class BLECharacteristic
{
  public:
    typedef void (*read_authorize_cb_t)  (BLECharacteristic& chr, ble_gatts_evt_read_t * request);
    typedef void (*write_authorize_cb_t) (BLECharacteristic& chr, ble_gatts_evt_write_t* request);
    typedef void (*write_cb_t)           (BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);
    typedef void (*write_cccd_cb_t)      (BLECharacteristic& chr, uint16_t value);

  protected:
    bool _is_temp;

    ble_gatt_char_props_t _properties;
    const char* _descriptor;
    uint16_t _max_len;

    struct ATTR_PACKED {
      uint8_t id;
      uint8_t type;
    }_report_ref_desc;

    /* Characteristic attribute metadata */
    /* https://devzone.nordicsemi.com/documentation/nrf51/5.2.0/html/a00269.html */
    ble_gatts_attr_md_t _attr_meta;

    ble_gatts_char_handles_t _handles;

    BLEService* _service; // pointer to parent's service

    // Callback pointer
    read_authorize_cb_t  _rd_authorize_cb;
    write_authorize_cb_t _wr_authorize_cb;

    write_cb_t           _wr_cb;
    write_cccd_cb_t      _cccd_wr_cb;

    void init(void);
    void eventHandler(ble_evt_t* event);
  
  public:
    BLEUuid uuid;

    typedef void (*chars_cb_t) (void);

    BLECharacteristic(void);
    BLECharacteristic(BLEUuid bleuuid);

    BLEService& parentService() { return *_service; }

    void setTempMemory(void);

    /*------------- Configure -------------*/
    void setUuid(BLEUuid bleuuid);
    void setProperties(uint8_t prop);
    void setPermission(BleSecurityMode read_perm, BleSecurityMode write_perm);
    void setMaxLen(uint16_t max_len);
    void setFixedLen(uint16_t fixed_len);
    void setStringDescriptor(const char* descriptor); // aka user descriptor
    void setReportRefDescriptor(uint8_t id, uint8_t type);

    /*------------- Callback -------------*/
    void setWriteCallback(write_cb_t fp);
    void setCccdWriteCallback(write_cccd_cb_t fp);
    void setReadAuthorizeCallback(read_authorize_cb_t fp);
    void setWriteAuthorizeCallbak(write_authorize_cb_t fp);

    err_t begin(void);

    ble_gatts_char_handles_t handles(void);

    /*------------- Write -------------*/
    err_t write(const void* data, int len, uint16_t offset = 0);
    err_t write(const char   * str);

    err_t write(int      num);
    err_t write(uint32_t num);
    err_t write(uint16_t num);
    err_t write(uint8_t  num);

    /*------------- Read -------------*/
    uint16_t read(void* buffer, int bufsize, uint16_t offset = 0);
    uint16_t read(uint32_t* num);
    uint16_t read(uint16_t* num);
    uint16_t read(uint8_t*  num);

    /*------------- Notify -------------*/
    bool  notifyEnabled(void);
    err_t notify(const void* data, int len, uint16_t offset);
    err_t notify(const void* data, int len);
    err_t notify(const char   * str);
    err_t notify(int      num);
    err_t notify(uint32_t num);
    err_t notify(uint16_t num);
    err_t notify(uint8_t  num);


    friend class AdafruitBluefruit;
};



#endif /* BLECHARACTERISTIC_H_ */
