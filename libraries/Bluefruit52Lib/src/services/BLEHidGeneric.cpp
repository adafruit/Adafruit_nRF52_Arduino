/**************************************************************************/
/*!
    @file     BLEHidGeneric.cpp
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

#include "bluefruit.h"

enum {
  REPORT_TYPE_INPUT = 1,
  REPORT_TYPE_OUTPUT,
  REPORT_TYPE_FEATURE
};

BLEHidGeneric::BLEHidGeneric(uint8_t num_input, uint8_t num_output, uint8_t num_feature)
  : BLEService(UUID16_SVC_HUMAN_INTERFACE_DEVICE), _chr_control(UUID16_CHR_HID_CONTROL_POINT)
{
  _has_keyboard = _has_mouse = false;
  _report_mode = true; // default is report mode

  _report_map = NULL;
  _report_map_len = 0;

  _input_len = _output_len = _feature_len = NULL;

  _num_input   = num_input;
  _num_output  = num_output;
  _num_feature = num_feature;

  // HID Information
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
  // bcd 1.1, country = 0, flag = normal connect
  _hid_info[0] = 0x01;
  _hid_info[1] = 0x01;
  _hid_info[2] = 0x00;
  _hid_info[3] = bit(1);

  _chr_protocol = NULL;
  _chr_inputs = _chr_outputs = _chr_features = NULL;
  _chr_boot_keyboard_input = _chr_boot_keyboard_output = _chr_boot_mouse_input = NULL;

  if ( _num_input )
  {
    _chr_inputs = new BLECharacteristic[_num_input];
  }

  if ( _num_output )
  {
    _chr_outputs = new BLECharacteristic[_num_output];
  }

  if ( _num_feature )
  {
    _chr_features = new BLECharacteristic[_num_feature];
  }
}

/*------------------------------------------------------------------*/
/* CONFIG
 *------------------------------------------------------------------*/
void BLEHidGeneric::enableKeyboard(bool enable)
{
  _has_keyboard = enable;
}

void BLEHidGeneric::enableMouse(bool enable)
{
  _has_mouse    = enable;
}

void BLEHidGeneric::setHidInfo(uint16_t bcd, uint8_t country, uint8_t flags)
{
  memcpy(_hid_info, &bcd, 2);
  _hid_info[2] = country;
  _hid_info[3] = flags;
}

void BLEHidGeneric::setReportMap(const uint8_t* report_map, size_t len)
{
  _report_map     = report_map;
  _report_map_len = len;
}

void BLEHidGeneric::setReportLen(uint16_t input_len[], uint16_t output_len[], uint16_t feature_len[])
{
  _input_len   = input_len;
  _output_len  = output_len;
  _feature_len = feature_len;
}

void BLEHidGeneric::setOutputReportCallback(uint8_t reportID, BLECharacteristic::write_cb_t fp)
{
  // index is ID-1
  uint8_t const idx =  ( reportID ? (reportID-1) : 0 );

  // report mode
  if ( idx < _num_output ) _chr_outputs[idx].setWriteCallback(fp);
}

/*------------------------------------------------------------------*/
/* Callbacks
 *------------------------------------------------------------------*/
void BLEHidGeneric::blehid_generic_protocol_mode_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;

  BLEHidGeneric& svc = (BLEHidGeneric&) chr->parentService();
  svc._report_mode = (*data); // 0 is boot, 1 Report

  LOG_LV2("HID", "Protocol Mode : %d (0 Boot, 1 Report)", *data);
}

/*------------------------------------------------------------------*/
/* Begin
 *------------------------------------------------------------------*/
err_t BLEHidGeneric::begin(void)
{
  VERIFY ( (_report_map != NULL) && _report_map_len, NRF_ERROR_INVALID_PARAM);

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Protocol Mode
  if ( _has_keyboard || _has_mouse )
  {
    _chr_protocol = new BLECharacteristic(UUID16_CHR_PROTOCOL_MODE);
    VERIFY(_chr_protocol, NRF_ERROR_NO_MEM);

    _chr_protocol->setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
    _chr_protocol->setFixedLen(1);
    _chr_protocol->setWriteCallback(BLEHidGeneric::blehid_generic_protocol_mode_cb);
    VERIFY_STATUS( _chr_protocol->begin() );
    _chr_protocol->write8(_report_mode);
  }

  // Input reports
  for(uint8_t i=0; i<_num_input; i++)
  {
    _chr_inputs[i].setUuid(UUID16_CHR_REPORT);
    _chr_inputs[i].setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_inputs[i].setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    _chr_inputs[i].setReportRefDescriptor(i+1, REPORT_TYPE_INPUT);

    // Input report len is configured, else variable len up to 255
    if ( _input_len ) _chr_inputs[i].setFixedLen( _input_len[i] );

    VERIFY_STATUS( _chr_inputs[i].begin() );
  }

  // Output reports
  for(uint8_t i=0; i<_num_output; i++)
  {
    _chr_outputs[i].setUuid(UUID16_CHR_REPORT);
    _chr_outputs[i].setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    _chr_outputs[i].setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    _chr_outputs[i].setReportRefDescriptor(i+1, REPORT_TYPE_OUTPUT);

    // Input report len is configured, else variable len up to 255
    if ( _output_len ) _chr_outputs[i].setFixedLen( _output_len[i] );

    VERIFY_STATUS( _chr_outputs[i].begin() );

    _chr_outputs[i].write8(0);
  }

  // Report Map (HID Report Descriptor)
  BLECharacteristic report_map(UUID16_CHR_REPORT_MAP);
  report_map.setTempMemory();
  report_map.setProperties(CHR_PROPS_READ);
  report_map.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  report_map.setFixedLen(_report_map_len);
  VERIFY_STATUS( report_map.begin() );
  report_map.write(_report_map, _report_map_len);

  // Boot Keyboard Input & Output Report
  if ( _has_keyboard )
  {
    _chr_boot_keyboard_input = new BLECharacteristic(UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT);
    _chr_boot_keyboard_input->setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_boot_keyboard_input->setFixedLen(8); // boot keyboard is 8 bytes
    _chr_boot_keyboard_input->setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    VERIFY_STATUS(_chr_boot_keyboard_input->begin());

    _chr_boot_keyboard_output = new BLECharacteristic(UUID16_CHR_BOOT_KEYBOARD_OUTPUT_REPORT);
    _chr_boot_keyboard_output->setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    _chr_boot_keyboard_output->setFixedLen(1); // boot keyboard is 1 byte
    _chr_boot_keyboard_output->setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    VERIFY_STATUS(_chr_boot_keyboard_output->begin());
    _chr_boot_keyboard_output->write8(0);
  }

  // Boot Mouse Input Report
  if ( _has_mouse )
  {
    _chr_boot_mouse_input = new BLECharacteristic(UUID16_CHR_BOOT_MOUSE_INPUT_REPORT);
    _chr_boot_mouse_input->setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_boot_mouse_input->setFixedLen(sizeof(hid_mouse_report_t));
    _chr_boot_mouse_input->setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    VERIFY_STATUS(_chr_boot_mouse_input->begin());
  }

  // HID Info
  BLECharacteristic hid_info(UUID16_CHR_HID_INFORMATION);
  hid_info.setTempMemory();
  hid_info.setProperties(CHR_PROPS_READ);
  hid_info.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  hid_info.setFixedLen(sizeof(_hid_info));
  VERIFY_STATUS( hid_info.begin() );
  hid_info.write(_hid_info, sizeof(_hid_info));

  // HID Control Point
  _chr_control.setProperties(CHR_PROPS_WRITE_WO_RESP);
  _chr_control.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_NO_MITM);
  _chr_control.setFixedLen(1);
  VERIFY_STATUS( _chr_control.begin() );
  _chr_control.write8(0);

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* Input Report
 *------------------------------------------------------------------*/
bool BLEHidGeneric::inputReport(uint16_t conn_hdl, uint8_t reportID, void const* data, int len)
{
  // index is ID-1
  uint8_t const idx =  ( reportID ? (reportID-1) : 0 );
  return _chr_inputs[idx].notify(conn_hdl, (uint8_t const*) data, len);
}

bool BLEHidGeneric::bootKeyboardReport(uint16_t conn_hdl, void const* data, int len)
{
  return _chr_boot_keyboard_input->notify(conn_hdl, data, len);
}

bool BLEHidGeneric::bootMouseReport(uint16_t conn_hdl, void const* data, int len)
{
  return _chr_boot_mouse_input->notify(conn_hdl, data, len);
}

bool BLEHidGeneric::inputReport(uint8_t reportID, void const* data, int len)
{
  return inputReport(BLE_CONN_HANDLE_INVALID, reportID, data, len);
}

bool BLEHidGeneric::bootKeyboardReport(void const* data, int len)
{
  return bootKeyboardReport(BLE_CONN_HANDLE_INVALID, data, len);
}

bool BLEHidGeneric::bootMouseReport(void const* data, int len)
{
  return bootMouseReport(BLE_CONN_HANDLE_INVALID, data, len);
}


// Conversion table from Ascii to keycode (shift, keycode)
const uint8_t hid_ascii_to_keycode[128][2] = { HID_ASCII_TO_KEYCODE };


// Conversion table from Keycode to Ascii (ascii without shift, ascii with shift)
const uint8_t hid_keycode_to_ascii[128][2] = { HID_KEYCODE_TO_ASCII };

