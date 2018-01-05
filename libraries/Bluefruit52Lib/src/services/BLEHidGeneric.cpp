/**************************************************************************/
/*!
    @file     BLEHidGeneric.cpp
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

#include "bluefruit.h"

enum {
  REPORT_TYPE_INPUT = 1,
  REPORT_TYPE_OUTPUT,
  REPORT_TYPE_FEATURE
};

BLEHidGeneric::BLEHidGeneric(uint8_t num_input, uint8_t num_output, uint8_t num_feature)
  : BLEService(UUID16_SVC_HUMAN_INTERFACE_DEVICE), _chr_control(UUID16_CHR_HID_CONTROL_POINT)
{
  _boot_keyboard = _boot_mouse = false;
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

  _output_cbs = NULL;

  if ( _num_input   )
  {
    _chr_inputs   = new BLECharacteristic[_num_input];
  }

  if ( _num_output  )
  {
    _chr_outputs  = new BLECharacteristic[_num_output];
    _output_cbs   = new output_report_cb_t[_num_output];

    for (uint8_t i=0; i<_num_output; i++) _output_cbs[i] = NULL;
  }

  if ( _num_feature )
  {
    _chr_features = new BLECharacteristic[_num_feature];
  }
}

void BLEHidGeneric::enableBootProtocol(bool bootKeyboard, bool bootMouse)
{
  _boot_keyboard = bootKeyboard;
  _boot_mouse    = bootMouse;
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

void BLEHidGeneric::setOutputReportCallback(uint8_t reportID, output_report_cb_t fp)
{
  _output_cbs[reportID] = fp;
}

COMMENT_OUT (
void blehidgeneric_output_cb(BLECharacteristic& chr, ble_gatts_evt_write_t* request)
{
  (void) chr;
  (void) request;
//  BLEHidGeneric& hid = (BLEHidGeneric&) chr.parentService();
//  PRINT_BUFFER(request->data, request->len);
}
)

err_t BLEHidGeneric::begin(void)
{
  VERIFY ( (_report_map != NULL) && _report_map_len, NRF_ERROR_INVALID_PARAM);

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Protocol Mode
  if ( _boot_keyboard || _boot_mouse )
  {
    _chr_protocol = new BLECharacteristic(UUID16_CHR_PROTOCOL_MODE);
    VERIFY(_chr_protocol, NRF_ERROR_NO_MEM);

    _chr_protocol->setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
    _chr_protocol->setFixedLen(1);
    VERIFY_STATUS( _chr_protocol->begin() );
    _chr_protocol->write8(1);
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
    _chr_outputs[i].setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    _chr_outputs[i].setReportRefDescriptor(i+1, REPORT_TYPE_OUTPUT);

    COMMENT_OUT(
    _chr_outputs[i].setWriteCallback(blehidgeneric_output_cb);
    )

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
  if ( _boot_keyboard )
  {
    _chr_boot_keyboard_input = new BLECharacteristic(UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT);
    _chr_boot_keyboard_input->setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_boot_keyboard_input->setFixedLen(8); // boot keyboard is 8 bytes
    _chr_boot_keyboard_input->setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    VERIFY_STATUS(_chr_boot_keyboard_input->begin());

    _chr_boot_keyboard_output = new BLECharacteristic(UUID16_CHR_BOOT_KEYBOARD_OUTPUT_REPORT);
    _chr_boot_keyboard_output->setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    _chr_boot_keyboard_output->setFixedLen(1); // boot keyboard is 1 byte
    _chr_boot_keyboard_output->setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    VERIFY_STATUS(_chr_boot_keyboard_output->begin());
    _chr_boot_keyboard_output->write8(0);
  }

  // Boot Mouse Input Report
  if ( _boot_mouse )
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
  _chr_control.write( (uint8_t) 0 );

  return ERROR_NONE;
}

bool BLEHidGeneric::inputReport(uint8_t reportID, void const* data, int len)
{
  // 0 will treated as report ID = 1
  if ( reportID == 0 ) reportID++;

  return _chr_inputs[reportID-1].notify( (uint8_t const*) data, len);
}

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/
const hid_ascii_to_keycode_entry_t HID_ASCII_TO_KEYCODE[128] =
{
    {0, 0                     }, // 0x00 Null
    {0, 0                     }, // 0x01
    {0, 0                     }, // 0x02
    {0, 0                     }, // 0x03
    {0, 0                     }, // 0x04
    {0, 0                     }, // 0x05
    {0, 0                     }, // 0x06
    {0, 0                     }, // 0x07
    {0, HID_KEY_BACKSPACE     }, // 0x08 Backspace
    {0, HID_KEY_TAB           }, // 0x09 Horizontal Tab
    {0, HID_KEY_RETURN        }, // 0x0A Line Feed
    {0, 0                     }, // 0x0B
    {0, 0                     }, // 0x0C
    {0, HID_KEY_RETURN        }, // 0x0D Carriage return
    {0, 0                     }, // 0x0E
    {0, 0                     }, // 0x0F
    {0, 0                     }, // 0x10
    {0, 0                     }, // 0x11
    {0, 0                     }, // 0x12
    {0, 0                     }, // 0x13
    {0, 0                     }, // 0x14
    {0, 0                     }, // 0x15
    {0, 0                     }, // 0x16
    {0, 0                     }, // 0x17
    {0, 0                     }, // 0x18
    {0, 0                     }, // 0x19
    {0, 0                     }, // 0x1A
    {0, HID_KEY_ESCAPE        }, // 0x1B Escape
    {0, 0                     }, // 0x1C
    {0, 0                     }, // 0x1D
    {0, 0                     }, // 0x1E
    {0, 0                     }, // 0x1F

    {0, HID_KEY_SPACE         }, // 0x20
    {1, HID_KEY_1             }, // 0x21 !
    {1, HID_KEY_APOSTROPHE    }, // 0x22 "
    {1, HID_KEY_3             }, // 0x23 #
    {1, HID_KEY_4             }, // 0x24 $
    {1, HID_KEY_5             }, // 0x25 %
    {1, HID_KEY_7             }, // 0x26 &
    {0, HID_KEY_APOSTROPHE    }, // 0x27 '
    {1, HID_KEY_9             }, // 0x28 (
    {1, HID_KEY_0             }, // 0x29 )
    {1, HID_KEY_8             }, // 0x2A *
    {1, HID_KEY_EQUAL         }, // 0x2B +
    {0, HID_KEY_COMMA         }, // 0x2C ,
    {0, HID_KEY_MINUS         }, // 0x2D -
    {0, HID_KEY_PERIOD        }, // 0x2E .
    {0, HID_KEY_SLASH         }, // 0x2F /
    {0, HID_KEY_0             }, // 0x30 0
    {0, HID_KEY_1             }, // 0x31 1
    {0, HID_KEY_2             }, // 0x32 2
    {0, HID_KEY_3             }, // 0x33 3
    {0, HID_KEY_4             }, // 0x34 4
    {0, HID_KEY_5             }, // 0x35 5
    {0, HID_KEY_6             }, // 0x36 6
    {0, HID_KEY_7             }, // 0x37 7
    {0, HID_KEY_8             }, // 0x38 8
    {0, HID_KEY_9             }, // 0x39 9
    {1, HID_KEY_SEMICOLON     }, // 0x3A :
    {0, HID_KEY_SEMICOLON     }, // 0x3B ;
    {1, HID_KEY_COMMA         }, // 0x3C <
    {0, HID_KEY_EQUAL         }, // 0x3D =
    {1, HID_KEY_PERIOD        }, // 0x3E >
    {1, HID_KEY_SLASH         }, // 0x3F ?

    {1, HID_KEY_2             }, // 0x40 @
    {1, HID_KEY_A             }, // 0x41 A
    {1, HID_KEY_B             }, // 0x42 B
    {1, HID_KEY_C             }, // 0x43 C
    {1, HID_KEY_D             }, // 0x44 D
    {1, HID_KEY_E             }, // 0x45 E
    {1, HID_KEY_F             }, // 0x46 F
    {1, HID_KEY_G             }, // 0x47 G
    {1, HID_KEY_H             }, // 0x48 H
    {1, HID_KEY_I             }, // 0x49 I
    {1, HID_KEY_J             }, // 0x4A J
    {1, HID_KEY_K             }, // 0x4B K
    {1, HID_KEY_L             }, // 0x4C L
    {1, HID_KEY_M             }, // 0x4D M
    {1, HID_KEY_N             }, // 0x4E N
    {1, HID_KEY_O             }, // 0x4F O
    {1, HID_KEY_P             }, // 0x50 P
    {1, HID_KEY_Q             }, // 0x51 Q
    {1, HID_KEY_R             }, // 0x52 R
    {1, HID_KEY_S             }, // 0x53 S
    {1, HID_KEY_T             }, // 0x55 T
    {1, HID_KEY_U             }, // 0x55 U
    {1, HID_KEY_V             }, // 0x56 V
    {1, HID_KEY_W             }, // 0x57 W
    {1, HID_KEY_X             }, // 0x58 X
    {1, HID_KEY_Y             }, // 0x59 Y
    {1, HID_KEY_Z             }, // 0x5A Z
    {0, HID_KEY_BRACKET_LEFT  }, // 0x5B [
    {0, HID_KEY_BACKSLASH     }, // 0x5C '\'
    {0, HID_KEY_BRACKET_RIGHT }, // 0x5D ]
    {1, HID_KEY_6             }, // 0x5E ^
    {1, HID_KEY_MINUS         }, // 0x5F _

    {0, HID_KEY_GRAVE         }, // 0x60 `
    {0, HID_KEY_A             }, // 0x61 a
    {0, HID_KEY_B             }, // 0x62 b
    {0, HID_KEY_C             }, // 0x63 c
    {0, HID_KEY_D             }, // 0x66 d
    {0, HID_KEY_E             }, // 0x65 e
    {0, HID_KEY_F             }, // 0x66 f
    {0, HID_KEY_G             }, // 0x67 g
    {0, HID_KEY_H             }, // 0x68 h
    {0, HID_KEY_I             }, // 0x69 i
    {0, HID_KEY_J             }, // 0x6A j
    {0, HID_KEY_K             }, // 0x6B k
    {0, HID_KEY_L             }, // 0x6C l
    {0, HID_KEY_M             }, // 0x6D m
    {0, HID_KEY_N             }, // 0x6E n
    {0, HID_KEY_O             }, // 0x6F o
    {0, HID_KEY_P             }, // 0x70 p
    {0, HID_KEY_Q             }, // 0x71 q
    {0, HID_KEY_R             }, // 0x72 r
    {0, HID_KEY_S             }, // 0x73 s
    {0, HID_KEY_T             }, // 0x75 t
    {0, HID_KEY_U             }, // 0x75 u
    {0, HID_KEY_V             }, // 0x76 v
    {0, HID_KEY_W             }, // 0x77 w
    {0, HID_KEY_X             }, // 0x78 x
    {0, HID_KEY_Y             }, // 0x79 y
    {0, HID_KEY_Z             }, // 0x7A z
    {1, HID_KEY_BRACKET_LEFT  }, // 0x7B {
    {1, HID_KEY_BACKSLASH     }, // 0x7C |
    {1, HID_KEY_BRACKET_RIGHT }, // 0x7D }
    {1, HID_KEY_GRAVE         }, // 0x7E ~
    {0, HID_KEY_DELETE        }  // 0x7F Delete
};

