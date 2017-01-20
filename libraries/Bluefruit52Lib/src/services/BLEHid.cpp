/**************************************************************************/
/*!
    @file     BLEHid.cpp
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

enum
{
  REPORT_INDEX_KEYBOARD =0,
  REPORT_INDEX_CONSUMER_CONTROL,
  REPORT_INDEX_MOUSE,
  REPORT_INDEX_GAMEPAD,
};

enum
{
  REPORT_ID_KEYBOARD = 1,
  REPORT_ID_CONSUMER_CONTROL,
  REPORT_ID_MOUSE,
  REPORT_ID_GAMEPAD
};


uint8_t const hid_report_descriptor[] =
{
  //------------- Keyboard Report  -------------//
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     ),
  HID_USAGE      ( HID_USAGE_DESKTOP_KEYBOARD ),
  HID_COLLECTION ( HID_COLLECTION_APPLICATION ),
    HID_REPORT_ID ( REPORT_ID_KEYBOARD      ),
    HID_USAGE_PAGE( HID_USAGE_PAGE_KEYBOARD ),
      // 8 bits Modifier Keys (Shfit, Control, Alt)
      HID_USAGE_MIN    ( 224                                    ),
      HID_USAGE_MAX    ( 231                                    ),
      HID_LOGICAL_MIN  ( 0                                      ),
      HID_LOGICAL_MAX  ( 1                                      ),

      HID_REPORT_COUNT ( 8                                      ),
      HID_REPORT_SIZE  ( 1                                      ),
      HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),

      // 8 bit reserved
      HID_REPORT_COUNT ( 1                                      ),
      HID_REPORT_SIZE  ( 8                                      ),
      HID_INPUT        ( HID_CONSTANT                           ),

    // LED Indicator Kana | Compose | Scroll Lock | CapsLock | NumLock
    HID_USAGE_PAGE  ( HID_USAGE_PAGE_LED                   ),
      /* 5-bit Led report */
      HID_USAGE_MIN    ( 1                                       ),
      HID_USAGE_MAX    ( 5                                       ),
      HID_REPORT_COUNT ( 5                                       ),
      HID_REPORT_SIZE  ( 1                                       ),
      HID_OUTPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE  ),
      /* led padding */
      HID_REPORT_COUNT ( 1                                       ),
      HID_REPORT_SIZE  ( 3                                       ),
      HID_OUTPUT       ( HID_CONSTANT                            ),

    // 6-byte Keycodes
    HID_USAGE_PAGE (HID_USAGE_PAGE_KEYBOARD),
      HID_USAGE_MIN    ( 0                                   ),
      HID_USAGE_MAX    ( 101                                 ),
      HID_LOGICAL_MIN  ( 0                                   ),
      HID_LOGICAL_MAX  ( 101                                 ),

      HID_REPORT_COUNT ( 6                                   ),
      HID_REPORT_SIZE  ( 8                                   ),
      HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE ),
  HID_COLLECTION_END,

#if 0
  //------------- Consumer Control Report -------------//
  HID_USAGE_PAGE ( HID_USAGE_PAGE_CONSUMER    ),
  HID_USAGE      ( HID_USAGE_CONSUMER_CONTROL ),
  HID_COLLECTION ( HID_COLLECTION_APPLICATION ),
    HID_REPORT_ID( REPORT_ID_CONSUMER_CONTROL ),
    HID_LOGICAL_MIN  ( 0x00                                ),
    HID_LOGICAL_MAX_N( 0x03FF, 2                           ),
    HID_USAGE_MIN    ( 0x00                                ),
    HID_USAGE_MAX_N  ( 0x03FF, 2                           ),
    HID_REPORT_COUNT ( 1                                   ),
    HID_REPORT_SIZE  ( 16                                  ),
    HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE ),
  HID_COLLECTION_END,

  //------------- Mouse Report: buttons + dx + dy + scroll + pan -------------//
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     ),
  HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE    ),
  HID_COLLECTION ( HID_COLLECTION_APPLICATION ),
    HID_REPORT_ID( REPORT_ID_MOUSE        ),
    HID_USAGE      (HID_USAGE_DESKTOP_POINTER ),
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL  ),
      HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON ),
        HID_USAGE_MIN    ( 1                                      ),
        HID_USAGE_MAX    ( 5                                      ),
        HID_LOGICAL_MIN  ( 0                                      ),
        HID_LOGICAL_MAX  ( 1                                      ),

        HID_REPORT_COUNT ( 5                                      ), /* Forward, Backward, Middle, Right, Left */
        HID_REPORT_SIZE  ( 1                                      ),
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),

        HID_REPORT_COUNT ( 1                                      ),
        HID_REPORT_SIZE  ( 3                                      ),
        HID_INPUT        ( HID_CONSTANT                           ), /* 5 bit padding followed 3 bit buttons */

      HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP ),
        HID_USAGE        ( HID_USAGE_DESKTOP_X                    ),
        HID_USAGE        ( HID_USAGE_DESKTOP_Y                    ),
        HID_LOGICAL_MIN  ( 0x81                                   ), /* -127 */
        HID_LOGICAL_MAX  ( 0x7f                                   ), /* 127  */

        HID_REPORT_COUNT ( 2                                      ), /* X, Y position */
        HID_REPORT_SIZE  ( 8                                      ),
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), /* relative values */

        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL                ), /* mouse scroll */
        HID_LOGICAL_MIN ( 0x81                                   ), /* -127 */
        HID_LOGICAL_MAX ( 0x7f                                   ), /* 127  */
        HID_REPORT_COUNT( 1                                      ),
        HID_REPORT_SIZE ( 8                                      ), /* 8-bit value */
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), /* relative values */

      HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER ),
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2           ), /* Horizontal wheel scroll */
        HID_LOGICAL_MIN ( 0x81                                   ), /* -127 */
        HID_LOGICAL_MAX ( 0x7f                                   ), /* 127  */
        HID_REPORT_COUNT( 1                                      ),
        HID_REPORT_SIZE ( 8                                      ), /* 8-bit value */
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE ), /* relative values */
    HID_COLLECTION_END,
  HID_COLLECTION_END,

  //------------- Gamepad Report -------------//
  /* Byte 0: 4 pad | 2 Y-axis | 2 X-axis
   * Byte 1: Button7-Button0
   */
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     ),
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  ),
  HID_COLLECTION ( HID_COLLECTION_APPLICATION ),
    HID_REPORT_ID ( REPORT_ID_GAMEPAD      ),
    HID_USAGE      (HID_USAGE_DESKTOP_POINTER ),
    HID_COLLECTION ( HID_COLLECTION_PHYSICAL  ),
      // X,Y joystick
      HID_USAGE    ( HID_USAGE_DESKTOP_X                    ),
      HID_USAGE    ( HID_USAGE_DESKTOP_Y                    ),
      HID_LOGICAL_MIN ( 0xFF                                   ), /* -1 */
      HID_LOGICAL_MAX ( 0x01                                   ), /* 1  */
      HID_REPORT_COUNT( 2                                      ), /* X, Y position */
      HID_REPORT_SIZE ( 2                                      ), /* 2-bit value */
      HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ), /* input values */
    HID_COLLECTION_END,

    /* X,Y padding */
    HID_REPORT_COUNT ( 4                                       ),
    HID_REPORT_SIZE  ( 1                                       ),
    HID_INPUT        ( HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE),

    // Buttons
    HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON ),
      HID_USAGE_MIN    ( 1                                      ),
      HID_USAGE_MAX    ( 8                                      ),
      HID_LOGICAL_MIN  ( 0                                      ),
      HID_LOGICAL_MAX  ( 1                                      ),
      HID_REPORT_COUNT ( 8                                      ),
      HID_REPORT_SIZE  ( 1                                      ),
      HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  HID_COLLECTION_END
#endif
};

BLEHid::BLEHid(void)
  : BLEService(UUID16_SVC_HUMAN_INTERFACE_DEVICE),
    _chr_protocol(UUID16_CHR_PROTOCOL_MODE), _chr_control(UUID16_CHR_HID_CONTROL_POINT),
    _chr_input()
{
  _keyboard_en = true;
  _mouse_en = false;

  _pchr_boot_keyboard = NULL;
}

err_t BLEHid::start(void)
{
  VERIFY_STATUS( this->addToGatt() );

  // Protocol Mode
  if ( _keyboard_en || _mouse_en )
  {
    _chr_protocol.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE_WO_RESP);
    _chr_protocol.setFixedLen(1);
    VERIFY_STATUS( _chr_protocol.start() );
    _chr_protocol.write( (uint8_t) 1);
  }

  // Input reports
  {
    _chr_input.setUuid(UUID16_CHR_REPORT);
    _chr_input.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _chr_input.setFixedLen(sizeof(hid_keyboard_report_t));
    _chr_input.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    _chr_input.setReportRefDescriptor(0x01, 0x01);
    VERIFY_STATUS( _chr_input.start() );
  }

  // Report Map (HID Report Descriptor)
  BLECharacteristic report_map(UUID16_CHR_REPORT_MAP);
  report_map.setProperties(CHR_PROPS_READ);
  report_map.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  report_map.setFixedLen(sizeof(hid_report_descriptor));
  VERIFY_STATUS( report_map.start() );
  report_map.write(hid_report_descriptor, sizeof(hid_report_descriptor));

  // Boot Keyboard Input Report
  if ( _keyboard_en )
  {
    _pchr_boot_keyboard = new BLECharacteristic(UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT);
    _pchr_boot_keyboard->setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    _pchr_boot_keyboard->setFixedLen(sizeof(hid_keyboard_report_t));
    _pchr_boot_keyboard->setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    VERIFY_STATUS(_pchr_boot_keyboard->start());
  }

  // Boot Mouse Input Report
  if ( _mouse_en )
  {

  }

  // HID Information
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
  // bcd 1.1, country = 0, flag = normal connect
  uint8_t hid_info_data[] = { 0x01, 0x01, 0x00, bit(1) };

  BLECharacteristic hid_info(UUID16_CHR_HID_INFORMATION);
  hid_info.setProperties(CHR_PROPS_READ);
  hid_info.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  hid_info.setFixedLen(sizeof(hid_info_data));
  VERIFY_STATUS( hid_info.start() );
  hid_info.write(hid_info_data, sizeof(hid_info_data));

  // HID Control Point
  _chr_control.setProperties(CHR_PROPS_WRITE_WO_RESP);
  _chr_control.setPermission(SECMODE_NO_ACCESS, SECMODE_ENC_NO_MITM);
  _chr_control.setFixedLen(1);
  VERIFY_STATUS( _chr_control.start() );
  _chr_control.write( (uint8_t) 0 );

  return NRF_SUCCESS;
}
