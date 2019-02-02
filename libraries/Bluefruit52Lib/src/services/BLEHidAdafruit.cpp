/**************************************************************************/
/*!
    @file     BLEHidAdafruit.cpp
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
    HID_REPORT_ID ( REPORT_ID_KEYBOARD        ),
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

    // 6-byte Keycodes
    HID_USAGE_PAGE (HID_USAGE_PAGE_KEYBOARD),
      HID_USAGE_MIN    ( 0                                   ),
      HID_USAGE_MAX    ( 255                                 ),
      HID_LOGICAL_MIN  ( 0                                   ),
      HID_LOGICAL_MAX  ( 255                                 ),

      HID_REPORT_COUNT ( 6                                   ),
      HID_REPORT_SIZE  ( 8                                   ),
      HID_INPUT        ( HID_DATA | HID_ARRAY | HID_ABSOLUTE ),

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
  HID_COLLECTION_END,

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

#if 0
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
      HID_REPORT_COUNT ( 8                                      ),    // Keyboard
      HID_REPORT_SIZE  ( 1                                      ),
      HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  HID_COLLECTION_END
#endif
};

BLEHidAdafruit::BLEHidAdafruit(void)
  : BLEHidGeneric(3, 1, 0)
{
  _mse_buttons = 0;
  _kbd_led_cb = NULL;
}

err_t BLEHidAdafruit::begin(void)
{
  uint16_t input_len [] = { sizeof(hid_keyboard_report_t),  sizeof(hid_consumer_control_report_t), sizeof(hid_mouse_report_t) };
  uint16_t output_len[] = { 1 };

  setReportLen(input_len, output_len, NULL);
  enableKeyboard(true);
  enableMouse(true);
  setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));

  VERIFY_STATUS( BLEHidGeneric::begin() );

  // Attempt to change the connection interval to 11.25-15 ms when starting HID
  Bluefruit.Periph.setConnInterval(9, 12);

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* Keyboard
 *------------------------------------------------------------------*/

void blehid_ada_keyboard_output_cb(BLECharacteristic* chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  LOG_LV2("HID", "Keyboard LED : 0x%02X", data[0]);
  VERIFY(len == 1, );

  BLEHidAdafruit& svc = (BLEHidAdafruit&) chr->parentService();
  if ( svc._kbd_led_cb ) svc._kbd_led_cb(data[0]);
}

void BLEHidAdafruit::setKeyboardLedCallback(kbd_led_cb_t fp)
{
  _kbd_led_cb = fp;

  // Report mode
  this->setOutputReportCallback(REPORT_ID_KEYBOARD, fp ? blehid_ada_keyboard_output_cb : NULL);

  // Boot mode
  _chr_boot_keyboard_output->setWriteCallback(fp ? blehid_ada_keyboard_output_cb : NULL);
}

bool BLEHidAdafruit::keyboardReport(hid_keyboard_report_t* report)
{
  if ( isBootMode() )
  {
    return bootKeyboardReport(report, sizeof(hid_keyboard_report_t));
  }else
  {
    return inputReport( REPORT_ID_KEYBOARD, report, sizeof(hid_keyboard_report_t));
  }
}

bool BLEHidAdafruit::keyboardReport(uint8_t modifier, uint8_t keycode[6])
{
  hid_keyboard_report_t report =
  {
      .modifier = modifier,
  };
  memcpy(report.keycode, keycode, 6);

  return keyboardReport(&report);
}

bool BLEHidAdafruit::keyboardReport(uint8_t modifier, uint8_t keycode0, uint8_t keycode1, uint8_t keycode2, uint8_t keycode3, uint8_t keycode4, uint8_t keycode5)
{
  hid_keyboard_report_t report =
  {
      .modifier = modifier,
      .reserved = 0,
      .keycode  = { keycode0, keycode1, keycode2, keycode3, keycode4, keycode5 }
  };

  return keyboardReport(&report);
}

bool BLEHidAdafruit::keyPress(char ch)
{
  hid_keyboard_report_t report;
  varclr(&report);

  report.modifier = ( HID_ASCII_TO_KEYCODE[(uint8_t)ch].shift ) ? KEYBOARD_MODIFIER_LEFTSHIFT : 0;
  report.keycode[0] = HID_ASCII_TO_KEYCODE[(uint8_t)ch].keycode;

  return keyboardReport(&report);
}

bool BLEHidAdafruit::keyRelease(void)
{
  hid_keyboard_report_t report;
  varclr(&report);

  return keyboardReport(&report);
}

bool BLEHidAdafruit::keySequence(const char* str, int interal)
{
  // Send each key in sequence
  char ch;
  while( (ch = *str++) != 0 )
  {
    char lookahead = *str;

    keyPress(ch);
    delay(interal);

    /* Only need to empty report if the next character is NULL or the same with
     * the current one, else no need to send */
    if ( lookahead == ch || lookahead == 0 )
    {
      keyRelease();
      delay(interal);
    }
  }

  return true;
}

/*------------------------------------------------------------------*/
/* Consumer Media Key
 *------------------------------------------------------------------*/
bool BLEHidAdafruit::consumerReport(uint16_t usage_code)
{
  return inputReport( REPORT_ID_CONSUMER_CONTROL, &usage_code, sizeof(usage_code));
}

bool BLEHidAdafruit::consumerKeyPress(uint16_t usage_code)
{
  return consumerReport(usage_code);
}

bool BLEHidAdafruit::consumerKeyRelease(void)
{
  uint16_t usage = 0;
  return consumerReport(usage);
}

/*------------------------------------------------------------------*/
/* Mouse
 *------------------------------------------------------------------*/
bool BLEHidAdafruit::mouseReport(hid_mouse_report_t* report)
{
  if ( isBootMode() )
  {
    return bootMouseReport(report, sizeof(hid_mouse_report_t));
  }else
  {
    return inputReport( REPORT_ID_MOUSE, report, sizeof(hid_mouse_report_t));
  }
}

bool BLEHidAdafruit::mouseReport(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t pan)
{
  hid_mouse_report_t report =
  {
      .buttons = buttons,
      .x       = x,
      .y       = y,
      .wheel   = wheel,
      .pan     = pan
  };

  _mse_buttons = buttons;

  return mouseReport(&report);
}

bool BLEHidAdafruit::mouseButtonPress(uint8_t buttons)
{
  _mse_buttons = buttons;
  return mouseReport(buttons, 0, 0, 0, 0);
}

bool BLEHidAdafruit::mouseButtonRelease(void)
{
  return mouseReport(0, 0, 0, 0, 0);
}

bool BLEHidAdafruit::mouseMove(int8_t x, int8_t y)
{
  return mouseReport(_mse_buttons, x, y, 0, 0);
}

bool BLEHidAdafruit::mouseScroll(int8_t scroll)
{
  return mouseReport(_mse_buttons, 0, 0, scroll, 0);
}

bool BLEHidAdafruit::mousePan(int8_t pan)
{
  return mouseReport(_mse_buttons, 0, 0, 0, pan);
}
