/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "bluefruit.h"

// For using USB HID descriptor template
#include "class/hid/hid_device.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

enum { REPORT_ID_GAMEPAD = 1 };

uint8_t const hid_gamepad_report_descriptor[] =
{
  TUD_HID_REPORT_DESC_GAMEPAD( HID_REPORT_ID(REPORT_ID_GAMEPAD) )
};

BLEHidGamepad::BLEHidGamepad(void)
  : BLEHidGeneric(1, 0, 0)
{

}

err_t BLEHidGamepad::begin(void)
{
  uint16_t input_len [] = { sizeof(hid_gamepad_report_t) };

  setReportLen(input_len, NULL, NULL);
  enableKeyboard(false);
  enableMouse(false);
  setReportMap(hid_gamepad_report_descriptor, sizeof(hid_gamepad_report_descriptor));

  VERIFY_STATUS( BLEHidGeneric::begin() );

  // Attempt to change the connection interval to 11.25-15 ms when starting HID
  Bluefruit.Periph.setConnInterval(9, 12);

  return ERROR_NONE;
}

//--------------------------------------------------------------------+
// Multiple connections
//--------------------------------------------------------------------+

bool BLEHidGamepad::report(uint16_t conn_hdl, hid_gamepad_report_t const* report)
{
  return inputReport(conn_hdl, REPORT_ID_GAMEPAD, report, sizeof(hid_gamepad_report_t));
}

bool BLEHidGamepad::reportButtons(uint16_t conn_hdl, uint32_t button_mask)
{
  hid_gamepad_report_t report = { 0 };
  report.buttons = button_mask;

  return this->report(conn_hdl, &report);
}

bool BLEHidGamepad::reportHat(uint16_t conn_hdl, uint8_t hat)
{
  hid_gamepad_report_t report = { 0 };
  report.hat = hat;

  return this->report(conn_hdl, &report);
}

bool BLEHidGamepad::reportJoystick(uint16_t conn_hdl, int8_t x, int8_t y, int8_t z, int8_t rz, int8_t rx, int8_t ry)
{
  hid_gamepad_report_t report =
  {
    .x  = x , .y  = y,
    .z  = z , .rz = rz,
    .rx = rx, .ry = ry
  };

  return this->report(conn_hdl, &report);
}

//--------------------------------------------------------------------+
// Single connections
//--------------------------------------------------------------------+

bool BLEHidGamepad::report(hid_gamepad_report_t const* report)
{
  return this->report(BLE_CONN_HANDLE_INVALID, report);
}

bool BLEHidGamepad::reportButtons(uint32_t button_mask)
{
  return this->reportButtons(BLE_CONN_HANDLE_INVALID, button_mask);
}

bool BLEHidGamepad::reportHat(uint8_t hat)
{
  return this->reportHat(BLE_CONN_HANDLE_INVALID, hat);
}

bool BLEHidGamepad::reportJoystick(int8_t x, int8_t y, int8_t z, int8_t rz, int8_t rx, int8_t ry)
{
  return this->reportJoystick(BLE_CONN_HANDLE_INVALID, x, y, z, rz, rx, ry);
}
