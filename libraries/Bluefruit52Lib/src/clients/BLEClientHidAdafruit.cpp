/**************************************************************************/
/*!
    @file     BLEClientHidAdafruit.cpp
    @author   hathach (tinyusb.org) (tinyusb.org)

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

BLEClientHidAdafruit::BLEClientHidAdafruit(void)
 : BLEClientService(UUID16_SVC_HUMAN_INTERFACE_DEVICE),
   _protcol_mode(UUID16_CHR_PROTOCOL_MODE),
   _hid_info(UUID16_CHR_HID_INFORMATION), _hid_control(UUID16_CHR_HID_CONTROL_POINT),
   _kbd_boot_input(UUID16_CHR_BOOT_KEYBOARD_INPUT_REPORT), _kbd_boot_output(UUID16_CHR_BOOT_KEYBOARD_OUTPUT_REPORT),
   _mse_boot_input(UUID16_CHR_BOOT_MOUSE_INPUT_REPORT)
{
  _kbd_cb = NULL;
  _mse_cb = NULL;
  varclr(&_last_kbd_report);
  varclr(&_last_mse_report);
}


void kbd_client_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEClientHidAdafruit& svc = (BLEClientHidAdafruit&) chr->parentService();
  svc._handle_kbd_input(data, len);
}

void mse_client_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEClientHidAdafruit& svc = (BLEClientHidAdafruit&) chr->parentService();
  svc._handle_mse_input(data, len);
}


bool BLEClientHidAdafruit::begin(void)
{
  // Invoke base class begin()
  BLEClientService::begin();

  _protcol_mode.begin(this);
  _hid_info.begin(this);
  _hid_control.begin(this);

  _kbd_boot_input.begin(this);
  _kbd_boot_output.begin(this);

  _mse_boot_input.begin(this);


  // set notify callback
  _kbd_boot_input.setNotifyCallback(kbd_client_notify_cb);
  _mse_boot_input.setNotifyCallback(mse_client_notify_cb);

  return true;
}

void BLEClientHidAdafruit::setKeyboardReportCallback(kbd_callback_t fp)
{
  _kbd_cb = fp;
}

void BLEClientHidAdafruit::setMouseReportCallback(mse_callback_t fp)
{
  _mse_cb = fp;
}

bool BLEClientHidAdafruit::discover(uint16_t conn_handle)
{
  // Call Base class discover
  VERIFY( BLEClientService::discover(conn_handle) );
  _conn_hdl = BLE_CONN_HANDLE_INVALID; // make as invalid until we found all chars

  // Discover all characteristics
  Bluefruit.Discovery.discoverCharacteristic(conn_handle, _protcol_mode, _kbd_boot_input, _kbd_boot_output, _mse_boot_input, _hid_info, _hid_control);

  VERIFY( _protcol_mode.discovered() && _hid_info.discovered() && _hid_control.discovered() );
  VERIFY ( keyboardPresent() || mousePresent() );

  _conn_hdl = conn_handle;
  return true;
}

/*------------------------------------------------------------------*/
/* Info
 *------------------------------------------------------------------*/
bool BLEClientHidAdafruit::getHidInfo(uint8_t info[4])
{
  return 4 == _hid_info.read(info, 4);
}

uint8_t BLEClientHidAdafruit::getCountryCode(void)
{
  uint8_t info[4] = { 0 };
  getHidInfo(info);

  return info[2];
}

bool BLEClientHidAdafruit::setBootMode(bool boot)
{
  // 0 is boot, 1 is protocol
  return _protcol_mode.write8(1-boot);
}

/*------------------------------------------------------------------*/
/* Keyboard
 *------------------------------------------------------------------*/
bool BLEClientHidAdafruit::keyboardPresent(void)
{
  return _kbd_boot_input.discovered() && _kbd_boot_output.discovered();
}

bool BLEClientHidAdafruit::enableKeyboard(void)
{
  return _kbd_boot_input.enableNotify();
}

bool BLEClientHidAdafruit::disableKeyboard(void)
{
  return _kbd_boot_input.disableNotify();
}

void BLEClientHidAdafruit::_handle_kbd_input(uint8_t* data, uint16_t len)
{
  varclr(&_last_kbd_report);
  memcpy(&_last_kbd_report, data, len);

  if ( _kbd_cb ) _kbd_cb(&_last_kbd_report);
}

void BLEClientHidAdafruit::getKeyboardReport(hid_keyboard_report_t* report)
{
  memcpy(report, &_last_kbd_report, sizeof(hid_keyboard_report_t));
}

/*------------------------------------------------------------------*/
/* Mouse
 *------------------------------------------------------------------*/
bool BLEClientHidAdafruit::mousePresent(void)
{
  return _mse_boot_input.discovered();
}

bool BLEClientHidAdafruit::enableMouse(void)
{
  return _mse_boot_input.enableNotify();
}

bool BLEClientHidAdafruit::disableMouse(void)
{
  return _mse_boot_input.disableNotify();
}

void BLEClientHidAdafruit::_handle_mse_input(uint8_t* data, uint16_t len)
{
  varclr(&_last_mse_report);
  memcpy(&_last_mse_report, data, len);

  if ( _mse_cb ) _mse_cb(&_last_mse_report);
}

void BLEClientHidAdafruit::getMouseReport(hid_mouse_report_t* report)
{
  memcpy(report, &_last_mse_report, sizeof(hid_mouse_report_t));
}

