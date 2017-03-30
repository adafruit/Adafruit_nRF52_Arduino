/**************************************************************************/
/*!
    @file     BLEAncs.cpp
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

/* ANCS Service        : 7905F431-B5CE-4E99-A40F-4B1E122D00D0
 * Notification Source : 9FBF120D-6301-42D9-8C58-25E699A21DBD
 * Control Point       : 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9
 * Data Source         : 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB
 */

const uint8_t BLEANCS_UUID_SERVICE[] =
{
    0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4,
    0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79
};

const uint8_t BLEANCS_UUID_CHR_CONTROL[] =
{
    0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98,
    0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69
};

const uint8_t BLEANCS_UUID_CHR_NOTIFICATION[]
{
    0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C,
    0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F
};

const uint8_t BLEANCS_UUID_CHR_DATA[] =
{
    0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE,
    0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22
};

BLEAncs::BLEAncs(void)
  : BLECentralService(BLEANCS_UUID_SERVICE), _control(BLEANCS_UUID_CHR_CONTROL),
    _notification(BLEANCS_UUID_CHR_NOTIFICATION), _data(BLEANCS_UUID_CHR_DATA)
{

}

bool BLEAncs::begin(void)
{
  // Invoke base class begin()
  BLECentralService::begin();

  // Initialize Discovery module if needed
  if ( !Bluefruit.Discovery.begun() ) Bluefruit.Discovery.begin();

  _control.begin();
  _notification.begin();
  _data.begin();

  return true;
}

bool BLEAncs::discover(uint16_t conn_handle)
{
  // Call BLECentralService discover
  VERIFY( BLECentralService::discover(conn_handle) );
  _discovered = false;

  // Discover characteristics
  BLECentralCharacteristic* chr_arr[] = { &_control, &_notification, &_data };

  VERIFY( 3 == Bluefruit.Discovery.discoverCharacteristic(conn_handle, chr_arr, 3) );

  _discovered = true;
  return true;
}

void BLEAncs::disconnect(void)
{

}
