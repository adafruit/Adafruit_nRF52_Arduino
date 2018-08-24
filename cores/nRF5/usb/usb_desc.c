/**************************************************************************/
/*!
    @file     usb_desc.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
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
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    This file is part of the tinyusb stack.
*/
/**************************************************************************/

#ifdef NRF52840_XXAA

#include "tusb.h"


/*------------- Interface Numbering -------------*/
enum {
    ITF_STR_LANGUAGE = 0 ,
    ITF_STR_MANUFACTURER ,
    ITF_STR_PRODUCT      ,
    ITF_STR_SERIAL       ,
    ITF_STR_CDC          ,
    ITF_STR_MSC
};

//--------------------------------------------------------------------+
// STRING DESCRIPTORS
//--------------------------------------------------------------------+

// Serial is 64-bit DeviceID -> 16 chars len
uint16_t usb_desc_str_serial[1+16] = { TUD_DESC_STR_HEADER(16) };

// array of pointer to string descriptors
uint16_t const * const string_desc_arr [] =
{
    // 0: is supported language = English
    TUD_DESC_STRCONV(0x0409),

    // 1: Manufacturer
    TUD_DESC_STRCONV('A','d','a','f','r','u','i','t',' ','I','n','d','u','s','t','r','i','e','s'),

    // 2: Product
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','n','R','F','5','2','8','4','0'),

    // 3: Serials TODO use chip ID
    usb_desc_str_serial,

    // 4: CDC Interface
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','S','e','r','i','a','l'),

    // 5: MSC Interface
    TUD_DESC_STRCONV('B','l','u','e','f','r','u','i','t',' ','U','F','2'),
};


// tud_desc_set is required by tinyusb stack
// since CFG_TUD_DESC_AUTO is enabled, we only need to set string_arr 
tud_desc_set_t tud_desc_set =
{
    .device       = NULL,
    .config       = NULL,
    .string_arr   = (uint8_t const **) string_desc_arr,
    .string_count = sizeof(string_desc_arr)/sizeof(string_desc_arr[0]),

    .hid_report =
    {
        .generic       = NULL,
        .boot_keyboard = NULL,
        .boot_mouse    = NULL
    }
};

#endif
