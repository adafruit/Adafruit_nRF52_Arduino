/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach for Adafruit
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

    // 3: Serials
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
