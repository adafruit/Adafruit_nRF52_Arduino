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
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_MCU                OPT_MCU_NRF5X

#ifdef NRF52840_XXAA
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE
#else
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_NONE
#endif

#define CFG_TUSB_OS                 OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG              0

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENDOINT0_SIZE       64

/*------------- Descriptors -------------*/

/* Enable auto generated descriptor, tinyusb will try its best to create
 * descriptor ( device, configuration, hid ) that matches enabled CFG_* in this file
 *
 * Note: All CFG_TUD_DESC_* are relevant only if CFG_TUD_DESC_AUTO is enabled
 */
#define CFG_TUD_DESC_AUTO           0

//------------- CLASS -------------//
#define CFG_TUD_CDC                 1

// disable msc for feather nrf52840 for now until have a more stable QSPI driver
#define CFG_TUD_MSC                 1

#define CFG_TUD_HID                 0
#define CFG_TUD_HID_KEYBOARD        0
#define CFG_TUD_HID_MOUSE           0

/* Use Boot Protocol for Keyboard, Mouse. Enable this will create separated HID interface
 * require more IN endpoints. If disabled, they they are all packed into a single
 * multiple report interface called "Generic". */
#define CFG_TUD_HID_KEYBOARD_BOOT   0
#define CFG_TUD_HID_MOUSE_BOOT      0


//--------------------------------------------------------------------
// CDC
//--------------------------------------------------------------------

// FIFO size of CDC TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE      256
#define CFG_TUD_CDC_TX_BUFSIZE      256

//--------------------------------------------------------------------
// MSC
//--------------------------------------------------------------------

// Number of supported Logical Unit Number (At least 1)
#define CFG_TUD_MSC_MAXLUN          1

// Buffer size of Device Mass storage
#define CFG_TUD_MSC_BUFSIZE         512

// Block size
#define CFG_TUD_MSC_BLOCK_SZ        512

// Vendor name included in Inquiry response, max 8 bytes
#define CFG_TUD_MSC_VENDOR          "Adafruit"

// Product name included in Inquiry response, max 16 bytes
#define CFG_TUD_MSC_PRODUCT         "Bluefruit nRF52"

// Product revision string included in Inquiry response, max 4 bytes
#define CFG_TUD_MSC_PRODUCT_REV     "1.0"

//--------------------------------------------------------------------
// HID
//--------------------------------------------------------------------

/* Use the HID_ASCII_TO_KEYCODE lookup if CFG_TUD_HID_KEYBOARD is enabled.
 * This will occupies 256 bytes of ROM. It will also enable the use of 2 extra APIs
 * - tud_hid_keyboard_send_char()
 * - tud_hid_keyboard_send_string()
 */
#define CFG_TUD_HID_ASCII_TO_KEYCODE_LOOKUP 1

//--------------------------------------------------------------------
// USB RAM PLACEMENT
//--------------------------------------------------------------------
#define CFG_TUSB_ATTR_USBRAM
#define CFG_TUSB_MEM_ALIGN          ATTR_ALIGNED(4)


#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
