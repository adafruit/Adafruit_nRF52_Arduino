/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
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

#ifndef ADAFRUIT_TINYUSB_CORE_H_
#define ADAFRUIT_TINYUSB_CORE_H_

#ifndef USE_TINYUSB
#error TinyUSB is not selected, please select it in Tools->Menu->USB Stack
#endif

#include "tusb.h"

#ifdef __cplusplus
#include "Adafruit_USBD_Device.h"
#include "Adafruit_USBD_CDC.h"
#endif

// Called by main.cpp to initialize usb device typically with CDC device for Serial
void Adafruit_TinyUSB_Core_init(void);

// Invoked when host disconnects cdc at baud 1200, usually touch feature to go into DFU mode
void Adafruit_TinyUSB_Core_touch1200(void);

#endif /* ADAFRUIT_TINYUSB_CORE_H_ */
