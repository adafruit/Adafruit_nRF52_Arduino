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

#ifndef ADAFRUIT_USBD_DEVICE_H_
#define ADAFRUIT_USBD_DEVICE_H_

#include "tusb.h"

class Adafruit_USBD_Interface
{
  public:
    virtual uint16_t getDescriptor(uint8_t itfnum, uint8_t* buf, uint16_t bufsize) = 0;
};

class Adafruit_USBD_Device
{
  private:
    tusb_desc_device_t _desc_device;

    uint8_t  _desc_cfg[256];
    uint16_t _desc_cfglen;

    uint8_t  _itf_count;

    uint8_t  _epin_count;
    uint8_t  _epout_count;

  public:
    Adafruit_USBD_Device(void);

    bool addInterface(Adafruit_USBD_Interface& itf);

    void setID(uint16_t vid, uint16_t pid);
    void setVersion(uint16_t bcd);
    bool begin(void);

    bool mounted      (void) { return tud_mounted(); }
    bool suspended    (void) { return tud_suspended(); }
    bool ready        (void) { return tud_ready(); }
    bool remoteWakeup (void) { return tud_remote_wakeup(); }

    friend uint8_t const * tud_descriptor_device_cb(void);
    friend uint8_t const * tud_descriptor_configuration_cb(uint8_t index);
};

extern Adafruit_USBD_Device USBDevice;

#endif /* ADAFRUIT_USBD_DEVICE_H_ */
