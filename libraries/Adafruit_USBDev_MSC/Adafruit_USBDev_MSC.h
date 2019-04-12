/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 hathach for Adafruit Industries
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

#ifndef ADAFRUIT_USBDEV_MSC_H_
#define ADAFRUIT_USBDEV_MSC_H_

#include "Adafruit_USBDevice.h"

class Adafruit_USBDev_MSC : Adafruit_USBDev_Interface
{
  private:
    uint32_t _block_count;
    uint16_t _block_size;

  public:
    Adafruit_USBDev_MSC(void);

    bool begin(void);

    void setCapacity(uint32_t block_count, uint16_t block_size);
    void getCapacity(uint32_t* block_count, uint16_t* block_size);

    // from Adafruit_USBInterface
    virtual uint16_t getDescriptor(uint8_t* buf, uint16_t bufsize);
};

#endif /* ADAFRUIT_USBDEV_MSC_H_ */
