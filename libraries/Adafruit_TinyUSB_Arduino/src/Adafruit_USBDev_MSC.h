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

#ifndef ADAFRUIT_USBDEV_MSC_H_
#define ADAFRUIT_USBDEV_MSC_H_

#include "Adafruit_TinyUSB_core.h"

class Adafruit_USBD_MSC : Adafruit_USBD_Interface
{
  public:
    typedef int32_t (*read_callback_t ) (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);
    typedef int32_t (*write_callback_t) (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);
    typedef void    (*flush_callback_t) (uint8_t lun);

    Adafruit_USBD_MSC(void);

    bool begin(void);

    void setCapacity(uint32_t block_count, uint16_t block_size);
    void getCapacity(uint32_t* block_count, uint16_t* block_size);
    void setCallback(read_callback_t rd_cb, write_callback_t wr_cb, flush_callback_t fl_cb);

    // from Adafruit_USBInterface
    virtual uint16_t getDescriptor(uint8_t* buf, uint16_t bufsize);

  private:
    uint32_t _block_count;
    uint16_t _block_size;

    read_callback_t  _rd_cb;
    write_callback_t _wr_cb;
    flush_callback_t _fl_cb;

    friend int32_t tud_msc_read10_cb (uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);
    friend int32_t tud_msc_write10_cb (uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);
    friend void tud_msc_write10_complete_cb (uint8_t lun);
};

#endif /* ADAFRUIT_USBDEV_MSC_H_ */
