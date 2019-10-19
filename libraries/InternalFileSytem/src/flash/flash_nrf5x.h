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

#ifndef FLASH_NRF52_H_
#define FLASH_NRF52_H_

#include "common_inc.h"

#define FLASH_NRF52_PAGE_SIZE   4096

#ifdef __cplusplus
 extern "C" {
#endif

void flash_nrf5x_flush (void);
bool flash_nrf5x_erase(uint32_t addr);

int flash_nrf5x_write (uint32_t dst, void const * src, uint32_t len);
int flash_nrf5x_read (void* dst, uint32_t src, uint32_t len);

//--------------------------------------------------------------------+
// Write helper
//--------------------------------------------------------------------+
static inline int flash_nrf5x_write8 (uint32_t dst, uint8_t num)
{
  return flash_nrf5x_write(dst, &num, sizeof(num));
}

static inline int flash_nrf5x_write16 (uint32_t dst, uint8_t num)
{
  return flash_nrf5x_write(dst, &num, sizeof(num));
}
static inline int flash_nrf5x_write32 (uint32_t dst, uint8_t num)
{
  return flash_nrf5x_write(dst, &num, sizeof(num));
}

//--------------------------------------------------------------------+
// Read helper
//--------------------------------------------------------------------+
static inline uint8_t flash_nrf5x_read8 (uint32_t src)
{
  uint8_t num;
  flash_nrf5x_read(&num, src, sizeof(num));
  return num;
}

static inline uint16_t flash_nrf5x_read16 (uint32_t src)
{
  uint16_t num;
  flash_nrf5x_read(&num, src, sizeof(num));
  return num;
}

static inline uint16_t flash_nrf5x_read32 (uint32_t src)
{
  uint32_t num;
  flash_nrf5x_read(&num, src, sizeof(num));
  return num;
}


#ifdef __cplusplus
 }
#endif

#endif /* FLASH_NRF52_H_ */
