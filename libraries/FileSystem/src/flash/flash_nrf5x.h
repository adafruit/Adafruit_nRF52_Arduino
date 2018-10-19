/**************************************************************************/
/*!
    @file     flash_nrf5x.h
    @author   hathach (tinyusb.org)

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

#ifndef FLASH_NRF52_H_
#define FLASH_NRF52_H_

#include "common_inc.h"

#define FLASH_NRF52_PAGE_SIZE   4096

#ifdef __cplusplus
 extern "C" {
#endif

void flash_nrf5x_flush (void);
bool flash_nrf5x_erase(uint32_t addr);

uint32_t flash_nrf5x_write (uint32_t dst, void const * src, uint32_t len);
uint32_t flash_nrf5x_read (void* dst, uint32_t src, uint32_t len);

static inline uint32_t flash_nrf5x_write8 (uint32_t dst, uint8_t num)
{
  flash_nrf5x_write(dst, &num, sizeof(num));
  return sizeof(num);
}

static inline uint32_t flash_nrf5x_write16 (uint32_t dst, uint8_t num)
{
  flash_nrf5x_write(dst, &num, sizeof(num));
  return sizeof(num);
}
static inline uint32_t flash_nrf5x_write32 (uint32_t dst, uint8_t num)
{
  flash_nrf5x_write(dst, &num, sizeof(num));
  return sizeof(num);
}

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
