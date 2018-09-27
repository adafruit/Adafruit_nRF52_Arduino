/**************************************************************************/
/*!
    @file     flash_cache.h
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

#ifndef FLASH_CACHE_H_
#define FLASH_CACHE_H_

#include <stdint.h>
#include <stdbool.h>

#define FLASH_API_PAGE_SIZE       4096
#define FLASH_CACHE_INVALID_ADDR  0xffffffff

typedef struct
{
  void (*erase) (uint32_t addr);
  uint32_t (*program) (uint32_t dst, void const * src, uint32_t len);
  uint32_t (*read) (void* dst, uint32_t src, uint32_t len);
  bool (*verify) (uint32_t addr, void const * buf, uint32_t len);

  uint32_t cache_addr;
  uint8_t cache_buf[FLASH_API_PAGE_SIZE] __attribute__((aligned(4)));
} flash_cache_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t flash_cache_write (flash_cache_t* fc, uint32_t dst, void const *src, uint32_t count);
void flash_cache_flush (flash_cache_t* fc);

#ifdef __cplusplus
 }
#endif

#endif /* FLASH_CACHE_H_ */
