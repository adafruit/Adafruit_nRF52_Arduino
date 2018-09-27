/**************************************************************************/
/*!
    @file     flash_nrf52.c
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

#include "../flash/flash_nrf52.h"

#include "../flash/flash_cache.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// Flash Abstraction Layer
static void fal_erase (uint32_t addr);
static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len);
static uint32_t fal_read (void* dst, uint32_t src, uint32_t len);
static bool fal_verify (uint32_t addr, void const * buf, uint32_t len);

flash_cache_t _cache = {
  .erase = fal_erase,
  .program = fal_program,
  .read = fal_read,
  .verify = fal_verify,
  .cache_addr = FLASH_CACHE_INVALID_ADDR
};

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
void flash_nrf52_flush (void)
{
  flash_cache_flush(&_cache);
}

uint32_t flash_nrf52_write (uint32_t dst, void const * src, uint32_t len)
{
  // TODO prevent write SD + bootloader region
  return flash_cache_write(&_cache, dst, src, len);
}

uint32_t flash_nrf52_read (void* dst, uint32_t src, uint32_t len)
{
  return fal_read(dst, src, len);
}

//--------------------------------------------------------------------+
// HAL for caching
//--------------------------------------------------------------------+
static void fal_erase (uint32_t addr)
{
  // TODO SD enabled
  nrf_nvmc_page_erase(addr);
}

static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len)
{
  // TODO SD enabled
  nrf_nvmc_write_words(dst, (uint32_t const *) src, len / 4);
  return len;
}

static uint32_t fal_read (void* dst, uint32_t src, uint32_t len)
{
  memcpy(dst, (void*) src, len);
  return len;
}

static bool fal_verify (uint32_t addr, void const * buf, uint32_t len)
{
  return 0 == memcmp((void*) addr, buf, len);
}


