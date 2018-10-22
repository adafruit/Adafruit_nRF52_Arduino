/**************************************************************************/
/*!
    @file     flash_nrf5x.c
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

#include "flash_nrf5x.h"
#include "flash_cache.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "rtos.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
static SemaphoreHandle_t _sem = NULL;
static volatile uint32_t _op_result;

void flash_nrf5x_event_cb (uint32_t event)
{
  _op_result = event;
  xSemaphoreGive(_sem);
}

// Flash Abstraction Layer
static bool fal_erase (uint32_t addr);
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
void flash_nrf5x_flush (void)
{
  flash_cache_flush(&_cache);
}

uint32_t flash_nrf5x_write (uint32_t dst, void const * src, uint32_t len)
{
  // TODO prevent write SD + bootloader region
  return flash_cache_write(&_cache, dst, src, len);
}

uint32_t flash_nrf5x_read (void* dst, uint32_t src, uint32_t len)
{
  // return cache value if available
  flash_cache_read(&_cache, dst, src, len);
  return len;
}

bool flash_nrf5x_erase(uint32_t addr)
{
  return fal_erase(addr);
}

//--------------------------------------------------------------------+
// HAL for caching
//--------------------------------------------------------------------+
bool fal_erase (uint32_t addr)
{
  // Init semaphore for first call
  if ( !_sem )
  {
    _sem = xSemaphoreCreateBinary();
    VERIFY(_sem);
  }

  // delay and retry if busy
  uint32_t err;
  while ( NRF_ERROR_BUSY == (err = sd_flash_page_erase(addr / FLASH_NRF52_PAGE_SIZE)) )
  {
    delay(1);
  }
  VERIFY_STATUS(err);

  // wait for async event if SD is enabled
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  if ( sd_en )
  {
    xSemaphoreTake(_sem, portMAX_DELAY);
    VERIFY(_op_result == NRF_EVT_FLASH_OPERATION_SUCCESS);
  }
  
  return true;
}

static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len)
{
  // delay and try again if busy
  uint32_t err;
  while ( NRF_ERROR_BUSY == (err = sd_flash_write((uint32_t*) dst, (uint32_t const *) src, len / 4)) )
  {
    delay(1);
  }
  VERIFY_STATUS(err);

  // wait for async event if SD is enabled
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  if ( sd_en )
  {
    xSemaphoreTake(_sem, portMAX_DELAY);
    VERIFY(_op_result == NRF_EVT_FLASH_OPERATION_SUCCESS);
  }

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


