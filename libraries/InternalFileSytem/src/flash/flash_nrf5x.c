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

#include "flash_nrf5x.h"
#include "flash_cache.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "delay.h"
#include "rtos.h"


#ifdef NRF52840_XXAA
  #define BOOTLOADER_ADDR        0xF4000
#else
  #define BOOTLOADER_ADDR        0x74000
#endif

// defined in linker script
extern uint32_t __flash_arduino_start[];
//extern uint32_t __flash_arduino_end[];

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
static SemaphoreHandle_t _sem = NULL;
static bool _flash_op_failed = false;

void flash_nrf5x_event_cb (uint32_t event)
{
  if ( _sem ) {
    // Record the result, for consumption by fal_erase or fal_program
    // Used to reattempt failed operations
    _flash_op_failed = (event == NRF_EVT_FLASH_OPERATION_ERROR);

    // Signal to fal_erase or fal_program that our async flash op is now complete
    xSemaphoreGive(_sem);
  } 
}

// How many retry attempts when performing flash operations 
#define MAX_RETRY 20

// Check whether a flash operation was successful, or should be repeated
static bool retry_flash_op (uint32_t op_result, bool sd_enabled) {
  // If busy
  if (op_result == NRF_ERROR_BUSY) {
    delay(1);
    return true; // Retry
  }

  // If unspecified error
  if (op_result != NRF_SUCCESS)
    return true; // Retry

  // If the soft device is enabled, flash operations run async
  // The callback (flash_nrf5x_event_cb) will give semaphore when the flash operation is complete
  // The callback also checks for NRF_EVT_FLASH_OPERATION_ERROR, which is not available to us otherwise
  if (sd_enabled) {
    xSemaphoreTake(_sem, portMAX_DELAY);
    if (_flash_op_failed) 
      return true; // Retry
  }

  // Success
  return false;
}

// Flash Abstraction Layer
static bool fal_erase (uint32_t addr);
static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len);
static uint32_t fal_read (void* dst, uint32_t src, uint32_t len);
static bool fal_verify (uint32_t addr, void const * buf, uint32_t len);

static uint8_t _cache_buffer[FLASH_CACHE_SIZE] __attribute__((aligned(4)));

static flash_cache_t _cache =
{
  .erase      = fal_erase,
  .program    = fal_program,
  .read       = fal_read,
  .verify     = fal_verify,

  .cache_addr = FLASH_CACHE_INVALID_ADDR,
  .cache_buf  = _cache_buffer
};

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
void flash_nrf5x_flush (void)
{
  flash_cache_flush(&_cache);
}

int flash_nrf5x_write (uint32_t dst, void const * src, uint32_t len)
{
  // Softdevice region
  VERIFY(dst >= ((uint32_t) __flash_arduino_start), -1);

  // Bootloader region
  VERIFY(dst < BOOTLOADER_ADDR, -1);

  return flash_cache_write(&_cache, dst, src, len);
}

int flash_nrf5x_read (void* dst, uint32_t src, uint32_t len)
{
  return flash_cache_read(&_cache, dst, src, len);
}

bool flash_nrf5x_erase(uint32_t addr)
{
  return fal_erase(addr);
}

//--------------------------------------------------------------------+
// HAL for caching
//--------------------------------------------------------------------+
static bool fal_erase (uint32_t addr)
{
  // Init semaphore for first call
  if ( _sem == NULL )
  {
    _sem = xSemaphoreCreateBinary();
    VERIFY(_sem);
  }

  // Check if soft device is enabled
  // If yes, flash operations are async, so we need to wait for the callback to give the semaphore
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  // Make multiple attempts to erase
  uint8_t attempt = 0;
  while (retry_flash_op(sd_flash_page_erase(addr / FLASH_NRF52_PAGE_SIZE), sd_en)) {
    if (++attempt > MAX_RETRY)
      return false; // Failure
  }
    return true; // Success
}

static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len)
{
  // Check if soft device is enabled
  // If yes, flash operations are async, so we need to wait for the callback to give the semaphore
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  uint32_t err;

  // Somehow S140 v6.1.1 assert an error when writing a whole page
  // https://devzone.nordicsemi.com/f/nordic-q-a/40088/sd_flash_write-cause-nrf_fault_id_sd_assert
  // Workaround: write half page at a time.
#if NRF52832_XXAA
  uint8_t attempt = 0;
  while (retry_flash_op(sd_flash_write((uint32_t*) dst, (uint32_t const *) src, len/4), sd_en)) {
    if (++attempt > MAX_RETRY)
      return 0; // Failure
  }
#else

  // First part of block
  uint8_t attempt = 0;
  while (retry_flash_op(sd_flash_write((uint32_t*) dst, (uint32_t const *) src, len/8), sd_en)) {
    if (++attempt > MAX_RETRY)
      return 0; // Failure
  }

  // Second part of block
  attempt = 0;
  while (retry_flash_op(sd_flash_write((uint32_t*) (dst+ len/2), (uint32_t const *) (src + len/2), len/8), sd_en)) {
    if (++attempt > MAX_RETRY)
      return 0; // Failure
  }
#endif

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
