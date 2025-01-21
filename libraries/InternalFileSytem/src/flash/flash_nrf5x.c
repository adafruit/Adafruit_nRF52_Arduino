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
#include "assert.h"


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
static uint32_t _flash_op_result = NRF_EVT_FLASH_OPERATION_SUCCESS;

void flash_nrf5x_event_cb (uint32_t event)
{
  if ( _sem ) {
    // Record the result, for consumption by fal_erase or fal_program
    // Used to reattempt failed operations
    _flash_op_result = event;

    // Signal to fal_erase or fal_program that our async flash op is now complete
    xSemaphoreGive(_sem);
  } 
}

// How many retry attempts when performing flash operations 
#define MAX_RETRY 20

// When soft device is enabled, flash ops are async
// Eventual success is reported via callback, which we await
static uint32_t wait_for_async_flash_op_completion(uint32_t initial_result) 
{
  // If initial result not NRF_SUCCESS, no need to await callback
  // We will pass the initial result (failure) straight through
  int32_t result = initial_result;

  // Operation was queued successfully
  if (initial_result == NRF_SUCCESS) {

    // Wait for result via callback 
    xSemaphoreTake(_sem, portMAX_DELAY);
    
    // If completed successfully
    if (_flash_op_result == NRF_EVT_FLASH_OPERATION_SUCCESS) { 
      result = NRF_SUCCESS; 
    }

    // If general failure.
    // The comment on NRF_EVT_FLASH_OPERATION_ERROR describes it as a timeout,
    // so we're using a similar error when translating from NRF_SOC_EVTS type to the global NRF52 error defines
    else if (_flash_op_result == NRF_EVT_FLASH_OPERATION_ERROR) { 
      result = NRF_ERROR_TIMEOUT; 
    }
    
    // If this assert triggers, we need to implement a new NRF_SOC_EVTS value
    else { 
      assert(false); 
    }
  }
  
  return result;
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
  if ( _sem == NULL ) {
    _sem = xSemaphoreCreateBinary();
    VERIFY(_sem);
  }

  // Check if soft device is enabled
  // If yes, flash operations are async, so we need to wait for the callback to give the semaphore
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  // Erase the page
  // Multiple attempts if needed
  uint32_t err;
  for (uint8_t attempt = 0; attempt < MAX_RETRY; ++attempt) {
    err = sd_flash_page_erase(addr / FLASH_NRF52_PAGE_SIZE);

    if (sd_en) { 
      err = wait_for_async_flash_op_completion(err); // Only async if soft device enabled
    }
    if (err == NRF_SUCCESS) {
      break;
    }
    if (err == NRF_ERROR_BUSY) {
      delay(1);
    }
  }
  VERIFY_STATUS(err, false); // Return false if all retries fail

  return true; // Successfully erased
}

static uint32_t fal_program (uint32_t dst, void const * src, uint32_t len)
{
  // wait for async event if SD is enabled
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  uint32_t err;

  // Somehow S140 v6.1.1 assert an error when writing a whole page
  // https://devzone.nordicsemi.com/f/nordic-q-a/40088/sd_flash_write-cause-nrf_fault_id_sd_assert
  // Workaround: write half page at a time.
#if NRF52832_XXAA
  // Write the page
  // Multiple attempts, if needed
  for (uint8_t attempt = 0; attempt < MAX_RETRY; ++attempt) {
    err = sd_flash_write((uint32_t*) dst, (uint32_t const *) src, len/4);

    if (sd_en) {
      err = wait_for_async_flash_op_completion(err); // Only async if soft device enabled
    }
    if (err == NRF_SUCCESS) { 
      break; 
    }
    if (err == NRF_ERROR_BUSY) {
      delay(1);
    }
  }
  VERIFY_STATUS(err, 0); // Return 0 if all retries fail

#else
  // Write first part of page
  // Multiple attempts, if needed
  for (uint8_t attempt = 0; attempt < MAX_RETRY; ++attempt) {
    err = sd_flash_write((uint32_t*) dst, (uint32_t const *) src, len/8);

    if (sd_en) { 
      err = wait_for_async_flash_op_completion(err); // Only async if soft device enabled
    }
    if (err == NRF_SUCCESS) {
      break;
    }
    if (err == NRF_ERROR_BUSY) {
      delay(1);
    }
  }
  VERIFY_STATUS(err, 0); // Return 0 if all retries fail

  // Write second part of page
  // Multiple attempts, if needed
  for (uint8_t attempt = 0; attempt < MAX_RETRY; ++attempt) {
    err = sd_flash_write((uint32_t*) (dst+ len/2), (uint32_t const *) (src + len/2), len/8);

    if (sd_en) {
      err = wait_for_async_flash_op_completion(err); // Only async if soft device enabled
    }
    if (err == NRF_SUCCESS) {
      break;
    }
    if (err == NRF_ERROR_BUSY) {
      delay(1);
    }
  }
  VERIFY_STATUS(err, 0); // Return 0 if all retries fail
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
