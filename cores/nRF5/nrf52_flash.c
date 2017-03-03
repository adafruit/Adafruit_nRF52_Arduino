/**************************************************************************/
/*!
    @file     nrf52_flash.c
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include "Arduino.h"
#include "nrf52_flash.h"

static SemaphoreHandle_t _evt_sem = NULL;
static volatile uint32_t _op_result;

void hal_flash_event_cb(uint32_t event)
{
  _op_result = event;
  xSemaphoreGive(_evt_sem);
}

int nrf52_flash_init(void)
{
  _evt_sem = xSemaphoreCreateCounting(10, 0);
  return (_evt_sem != NULL) ? 0 : 1;
}

int nrf52_flash_erase_sector(uint32_t sector_address)
{
  uint32_t err;

//  cprintf("Flash Erase at 0x08%lX \n", sector_address);

  // delay and try again if busy
  while ( NRF_ERROR_BUSY == (err = sd_flash_page_erase(sector_address/NRF52K_FLASH_SECTOR_SZ)) )
  {
    delay(1);
  }
  VERIFY_STATUS(err);

  xSemaphoreTake(_evt_sem, portMAX_DELAY);
  return (_op_result == NRF_EVT_FLASH_OPERATION_SUCCESS ) ? 0 : (-1);
}

static int write_and_wait(uint32_t addr, uint32_t const * const src, uint32_t size)
{
  uint32_t err;

//  cprintf("Flash Write at 0x08%lX with %d bytes\n", addr, size);

  // delay and try again if busy
  while ( NRF_ERROR_BUSY == (err = sd_flash_write( (uint32_t*) addr, src, size)) )
  {
    delay(1);
  }
  VERIFY_STATUS(err);

  xSemaphoreTake(_evt_sem, portMAX_DELAY);
  return (_op_result == NRF_EVT_FLASH_OPERATION_SUCCESS ) ? 0 : (-1);
}

int nrf52_flash_write(uint32_t address, const void *src, uint32_t num_bytes)
{

  /* SD Flash requires
   * 1. src data must be word aligned
   * 2. address must be word aligned
   * 2. each write is 4 bytes
   *
   * This cause issue with string literal. Solution is break up to 3 sequence write
   * 1. First is required if address not started at word aligned
   * 2. Malloc aligned data for src if needed
   * 3. Trailing bytes
   */

  /*------------- non-aligned Address -------------*/
  // Address is not aligned
  uint32_t miss = address & 0x03;

  if ( miss )
  {
    /* Address is not aligned. Combine odd bytes data + existed data
     * then update and write back
     */

    uint32_t cnt = 4 - miss;
    uint32_t val;

    memcpy( &val, (uint8_t*) align4(address), 4);
    memcpy( ((uint8_t*)&val)+miss, src, cnt);

    VERIFY_STATUS( write_and_wait( align4(address), &val, 1) );

    address   += cnt;
    src       += cnt;
    num_bytes -= cnt;
  }

  /*------------- Main write -------------*/
  uint32_t count = align4(num_bytes);
  uint32_t* tempbuf;

  // src is not word aligned
  miss = ((uint32_t)src) & 0x03;
  if (miss)
  {
    // Malloc and copy multiple of 4 bytes from src to write to flash
    tempbuf = (uint32_t*) rtos_malloc( count );
    VERIFY( tempbuf, NRF_ERROR_NO_MEM);

    memcpy(tempbuf, src, count);
  }else
  {
    tempbuf = (uint32_t*) src;
  }

  VERIFY_STATUS( write_and_wait(address, tempbuf, count / 4) );

  if (miss)
  {
    free(tempbuf);
  }

  address   += count;
  src       += count;
  num_bytes -= count;

  /*------------- Trailing Bytes -------------*/
  if ( num_bytes )
  {
    uint32_t val;

    memcpy( &val, (uint8_t*) address, 4);
    memcpy(&val, src, num_bytes);

    VERIFY_STATUS( write_and_wait(address, &val, 1) );
  }

  return 0;
}

