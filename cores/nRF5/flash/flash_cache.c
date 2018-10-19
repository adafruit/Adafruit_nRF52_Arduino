/**************************************************************************/
/*!
    @file     flash_cache.c
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

#include "flash_cache.h"

#include "common_func.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
static inline uint32_t page_addr_of (uint32_t addr)
{
  return addr & ~(FLASH_CACHE_SIZE - 1);
}

static inline uint32_t page_offset_of (uint32_t addr)
{
  return addr & (FLASH_CACHE_SIZE - 1);
}

uint32_t flash_cache_write (flash_cache_t* fc, uint32_t dst, void const * src, uint32_t len)
{
  uint8_t const * src8 = (uint8_t const *) src;
  uint32_t remain = len;

  // Program blocks up to page boundary each loop
  while ( remain )
  {
    uint32_t const page_addr = page_addr_of(dst);
    uint32_t const offset = page_offset_of(dst);

    uint32_t wr_bytes = FLASH_CACHE_SIZE - offset;
    wr_bytes = min32(remain, wr_bytes);

    // Page changes, flush old and update new cache
    if ( page_addr != fc->cache_addr )
    {
      flash_cache_flush(fc);
      fc->cache_addr = page_addr;

      // read existing flash to cache except those we are writing
      if ( offset )
      {
        fc->read(fc->cache_buf, page_addr, offset);
      }

      uint32_t const last_byte = offset + wr_bytes;
      if ( last_byte < FLASH_CACHE_SIZE )
      {
        fc->read(fc->cache_buf + last_byte, page_addr + last_byte, FLASH_CACHE_SIZE - last_byte);
      }
    }

    memcpy(fc->cache_buf + offset, src8, wr_bytes);

    // adjust for next run
    src8 += wr_bytes;
    remain -= wr_bytes;
  }

  return len - remain;
}

void flash_cache_flush (flash_cache_t* fc)
{
  if ( fc->cache_addr == FLASH_CACHE_INVALID_ADDR ) return;

  // skip erase & program if verify() exists, and memory matches
  if ( !(fc->verify && fc->verify(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE)) )
  {
    fc->erase(fc->cache_addr);
    fc->program(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE);
  }

  fc->cache_addr = FLASH_CACHE_INVALID_ADDR;
}

void flash_cache_read (flash_cache_t* fc, void* dst, uint32_t addr, uint32_t count)
{
  // overwrite with cache value if available
  if ( (fc->cache_addr != FLASH_CACHE_INVALID_ADDR) &&
       !(addr < fc->cache_addr && addr + count <= fc->cache_addr) &&
       !(addr >= fc->cache_addr + FLASH_CACHE_SIZE) )
  {
    int dst_off = fc->cache_addr - addr;
    int src_off = 0;

    if ( dst_off < 0 )
    {
      src_off = -dst_off;
      dst_off = 0;
    }

    int cache_bytes = minof(FLASH_CACHE_SIZE-src_off, count - dst_off);

    // start to cached
    if ( dst_off ) fc->read(dst, addr, dst_off);

    // cached
    memcpy(dst + dst_off, fc->cache_buf + src_off, cache_bytes);

    // cached to end
    int copied = dst_off + cache_bytes;
    if ( copied < count ) fc->read(dst + copied, addr + copied, count - copied);
  }
  else
  {
    memcpy(dst, (void*) addr, count);
  }
}
