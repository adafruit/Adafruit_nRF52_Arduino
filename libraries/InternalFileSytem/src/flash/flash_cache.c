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

#include <string.h>
#include "flash_cache.h"
#include "common_func.h"
#include "variant.h"
#include "wiring_digital.h"

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

  // Program up to page boundary each loop
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

      // read a whole page from flash
      fc->read(fc->cache_buf, page_addr, FLASH_CACHE_SIZE);
    }

    memcpy(fc->cache_buf + offset, src8, wr_bytes);

    // adjust for next run
    src8 += wr_bytes;
    remain -= wr_bytes;
    dst += wr_bytes;
  }

  return len - remain;
}

void flash_cache_flush (flash_cache_t* fc)
{
  if ( fc->cache_addr == FLASH_CACHE_INVALID_ADDR ) return;

  // skip erase & program if verify() exists, and memory matches
  if ( !(fc->verify && fc->verify(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE)) )
  {
    // indicator
    ledOn(LED_BUILTIN);

    fc->erase(fc->cache_addr);
    fc->program(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE);

    ledOff(LED_BUILTIN);
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
    fc->read(dst, addr, count);
  }
}
