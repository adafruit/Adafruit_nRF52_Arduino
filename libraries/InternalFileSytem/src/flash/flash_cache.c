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

int flash_cache_write (flash_cache_t* fc, uint32_t dst, void const * src, uint32_t len)
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
    // indicator TODO allow to disable flash indicator
    ledOn(LED_BUILTIN);

    fc->erase(fc->cache_addr);
    fc->program(fc->cache_addr, fc->cache_buf, FLASH_CACHE_SIZE);

    ledOff(LED_BUILTIN);
  }

  fc->cache_addr = FLASH_CACHE_INVALID_ADDR;
}

int flash_cache_read (flash_cache_t* fc, void* dst, uint32_t addr, uint32_t count)
{
  // there is no check for overflow / wraparound for dst + count, addr + count.
  // this might be a useful thing to add for at least debug builds.

  // overwrite with cache value if available
  if ( (fc->cache_addr != FLASH_CACHE_INVALID_ADDR) &&               // cache is not valid
       !(addr < fc->cache_addr && addr + count <= fc->cache_addr) && // starts before, ends before cache area
       !(addr >= fc->cache_addr + FLASH_CACHE_SIZE) )                // starts after end of cache area
  {
    // This block is entered only when the read overlaps the cache area by at least one byte.
    // If the read starts before the cache area, it's further guaranteed
    //    that count is large enough to cause the read to enter
    //    the cache area by at least 1 byte.
    uint32_t dst_off = 0;
    uint32_t src_off = 0;
    if (addr < fc->cache_addr)
    {
      dst_off = fc->cache_addr - addr;
      // Read the bytes prior to the cache address
      fc->read(dst, addr, dst_off);
    }
    else
    {
      src_off = addr - fc->cache_addr;      
    }
    
    // Thus, after the above code block executes:
    // *** AT MOST ***, only one of src_off and dst_off are non-zero;
    // (Both may be zero when the read starts at the start of the cache area.)
    // dst_off corresponds to the number of bytes already read from PRIOR to the cache area.
    // src_off corresponds to the byte offset to start reading at, from WITHIN the cache area.

    // How many bytes to memcpy from flash area?
    // Remember that, AT MOST, one of src_off and dst_off are non-zero.
    // If src_off is non-zero, then dst_off is zero, representing that the
    //   read starts inside the cache.  In this case:
    //     PARAM1 := FLASH_CACHE_SIZE - src_off == maximum possible bytes to read from cache
    //     PARAM2 := count
    //   Thus, taking the minimum of the two gives the number of bytes to read from cache,
    //     in the range [ 1 .. FLASH_CACHE_SIZE-src_off ].
    // Else if dst_off is non-zero, then src_off is zero, representing that the
    //   read started prior to the cache area.  In this case:
    //     PARAM1 := FLASH_CACHE_SIZE == full size of the cache
    //     PARAM2 := count - dst_off == total bytes requested, minus the count of those already read
    //   Because the original request is guaranteed to overlap the cache, the range for
    //     PARAM2 is ensured to be [ 1 .. count-1 ].
    //   Thus, taking the minimum of the two gives the number of bytes to read from cache,
    //     in the range [ 1 .. FLASH_CACHE_SIZE ]
    // Else both src_off and dst_off are zero, representing that the read is starting
    //   exactly aligned to the cache.
    //     PARAM1 := FLASH_CACHE_SIZE
    //     PARAM2 := count
    //   Thus, taking the minimum of the two gives the number of bytes to read from cache,
    //     in the range [ 1 .. FLASH_CACHE_SIZE ]
    // 
    // Therefore, in all cases, there is assurance that cache_bytes
    // will be in the final range [1..FLASH_CACHE_SIZE].
    uint32_t cache_bytes = minof(FLASH_CACHE_SIZE-src_off, count - dst_off);

    // Use memcpy to read cached data into the buffer
    // If src_off is non-zero, then dst_off is zero, representing that the
    //   read starts inside the cache.  In this case:
    //     PARAM1 := dst
    //     PARAM2 := fc->cache_buf + src_off
    //     PARAM3 := cache_bytes
    //   Thus, all works as expected when starting in the midst of the cache.
    // Else if dst_off is non-zero, then src_off is zero, representing that the
    //   read started prior to the cache.  In this case:
    //     PARAM1 := dst + dst_off == destination offset by number of bytes already read
    //     PARAM2 := fc->cache_buf
    //     PARAM3 := cache_bytes
    //   Thus, all works as expected when starting prior to the cache.
    // Else both src_off and dst_off are zero, representing that the read is starting
    //   exactly aligned to the cache.
    //     PARAM1 := dst
    //     PARAM2 := fc->cache_buf
    //     PARAM3 := cache_bytes
    //   Thus, all works as expected when starting exactly at the cache boundary
    // 
    // Therefore, in all cases, there is assurance that cache_bytes
    // will be in the final range [1..FLASH_CACHE_SIZE].
    memcpy(dst + dst_off, fc->cache_buf + src_off, cache_bytes);

    // Read any final bytes from flash
    // As noted above, dst_off represents the count of bytes read prior to the cache
    // while cache_bytes represents the count of bytes read from the cache;
    // This code block is guaranteed to overlap the cache area by at least one byte.
    // Thus, copied will correspond to the total bytes already copied,
    // and is guaranteed to be in the range [ 1 .. count ].
    uint32_t copied = dst_off + cache_bytes;
    
    // 
    if ( copied < count )
    {
      fc->read(dst + copied, addr + copied, count - copied);
    }
  }
  else
  {
    // not using the cache, so just forward to read from flash
    fc->read(dst, addr, count);
  }

  return (int) count;
}
