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

#ifndef FLASH_CACHE_H_
#define FLASH_CACHE_H_

#include <stdint.h>
#include <stdbool.h>

#define FLASH_CACHE_SIZE          4096        // must be a erasable page size
#define FLASH_CACHE_INVALID_ADDR  0xffffffff

typedef struct
{
    bool (*erase) (uint32_t addr);
    uint32_t (*program) (uint32_t dst, void const * src, uint32_t len);
    uint32_t (*read) (void* dst, uint32_t src, uint32_t len);
    bool (*verify) (uint32_t addr, void const * buf, uint32_t len);

    uint32_t cache_addr;
    uint8_t* cache_buf;
} flash_cache_t;

#ifdef __cplusplus
extern "C" {
#endif

int flash_cache_write (flash_cache_t* fc, uint32_t dst, void const *src, uint32_t count);
void flash_cache_flush (flash_cache_t* fc);
int flash_cache_read (flash_cache_t* fc, void* dst, uint32_t addr, uint32_t count);

#ifdef __cplusplus
 }
#endif

#endif /* FLASH_CACHE_H_ */
