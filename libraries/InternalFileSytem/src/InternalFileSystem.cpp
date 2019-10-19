/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 hathach for Adafruit Industries
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

#include "InternalFileSystem.h"
#include "flash/flash_nrf5x.h"

#ifdef NRF52840_XXAA
#define LFS_FLASH_ADDR        0xED000
#else
#define LFS_FLASH_ADDR        0x6D000
#endif

#define LFS_FLASH_TOTAL_SIZE  (7*FLASH_NRF52_PAGE_SIZE)
#define LFS_BLOCK_SIZE        128

//--------------------------------------------------------------------+
// LFS Disk IO
//--------------------------------------------------------------------+

static inline uint32_t lba2addr(uint32_t block)
{
  return ((uint32_t) LFS_FLASH_ADDR) + block * LFS_BLOCK_SIZE;
}

static int _internal_flash_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  (void) c;

  uint32_t addr = lba2addr(block) + off;
  VERIFY( flash_nrf5x_read(buffer, addr, size) > 0, -1);

  return 0;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int _internal_flash_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
  (void) c;

  uint32_t addr = lba2addr(block) + off;
  VERIFY( flash_nrf5x_write(addr, buffer, size), -1)

  return 0;
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int _internal_flash_erase (const struct lfs_config *c, lfs_block_t block)
{
  (void) c;

  uint32_t addr = lba2addr(block);

  // implement as write 0xff to whole block address
  for(int i=0; i <LFS_BLOCK_SIZE; i++)
  {
    flash_nrf5x_write8(addr + i, 0xFF);
  }

  // flash_nrf5x_flush();

  return 0;
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
static int _internal_flash_sync (const struct lfs_config *c)
{
  (void) c;
  flash_nrf5x_flush();
  return 0;
}


static struct lfs_config _InternalFSConfig =
{
  .context = NULL,

  .read = _internal_flash_read,
  .prog = _internal_flash_prog,
  .erase = _internal_flash_erase,
  .sync = _internal_flash_sync,

  .read_size = LFS_BLOCK_SIZE,
  .prog_size = LFS_BLOCK_SIZE,
  .block_size = LFS_BLOCK_SIZE,
  .block_count = LFS_FLASH_TOTAL_SIZE / LFS_BLOCK_SIZE,
  .lookahead = 128,

  .read_buffer = NULL,
  .prog_buffer = NULL,
  .lookahead_buffer = NULL,
  .file_buffer = NULL
};

InternalFileSystem InternalFS;

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

InternalFileSystem::InternalFileSystem(void)
  : Adafruit_LittleFS(&_InternalFSConfig)
{

}

bool InternalFileSystem::begin(void)
{
  // failed to mount, erase all sector then format and mount again
  if ( !Adafruit_LittleFS::begin() )
  {
    // Erase all sectors of internal flash region for Filesystem.
    for ( uint32_t addr = LFS_FLASH_ADDR; addr < LFS_FLASH_ADDR + LFS_FLASH_TOTAL_SIZE; addr += FLASH_NRF52_PAGE_SIZE )
    {
      VERIFY( flash_nrf5x_erase(addr) );
    }

    // lfs format
    this->format();

    // mount again if still failed, give up
    if ( !Adafruit_LittleFS::begin() ) return false;
  }

  return true;
}
