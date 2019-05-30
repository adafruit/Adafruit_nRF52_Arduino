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

#include <Arduino.h>
#include <string.h>

#include "Adafruit_LittleFS.h"
#include "flash/flash_nrf5x.h"

using namespace Adafruit_LittleFS_Namespace;

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

#if CFG_DEBUG

#define VERIFY_LFS(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, dbg_strerr_lfs)
#define PRINT_LFS_ERR(_err)   VERIFY_MESS(_err, dbg_strerr_lfs)

const char* dbg_strerr_lfs (int32_t err)
{
  switch ( err )
  {
    case LFS_ERR_OK       : return "LFS_ERR_OK";
    case LFS_ERR_IO       : return "LFS_ERR_IO";
    case LFS_ERR_CORRUPT  : return "LFS_ERR_CORRUPT";
    case LFS_ERR_NOENT    : return "LFS_ERR_NOENT";
    case LFS_ERR_EXIST    : return "LFS_ERR_EXIST";
    case LFS_ERR_NOTDIR   : return "LFS_ERR_NOTDIR";
    case LFS_ERR_ISDIR    : return "LFS_ERR_ISDIR";
    case LFS_ERR_NOTEMPTY : return "LFS_ERR_NOTEMPTY";
    case LFS_ERR_BADF     : return "LFS_ERR_BADF";
    case LFS_ERR_INVAL    : return "LFS_ERR_INVAL";
    case LFS_ERR_NOSPC    : return "LFS_ERR_NOSPC";
    case LFS_ERR_NOMEM    : return "LFS_ERR_NOMEM";

    default:
      static char errcode[10];
      sprintf(errcode, "%d", err);
      return errcode;
  }

  return NULL;
}

#endif

//--------------------------------------------------------------------+
// flash API
//--------------------------------------------------------------------+

static inline uint32_t lba2addr(uint32_t block)
{
  return ((uint32_t) LFS_FLASH_ADDR) + block * LFS_BLOCK_SIZE;
}

static int _internal_flash_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  (void) c;

   uint32_t addr = lba2addr(block) + off;
  flash_nrf5x_read(buffer, addr, size);

   return 0;
}

 // Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
static int _internal_flash_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                         lfs_size_t size)
{
  (void) c;

   uint32_t addr = lba2addr(block) + off;
  flash_nrf5x_write(addr, buffer, size);

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

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+
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

Adafruit_LittleFS InternalFS(&_InternalFSConfig);

Adafruit_LittleFS::Adafruit_LittleFS (void)
  : Adafruit_LittleFS(NULL)
{

}

Adafruit_LittleFS::Adafruit_LittleFS (struct lfs_config* cfg)

{
  _lfs_cfg = cfg;

  _begun = false;
  _mounted = false;
}

Adafruit_LittleFS::~Adafruit_LittleFS ()
{

}

bool Adafruit_LittleFS::begin (struct lfs_config * cfg)
{
  if ( _begun ) return true;

  if (cfg) _lfs_cfg = cfg;
  if (!_lfs_cfg) return false;

  _begun = true;

  int err = lfs_mount(&_lfs, _lfs_cfg);

  // reformat if we can't mount the filesystem
  if ( LFS_ERR_CORRUPT == err )
  {
    LOG_LV1("IFLASH", "Format internal file system");
    this->format(false);
  } else {
    _mounted = true;
  }

  return true;
}

void Adafruit_LittleFS::_flash_erase_all()
{
  for ( uint32_t addr = LFS_FLASH_ADDR; addr < LFS_FLASH_ADDR + LFS_FLASH_TOTAL_SIZE; addr += FLASH_NRF52_PAGE_SIZE )
  {
    flash_nrf5x_erase(addr);
  }
}


bool Adafruit_LittleFS::format (bool eraseall)
{
  if ( eraseall ) {
    _flash_erase_all();
  }
  if(_mounted) {
    VERIFY_LFS(lfs_unmount(&_lfs), false);
  }
  VERIFY_LFS(lfs_format(&_lfs, _lfs_cfg), false);
  VERIFY_LFS(lfs_mount(&_lfs, _lfs_cfg), false);

  _mounted = true;

  return true;
}

Adafruit_LittleFS_Namespace::File Adafruit_LittleFS::open (char const *filepath, uint8_t mode)
{
  return Adafruit_LittleFS_Namespace::File(filepath, mode, *this);
}

bool Adafruit_LittleFS::exists (char const *filepath)
{
  struct lfs_info info;
  return 0 == lfs_stat(&_lfs, filepath, &info);
}

bool Adafruit_LittleFS::mkdir (char const *filepath)
{
  const char* slash = filepath;
  if ( slash[0] == '/' ) slash++;    // skip root '/'

  while ( NULL != (slash = strchr(slash, '/')) )
  {
    char parent[slash - filepath + 1] = { 0 };
    memcpy(parent, filepath, slash - filepath);

    // make intermediate parent
    int rc = lfs_mkdir(&_lfs, parent);
    if ( rc != LFS_ERR_OK && rc != LFS_ERR_EXIST )
    {
      PRINT_LFS_ERR(rc);
      return false;
    }

    slash++;
  }
  
  int rc = lfs_mkdir(&_lfs, filepath);
  if ( rc != LFS_ERR_OK && rc != LFS_ERR_EXIST )
  {
    PRINT_LFS_ERR(rc);
    return false;
  }

  return true;
}

bool Adafruit_LittleFS::remove (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath), false);
  return true;
}

bool Adafruit_LittleFS::rmdir (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}

bool Adafruit_LittleFS::rmdir_r (char const *filepath)
{
  /* adafruit: lfs is modified to remove non-empty folder,
   According to below issue, comment these 2 line won't corrupt filesystem
   https://github.com/ARMmbed/littlefs/issues/43 */
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}
