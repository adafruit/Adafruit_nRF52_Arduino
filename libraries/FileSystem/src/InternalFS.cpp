/**************************************************************************/
/*!
    @file     InternalFS.cpp
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

#include <Arduino.h>
#include "InternalFS.h"


#ifdef NRF52840_XXAA
#define LFS_FLASH_ADDR     0xED000
#else
#define LFS_FLASH_ADDR     0x6D000
#endif

#define LFS_FLASH_SIZE     (4*4096) //(7*4096)
#define LFS_BLOCK_SIZE     128

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
uint8_t ramdisk[LFS_FLASH_SIZE/LFS_BLOCK_SIZE][LFS_BLOCK_SIZE];

int _iflash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  (void) c;

  memcpy(buffer, ramdisk[block] + off, size);

  return 0;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int _iflash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
  (void) c;

  memcpy(ramdisk[block] + off, buffer, size);
  return 0;
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int _iflash_erase(const struct lfs_config *c, lfs_block_t block)
{
  (void) c;
  return 0;
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int _iflash_sync(const struct lfs_config *c)
{
  (void) c;
  return 0;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
LittleFS InternalFS;

LittleFS::LittleFS (void)
{
  _lfs_cfg = ((struct lfs_config) {
    .context = NULL,
    .read = _iflash_read,
    .prog = _iflash_prog,
    .erase = _iflash_erase,
    .sync = _iflash_sync,

    .read_size = LFS_BLOCK_SIZE,
    .prog_size = LFS_BLOCK_SIZE,
    .block_size = LFS_BLOCK_SIZE,
    .block_count = LFS_FLASH_SIZE/LFS_BLOCK_SIZE,
    .lookahead = 128
  } );
}

LittleFS::~LittleFS ()
{

}

bool LittleFS::begin (void)
{
  int err = lfs_mount(&_lfs, &_lfs_cfg);
  PRINT_INT(err);

  // reformat if we can't mount the filesystem
  // this should only happen on the first boot
  if (err) {
    err = lfs_format(&_lfs, &_lfs_cfg);
    PRINT_INT(err);
    err = lfs_mount(&_lfs, &_lfs_cfg);
    PRINT_INT(err);
  }
  PRINT_LOCATION();

  return true;
}

BluefuritLib::File LittleFS::open (char const *filename, uint8_t mode)
{
  BluefuritLib::File file(*this);

  return file;
}

bool LittleFS::exists (char const *filepath)
{

}

bool LittleFS::mkdir (char const *filepath)
{

}

bool LittleFS::remove (char const *filepath)
{

}

bool LittleFS::rmdir (char const *filepath)
{

}
