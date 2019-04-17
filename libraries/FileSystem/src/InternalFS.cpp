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
#include "InternalFS.h"
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

#if !CFG_DEBUG

#define VERIFY_LFS(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, NULL)
#define PRINT_LFS_ERR(_err)

#else

#define VERIFY_LFS(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, dbg_strerr_lfs)
#define PRINT_LFS_ERR(_err)   VERIFY_MESS(_err, dbg_strerr_lfs)

static const char* dbg_strerr_lfs (int32_t err)
{
  switch ( err )
  {
    case LFS_ERR_OK:
      return "LFS_ERR_OK";

    case LFS_ERR_IO:
      return "LFS_ERR_IO";

    case LFS_ERR_CORRUPT:
      return "LFS_ERR_CORRUPT";

    case LFS_ERR_NOENT:
      return "LFS_ERR_NOENT";

    case LFS_ERR_EXIST:
      return "LFS_ERR_EXIST";

    case LFS_ERR_NOTDIR:
      return "LFS_ERR_NOTDIR";

    case LFS_ERR_ISDIR:
      return "LFS_ERR_ISDIR";

    case LFS_ERR_NOTEMPTY:
      return "LFS_ERR_NOTEMPTY";

    case LFS_ERR_BADF:
      return "LFS_ERR_BADF";

    case LFS_ERR_INVAL:
      return "LFS_ERR_INVAL";

    case LFS_ERR_NOSPC:
      return "LFS_ERR_NOSPC";

    case LFS_ERR_NOMEM:
      return "LFS_ERR_NOMEM";

    default:
    {
      static char errcode[10];
      sprintf(errcode, "%d", err);
      return errcode;
    }
  }
}

#endif

static inline uint32_t lba2addr(uint32_t block)
{
  return ((uint32_t) LFS_FLASH_ADDR) + block * LFS_BLOCK_SIZE;
}

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+
LittleFS InternalFS;

LittleFS::LittleFS (void) :
  LittleFS( LFS_BLOCK_SIZE, LFS_BLOCK_SIZE, LFS_BLOCK_SIZE, LFS_FLASH_TOTAL_SIZE / LFS_BLOCK_SIZE, 128)
{
}

LittleFS::LittleFS (lfs_size_t read_size, lfs_size_t prog_size, lfs_size_t block_size, lfs_size_t block_count, lfs_size_t lookahead)
{
  varclr(&_lfs_cfg);
  _lfs_cfg.context = this;
  _lfs_cfg.read = _iflash_read;
  _lfs_cfg.prog = _iflash_prog;
  _lfs_cfg.erase = _iflash_erase;
  _lfs_cfg.sync = _iflash_sync;

  _lfs_cfg.read_size = read_size;
  _lfs_cfg.prog_size = prog_size;
  _lfs_cfg.block_size = block_size;
  _lfs_cfg.block_count = block_count;
  _lfs_cfg.lookahead = lookahead;

  _begun = false;
  _mounted = false;
}

LittleFS::~LittleFS ()
{

}

bool LittleFS::begin (void)
{
  if ( _begun ) return true;
  _begun = true;

  int err = lfs_mount(&_lfs, &_lfs_cfg);

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

bool LittleFS::format (bool eraseall)
{
  if ( eraseall ) {
    _flash_erase_all();
  }
  if(_mounted) {
    VERIFY_LFS(lfs_unmount(&_lfs), false);
  }
  VERIFY_LFS(lfs_format(&_lfs, &_lfs_cfg), false);
  VERIFY_LFS(lfs_mount(&_lfs, &_lfs_cfg), false);

  _mounted = true;

  return true;
}

BluefruitFS::File LittleFS::_open_file (char const *filepath, uint8_t mode)
{
  BluefruitFS::File file(*this);

  int flags = (mode == FILE_READ) ? LFS_O_RDONLY :
              (mode == FILE_WRITE) ? (LFS_O_RDWR | LFS_O_CREAT) : 0;

  if ( flags )
  {
    lfs_file_t* fhdl = (lfs_file_t*) rtos_malloc(sizeof(lfs_file_t));
    VERIFY(fhdl, file);

    int rc = lfs_file_open(&_lfs, fhdl, filepath, flags);

    if ( rc )
    {
      rtos_free(fhdl);
      PRINT_LFS_ERR(rc);
    }
    else
    {
      // move to end of file
      if ( mode == FILE_WRITE ) lfs_file_seek(&_lfs, fhdl, 0, LFS_SEEK_END);

      file._hdl = fhdl;
      file._is_dir = false;

      file._path = (char*) rtos_malloc(strlen(filepath) + 1);
      strcpy(file._path, filepath);
    }
  }

  return file;
}

BluefruitFS::File LittleFS::_open_dir (char const *filepath)
{
  BluefruitFS::File file(*this);

  lfs_dir_t* fhdl = (lfs_dir_t*) rtos_malloc(sizeof(lfs_dir_t));
  int rc = lfs_dir_open(&_lfs, fhdl, filepath);

  if ( rc )
  {
    rtos_free(fhdl);
    PRINT_LFS_ERR(rc);
  }
  else
  {
    file._hdl = fhdl;
    file._is_dir = true;

    file._path = (char*) rtos_malloc(strlen(filepath) + 1);
    strcpy(file._path, filepath);
  }

  return file;
}

BluefruitFS::File LittleFS::open (char const *filepath, uint8_t mode)
{
  BluefruitFS::File file(*this);
  struct lfs_info info;

  int rc = lfs_stat(&_lfs, filepath, &info);
  if ( LFS_ERR_OK == rc )
  {
    // file existed, open file or directory accordingly
    file = (info.type == LFS_TYPE_REG) ? _open_file(filepath, mode) : _open_dir(filepath);
  }
  else if ( LFS_ERR_NOENT == rc )
  {
    // file not existed, only proceed with FILE_WRITE mode
    if ( mode == FILE_WRITE ) file = _open_file(filepath, mode);
  }
  else
  {
    PRINT_LFS_ERR(rc);
  }

  return file;
}

bool LittleFS::exists (char const *filepath)
{
  struct lfs_info info;
  return 0 == lfs_stat(&_lfs, filepath, &info);
}

bool LittleFS::mkdir (char const *filepath)
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

bool LittleFS::remove (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath), false);
  return true;
}

bool LittleFS::rmdir (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}

bool LittleFS::rmdir_r (char const *filepath)
{
  /* adafruit: lfs is modified to remove non-empty folder,
   According to below issue, comment these 2 line won't corrupt filesystem
   https://github.com/ARMmbed/littlefs/issues/43 */
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}

//--------------------------------------------------------------------+
// FILE API
//--------------------------------------------------------------------+
size_t LittleFS::_f_write (void* fhdl, uint8_t const *buf, size_t size)
{
  lfs_ssize_t wrcount = lfs_file_write(&_lfs, (lfs_file_t*) fhdl, buf, size);
  VERIFY(wrcount > 0, 0);
  return wrcount;
}

void LittleFS::_f_flush (void* fhdl)
{
  VERIFY_LFS(lfs_file_sync(&_lfs, (lfs_file_t* ) fhdl),);
}

int LittleFS::_f_read (void* fhdl, void *buf, uint16_t nbyte)
{
  return lfs_file_read(&_lfs, (lfs_file_t*) fhdl, buf, nbyte);
}

bool LittleFS::_f_seek (void* fhdl, uint32_t pos)
{
  return lfs_file_seek(&_lfs, (lfs_file_t*) fhdl, pos, LFS_SEEK_SET) >= 0;
}

uint32_t LittleFS::_f_position (void* fhdl)
{
  return lfs_file_tell(&_lfs, (lfs_file_t*) fhdl);
}

uint32_t LittleFS::_f_size (void* fhdl)
{
  return lfs_file_size(&_lfs, (lfs_file_t*) fhdl);
}

void LittleFS::_f_close (void* fhdl, bool is_dir)
{
  if ( is_dir )
  {
    lfs_dir_close(&_lfs, (lfs_dir_t *) fhdl);
  }
  else
  {
    lfs_file_close(&_lfs, (lfs_file_t*) fhdl);
  }
}

File LittleFS::_f_openNextFile (void* fhdl, char const* cwd, uint8_t mode)
{
  BluefruitFS::File file(*this);
  struct lfs_info info;

  int rc;

  // lfs_dir_read return 0 when reaching end of directory, 1 if found an entry
  // skip "." and ".." entries
  do
  {
    rc = lfs_dir_read(&_lfs, (lfs_dir_t *) fhdl, &info);
  } while ( rc == 1 && (!strcmp(".", info.name) || !strcmp("..", info.name)) );

  if ( rc == 1 )
  {
    // string cat name with current folder
    char filepath[strlen(cwd) + 1 + strlen(info.name) + 1];

    strcpy(filepath, cwd);
    if ( !(cwd[0] == '/' && cwd[1] == 0) ) strcat(filepath, "/");    // only add '/' if cwd is not root
    strcat(filepath, info.name);

    file = (info.type == LFS_TYPE_REG) ? _open_file(filepath, mode) : _open_dir(filepath);
  }
  else if ( rc < 0 )
  {
    PRINT_LFS_ERR(rc);
  }

  return file;
}

void LittleFS::_f_rewindDirectory (void* fhdl)
{
  VERIFY_LFS(lfs_dir_rewind(&_lfs, (lfs_dir_t* ) fhdl),);
}


int LittleFS::_flash_read (lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  uint32_t addr = lba2addr(block) + off;
  flash_nrf5x_read(buffer, addr, size);

  return 0;
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int LittleFS::_flash_prog (lfs_block_t block, lfs_off_t off, const void *buffer,
                         lfs_size_t size)
{
  uint32_t addr = lba2addr(block) + off;
  flash_nrf5x_write(addr, buffer, size);

  return 0;
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int LittleFS::_flash_erase (lfs_block_t block)
{
  uint32_t addr = lba2addr(block);

  // implement as write 0xff to whole block address
  for(int i=0; i <LFS_BLOCK_SIZE; i++)
  {
    flash_nrf5x_write8(addr + i, 0xFF);
  }

  // flash_nrf5x_flush();

  return 0;
}

void LittleFS::_flash_erase_all()
{
  for ( uint32_t addr = LFS_FLASH_ADDR; addr < LFS_FLASH_ADDR + LFS_FLASH_TOTAL_SIZE; addr += FLASH_NRF52_PAGE_SIZE )
  {
    flash_nrf5x_erase(addr);
  }
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int LittleFS::_flash_sync ()
{
  flash_nrf5x_flush();
  return 0;
}

int LittleFS::_iflash_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  return ((LittleFS *)c->context)->_flash_read(block, off, buffer, size);
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int LittleFS::_iflash_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                         lfs_size_t size)
{
  return ((LittleFS *)c->context)->_flash_prog(block, off, buffer, size);
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int LittleFS::_iflash_erase (const struct lfs_config *c, lfs_block_t block)
{
  return ((LittleFS *)c->context)->_flash_erase(block);
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int LittleFS::_iflash_sync (const struct lfs_config *c)
{
  return ((LittleFS *)c->context)->_flash_sync();
}
