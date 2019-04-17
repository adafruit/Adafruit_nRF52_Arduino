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

#ifdef NRF52840_XXAA

#include <Arduino.h>
#include <string.h>
#include "ExternalFS.h"
#include "flash/flash_qspi.h"

// up to 11 characters
#define VOLUME_LABEL    "BLUEFRUIT"
const char README_CONTENT[] =
                              "This drive can be used to add or remove files to the on-board \r\n"
                              "SPI flash IC on the Adafruit Bluefruit nRF52 Feather.\r\n"
                              "\r\n"
                              "For information on how to use the Bluefruit nRF52 Feather, see the following links:\r\n"
                              "\r\n"
                              "Adafruit Bluefruit nRF52 Feather Learning Guide:\r\n"
                              "https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide\r\n"
                              "\r\n"
                              "Adafruit Bluefruit nRF52 Feather Product Page:\r\n"
                              "- https://www.adafruit.com/product/4062\r\n";


//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
#if !CFG_DEBUG

#define VERIFY_FAT(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, NULL)
#define PRINT_FAT_ERR(_err)

#else

#define VERIFY_FAT(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, dbg_strerr_fat)
#define PRINT_FAT_ERR(_err)   VERIFY_MESS(_err, dbg_strerr_fat)

static const char* dbg_strerr_fat (int32_t err)
{
  switch ( err )
  {
    case FR_DISK_ERR:
      return "FR_DISK_ERR";
    case FR_INT_ERR:
      return "FR_INT_ERR";
    case FR_NOT_READY:
      return "FR_NOT_READY";
    case FR_NO_FILE:
      return "FR_NO_FILE";
    case FR_NO_PATH:
      return "FR_NO_PATH";
    case FR_INVALID_NAME:
      return "FR_INVALID_NAME";
    case FR_DENIED:
      return "FR_DENIED";
    case FR_EXIST:
      return "FR_EXIST";
    case FR_INVALID_OBJECT:
      return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED:
      return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE:
      return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED:
      return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM:
      return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED:
      return "FR_MKFS_ABORTED";
    case FR_TIMEOUT:
      return "FR_TIMEOUT";
    case FR_LOCKED:
      return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE:
      return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES:
      return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER:
      return "FR_INVALID_PARAMETER";
    default:
      return "Unlikely Error";
  }
}

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
FatFS ExternalFS;

FatFS::FatFS ()
{
  _fs = NULL;
  _begun = false;
}

FatFS::~FatFS ()
{
}

bool FatFS::begin (void)
{
  if ( _begun ) return true;
  _begun = true;

  if ( !_fs ) _fs = (FATFS*) rtos_malloc(sizeof(FATFS));
  VERIFY(_fs);

  // Mount file system
  uint32_t err = f_mount(_fs, "", 1);

  // If FileSystem is not available, format it
  if ( FR_NO_FILESYSTEM == err )
  {
    LOG_LV1("FATFS", "No file system, format it");
    this->format(false);

    return true;
  }
  else
  {
    VERIFY_FAT(err, false);
  }

  return true;
}

bool FatFS::format (bool eraseall)
{
  if ( eraseall )
  {
    LOG_LV2("EFLASH", "full erasing");
    VERIFY(flash_qspi_chiperase());
  }

  uint8_t workbuf[FF_MAX_SS];
  VERIFY_FAT(f_mkfs("", FM_FAT | FM_SFD, 4096, workbuf, FF_MAX_SS), false);
  VERIFY_FAT(f_setlabel(VOLUME_LABEL), false);
  VERIFY_FAT(f_mount(_fs, "", 1), false);

  // Always create README file when format
  File readme("README.TXT", FILE_WRITE, *this);
  readme.write(README_CONTENT);
  readme.close();


  return true;
}

File FatFS::_open_file (char const *filepath, uint8_t mode)
{
  BluefruitFS::File file(*this);

  int flags = (mode == FILE_READ) ? FA_READ :
              (mode == FILE_WRITE) ? (FA_READ | FA_WRITE | FA_OPEN_APPEND) : 0;

  if ( flags )
  {
    FIL* fhdl = (FIL*) rtos_malloc(sizeof(FIL));
    int rc = f_open(fhdl, filepath, flags);

    if ( rc )
    {
      rtos_free(fhdl);
      PRINT_FAT_ERR(rc);
    }
    else
    {
      file._hdl = fhdl;
      file._is_dir = false;

      file._path = (char*) rtos_malloc(strlen(filepath) + 1);
      strcpy(file._path, filepath);
    }
  }

  return file;
}

File FatFS::_open_dir (char const *filepath)
{
  BluefruitFS::File file(*this);

  DIR* fhdl = (DIR*) rtos_malloc(sizeof(DIR));
  int rc = f_opendir(fhdl, filepath);

  if ( rc )
  {
    rtos_free(fhdl);
    PRINT_FAT_ERR(rc);
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

File FatFS::open (char const *filepath, uint8_t mode)
{
  BluefruitFS::File file(*this);

  FILINFO fno;
  FRESULT rc = FR_OK;
  bool is_root =  (filepath[0] == '/' && filepath[1] == 0);

  // f_stat return FR_INVALID_NAME for root
  if ( !is_root ) rc = f_stat(filepath, &fno);

  if ( FR_OK == rc )
  {
    // file existed, open file or directory accordingly
    file = (is_root || (fno.fattrib & AM_DIR)) ? _open_dir(filepath) : _open_file(filepath, mode);
  }
  else if ( FR_NO_FILE == rc )
  {
    // file not existed, only proceed with FILE_WRITE mode
    if ( mode == FILE_WRITE ) file = _open_file(filepath, mode);
  }
  else
  {
    PRINT_STR(filepath);
    PRINT_FAT_ERR(rc);
  }

  return file;
}

bool FatFS::exists (char const *filepath)
{
  if ( filepath[0] == '/' && filepath[1] == 0 ) return true;    // root always exists

  FILINFO fno;
  return FR_OK == f_stat(filepath, &fno);
}

bool FatFS::mkdir (char const *filepath)
{
  VERIFY_FAT(f_mkdir(filepath), false);
  return true;
}

bool FatFS::remove (char const *filepath)
{
  VERIFY_FAT(f_unlink(filepath), false);
  return true;
}

bool FatFS::rmdir (char const *filepath)
{
  VERIFY_FAT(f_unlink(filepath), false);
  return true;
}

bool FatFS::rmdir_r (char const *filepath)
{
  // not supported yet
  return false;
}

size_t FatFS::_f_write (void* fhdl, uint8_t const *buf, size_t size)
{
  UINT byte_write = 0;
  VERIFY_FAT(f_write((FIL* ) fhdl, buf, size, &byte_write), byte_write);
  return byte_write;
}

int FatFS::_f_read (void* fhdl, void *buf, uint16_t nbyte)
{
  UINT byte_read = 0;
  VERIFY_FAT(f_read((FIL* ) fhdl, buf, nbyte, &byte_read), byte_read);
  return byte_read;
}

void FatFS::_f_flush (void* fhdl)
{
  VERIFY_FAT(f_sync((FIL* ) fhdl),);
}

void FatFS::_f_close (void* fhdl, bool is_dir)
{
  if ( is_dir )
  {
    VERIFY_FAT(f_closedir((DIR* ) fhdl),);
  }
  else
  {
    VERIFY_FAT(f_close((FIL* ) fhdl),);
  }
}

bool FatFS::_f_seek (void* fhdl, uint32_t pos)
{
  VERIFY_FAT(f_lseek((FIL* ) fhdl, pos), false);
  return true;
}

uint32_t FatFS::_f_position (void* fhdl)
{
  return f_tell((FIL* ) fhdl);
}

uint32_t FatFS::_f_size (void* fhdl)
{
  return f_size((FIL* ) fhdl);
}

File FatFS::_f_openNextFile (void* fhdl, char const* cwd, uint8_t mode)
{
  BluefruitFS::File file(*this);

  FILINFO fno;
  FRESULT rc;

  // f_readdir return FR_OK if there is no error. If there is no more entry,
  // fno.fname will be set to NUL string
  VERIFY_FAT(f_readdir((DIR * ) fhdl, &fno), file);

  if ( fno.fname[0] != 0 )
  {
    // string cat name with current folder
    char filepath[strlen(cwd) + 1 + strlen(fno.fname) + 1];

    strcpy(filepath, cwd);
    if ( !(cwd[0] == '/' && cwd[1] == 0) ) strcat(filepath, "/");    // only add '/' if cwd is not root
    strcat(filepath, fno.fname);

    file = (fno.fattrib & AM_DIR) ? _open_dir(filepath) : _open_file(filepath, mode);
  }

  return file;
}

void FatFS::_f_rewindDirectory (void* fhdl)
{
  VERIFY_FAT(f_rewinddir((DIR * ) fhdl),);
}

// for USB MSC to check and update fatfs sector cache
void FatFS::updateCache (uint32_t lba, void const* buffer, uint32_t bufsize)
{
  VERIFY(_fs,);
  if ( (lba <= _fs->winsect) && (_fs->winsect < (lba + bufsize / FF_MAX_SS)) )
  {
    memcpy(_fs->win, buffer + FF_MAX_SS * (_fs->winsect - lba), FF_MAX_SS);
  }
}

#endif
