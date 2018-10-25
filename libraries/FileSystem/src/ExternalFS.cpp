/**************************************************************************/
/*!
    @file     ExternalFS.cpp
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

#ifdef NRF52840_XXAA

#include <Arduino.h>
#include <string.h>
#include "ExternalFS.h"

// up to 11 characters
#define VOLUME_LABEL    "BLUEFRUIT"

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

    uint8_t workbuf[FF_MAX_SS];
    VERIFY_FAT(f_mkfs("", FM_FAT | FM_SFD, 4096, workbuf, FF_MAX_SS), false);

    VERIFY_FAT(f_setlabel(VOLUME_LABEL), false);

//    FatFile readme;
//    readme.open("readme.txt", FAT_FILE_WRITE | FAT_FILE_CREATE_ALWAYS);
//    readme.write(README_CONTENT);
//    readme.close();

    return true;
  }
  else
  {
    VERIFY_FAT(err, false);
  }

  return true;
}

File FatFS::_open_file (char const *filepath, uint8_t mode)
{
  BluefuritLib::File file(*this);

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
  BluefuritLib::File file(*this);

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
  BluefuritLib::File file(*this);

  FILINFO fno;
  FRESULT rc = f_stat(filepath, &fno);

  if ( FR_OK == rc )
  {
    // file existed, open file or directory accordingly
    file = (fno.fattrib & AM_DIR) ? _open_file(filepath, mode) : _open_dir(filepath);
  }
  else if ( FR_NO_FILE == rc )
  {
    // file not existed, only proceed with FILE_WRITE mode
    if ( mode == FILE_WRITE ) file = _open_file(filepath, mode);
  }
  else
  {
    PRINT_FAT_ERR(rc);
  }

  return file;
}

bool FatFS::exists (char const *filepath)
{
}
bool FatFS::mkdir (char const *filepath)
{
}
bool FatFS::remove (char const *filepath)
{
}
bool FatFS::rmdir (char const *filepath)
{
}
bool FatFS::rmdir_r (char const *filepath)
{
}

size_t FatFS::_f_write (void* fhdl, uint8_t const *buf, size_t size)
{
}
int FatFS::_f_read (void* fhdl, void *buf, uint16_t nbyte)
{
}
void FatFS::_f_flush (void* fhdl)
{
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
}

uint32_t FatFS::_f_position (void* fhdl)
{
}

uint32_t FatFS::_f_size (void* fhdl)
{
  return f_size((FIL* ) fhdl);
}

File FatFS::_f_openNextFile (void* fhdl, char const* cwd, uint8_t mode)
{
  BluefuritLib::File file(*this);

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

    file = (fno.fattrib & AM_DIR) ? _open_file(filepath, mode) : _open_dir(filepath);
  }

  return file;
}

void FatFS::_f_rewindDirectory (void* fhdl)
{
}



#endif
