/**************************************************************************/
/*!
    @file     NewtNffs.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include "NewtNffs.h"

#include "syscfg/syscfg.h"
#include "hal/hal_flash.h"

#define NFFS_AREA_MAX    8

NewtNffs Nffs;

NewtNffs::NewtNffs(void)
{
  errnum       = FS_EOK;
  _initialized = false;
}

bool NewtNffs::begin(void)
{
  // Follow code in nffs_pkg_init()
  if (_initialized) return true;
  _initialized = true;

  // Init flash
  nrf52_flash_init();

  /* Initialize nffs's internal state. */
  errnum = nffs_init();
  VERIFY_STATUS( errnum, false );

  /* Convert the set of flash blocks we intend to use for nffs into an array
   * of nffs area descriptors.
   */
  struct nffs_area_desc descs[NFFS_AREA_MAX + 1];
  int cnt = NFFS_AREA_MAX;

  errnum = nffs_misc_desc_from_flash_area( MYNEWT_VAL(NFFS_FLASH_AREA), &cnt, descs);
  VERIFY_STATUS( errnum, false );

  /* Attempt to restore an existing nffs file system from flash. */
  errnum = nffs_detect(descs);

  // If not formatted, format it
  if (FS_ECORRUPT == errnum)
  {
    PRINT_MESS("No FS detected, format");
    errnum = nffs_format(descs);
    VERIFY_STATUS( errnum, false );
  }

  return true;
}

bool NewtNffs::format(void)
{
  struct nffs_area_desc descs[NFFS_AREA_MAX + 1];
  int cnt = NFFS_AREA_MAX;

  errnum = nffs_misc_desc_from_flash_area( MYNEWT_VAL(NFFS_FLASH_AREA), &cnt, descs);
  VERIFY_STATUS( errnum, false );

  errnum = nffs_format(descs);
  VERIFY_STATUS( errnum, false );

  return true;
}

bool NewtNffs::mkdir(const char* path)
{
  errnum = fs_mkdir(path);
  return (errnum == FS_EOK);
}

/**
 * Make a directory (and its parents if not existed) a.k.a mkdir -p
 * @param path Absolute path of directory, must start with '/'
 * @return
 */
bool NewtNffs::mkdir_p(const char* path)
{
  char* parent = (char*) rtos_malloc(strlen(path)+1);
  const char* slash = path+1; // skip root '/'

  memclr(parent, strlen(path)+1);

  while( NULL != (slash = strchr(slash, '/')) )
  {
    memcpy(parent, path, slash-path);

    // make parent, ignore return value (OK or Already Existed)
    fs_mkdir(parent);

    slash++;
  }
  errnum = fs_mkdir(path);

  rtos_free(parent);

  return (errnum == FS_EOK);
}

bool NewtNffs::remove(const char* path)
{
  errnum = fs_unlink(path);
  return (errnum == FS_EOK);
}

bool NewtNffs::rename(const char* from, const char* to)
{
  errnum = fs_rename(from, to);
  return (errnum == FS_EOK);
}

uint32_t NewtNffs::readFile (const char* filename, void* buffer, uint32_t bufsize, int32_t offset)
{
  NffsFile f(filename);

  VERIFY ( f.exists(), 0 );

  if (offset != 0)
  {
    VERIFY ( f.seek(offset), 0 );
  }

  uint32_t count = f.read( (uint8_t*) buffer, bufsize);
  errnum = f.errnum;

  f.close();

  return count;
}

bool NewtNffs::writeFile(const char* filename, const void* data, uint32_t count, int32_t offset)
{
  NffsFile f(filename, FS_ACCESS_WRITE);

  // File must exists when offset not zero
  if ( offset != 0 )
  {
    VERIFY ( f.seek(offset), 0 );
  }

  bool result = f.write( (const uint8_t*) data, count);
  errnum = f.errnum;

  f.close();

  return result;
}

