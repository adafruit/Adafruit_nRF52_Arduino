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

using namespace Adafruit_LittleFS_Namespace;


//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+

Adafruit_LittleFS::Adafruit_LittleFS (void)
  : Adafruit_LittleFS(NULL)
{

}

Adafruit_LittleFS::Adafruit_LittleFS (struct lfs_config* cfg)
{
  varclr(&_lfs);
  _lfs_cfg = cfg;
  _mounted = false;
}

Adafruit_LittleFS::~Adafruit_LittleFS ()
{

}

// Initialize and mount the file system
// Return true if mounted successfully else probably corrupted.
// User should format the disk and try again
bool Adafruit_LittleFS::begin (struct lfs_config * cfg)
{
  if ( _mounted ) return true;

  if (cfg) _lfs_cfg = cfg;
  if (!_lfs_cfg) return false;

  VERIFY_LFS(lfs_mount(&_lfs, _lfs_cfg), false);
  _mounted = true;

  return true;
}

// Tear down and unmount file system
void Adafruit_LittleFS::end(void)
{
  if (!_mounted) return;

  _mounted = false;
  VERIFY_LFS(lfs_unmount(&_lfs), );
}

bool Adafruit_LittleFS::format (void)
{
  // if already mounted: umount -> format -> remount
  if(_mounted) VERIFY_LFS(lfs_unmount(&_lfs), false);

  VERIFY_LFS(lfs_format(&_lfs, _lfs_cfg), false);

  if (_mounted) VERIFY_LFS(lfs_mount(&_lfs, _lfs_cfg), false);

  return true;
}

// Open a file or folder
Adafruit_LittleFS_Namespace::File Adafruit_LittleFS::open (char const *filepath, uint8_t mode)
{
  return Adafruit_LittleFS_Namespace::File(filepath, mode, *this);
}

// Check if file or folder exists
bool Adafruit_LittleFS::exists (char const *filepath)
{
  struct lfs_info info;
  return 0 == lfs_stat(&_lfs, filepath, &info);
}

// Create a directory, create intermediate parent if needed
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

// Remove a file
bool Adafruit_LittleFS::remove (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath), false);
  return true;
}

// Remove a folder
bool Adafruit_LittleFS::rmdir (char const *filepath)
{
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}

// Remove a folder recursively
bool Adafruit_LittleFS::rmdir_r (char const *filepath)
{
  /* adafruit: lfs is modified to remove non-empty folder,
   According to below issue, comment these 2 line won't corrupt filesystem
   https://github.com/ARMmbed/littlefs/issues/43 */
  VERIFY_LFS(lfs_remove(&_lfs, filepath));
  return true;
}

//------------- Debug -------------//
#if CFG_DEBUG

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
