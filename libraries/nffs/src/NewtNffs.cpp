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

NewtNffs Nffs;

NewtNffs::NewtNffs(void)
{
  errnum = FS_EOK;
}

void NewtNffs::begin(void)
{
  nffs_pkg_init();
}

bool NewtNffs::format(void)
{

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
