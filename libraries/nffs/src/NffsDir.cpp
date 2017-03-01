/**************************************************************************/
/*!
    @file     NffsDir.cpp
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

void NffsDir::_init(void)
{
  _dir   = NULL;
  errnum = FS_EOK;
}

NffsDir::NffsDir(void)
{
  _init();
}

NffsDir::NffsDir(const char* path)
{
  _init();

  open(path);
}


bool NffsDir::open(const char* path)
{
  errnum = fs_opendir(path, &_dir);

  return (errnum == FS_EOK);
}

bool NffsDir::open(const char* parent_dir, NffsDirEntry& dirent)
{
  bool result = false;
  char* name = (char*) rtos_malloc(NFFS_FILENAME_MAX_LEN+1);

  VERIFY(name);

  strcpy(name, parent_dir);
  strcat(name, "/");

  int len = strlen(name);

  if ( dirent.getName(name+len, NFFS_FILENAME_MAX_LEN+1-len) > 0 )
  {
    result = open(name);
  }

  rtos_free(name);

  return result;
}

bool NffsDir::close(void)
{
  errnum = fs_closedir(_dir);

  return (errnum == FS_EOK);
}

bool NffsDir::exists(void)
{
  return (_dir != NULL);
}

NffsDirEntry NffsDir::read(void)
{
  struct fs_dirent* dirent = NULL ;

  errnum = fs_readdir(_dir, &dirent);

  return NffsDirEntry(dirent);
}

bool NffsDir::read(NffsDirEntry* entry)
{
  *entry = read();

  return entry->exists();
}
