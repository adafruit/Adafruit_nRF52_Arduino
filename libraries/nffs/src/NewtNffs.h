/**************************************************************************/
/*!
    @file     NewtNffs.h
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
#ifndef NEWTNFFS_H_
#define NEWTNFFS_H_

#include <Arduino.h>

#include "nffs/nffs.h"
#include "fs/fsutil.h"

#include "NffsDirEntry.h"
#include "NffsFile.h"
#include "NffsDir.h"

class NewtNffs
{
public:
  int errnum;

  NewtNffs(void);

  void begin(void);

  // Format the flash
  bool format(void);

  // Make a directory, All intermediate directories must already exist
  bool mkdir(const char* path);

  // Make a directory and its parents if not existed a.k.a "mkdir -p"
  bool mkdir_p(const char* path);

  // Remove a file or an empty directory
  bool remove(const char* path);
  bool rm    (const char* path) { return remove(path); }

  // Remove a non-readonly file or an empty directory
  bool rename(const char* from, const char* to);
  bool mv    (const char* from, const char* to) { return rename(from, to); }
};


extern NewtNffs Nffs;

#endif /* NEWTNFFS_H_ */
