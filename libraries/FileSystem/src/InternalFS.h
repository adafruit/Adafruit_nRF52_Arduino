/**************************************************************************/
/*!
    @file     InternalFS.h
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

#ifndef INTERNALFS_H_
#define INTERNALFS_H_

#include "Bluefruit_FileIO.h"

// Internal Flash uses ARM Little FileSystem
// https://github.com/ARMmbed/littlefs
#include "littlefs/lfs.h"

class LittleFS: public BluefruitFS::FileSystemClass
{
  public:
    LittleFS (void);
    virtual ~LittleFS ();

    bool begin (void);
    BluefruitFS::File open (char const *filepath, uint8_t mode = FILE_READ);
    bool exists (char const *filepath);
    bool mkdir (char const *filepath);
    bool remove (char const *filepath);
    bool rmdir (char const *filepath);
    bool rmdir_r (char const *filepath);
    bool format (bool eraseall);

    virtual size_t _f_write (void* fhdl, uint8_t const *buf, size_t size);
    virtual int _f_read (void* fhdl, void *buf, uint16_t nbyte);
    virtual void _f_flush (void* fhdl);
    virtual void _f_close (void* fhdl, bool is_dir);
    virtual bool _f_seek (void* fhdl, uint32_t pos);
    virtual uint32_t _f_position (void* fhdl);
    virtual uint32_t _f_size (void* fhdl);

    virtual BluefruitFS::File _f_openNextFile (void* fhdl, char const* cwd, uint8_t mode);
    virtual void _f_rewindDirectory (void* fhdl);

  private:
    struct lfs_config _lfs_cfg;
    lfs_t _lfs;
    bool _begun;

    BluefruitFS::File _open_file (char const *filepath, uint8_t mode);
    BluefruitFS::File _open_dir (char const *filepath);
};

extern LittleFS InternalFS;

#endif /* INTERNALFS_H_ */
