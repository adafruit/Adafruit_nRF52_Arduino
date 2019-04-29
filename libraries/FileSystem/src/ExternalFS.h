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
#ifndef EXTERNALFS_H_
#define EXTERNALFS_H_

#include "Bluefruit_FileIO.h"

// External Flash uses Fat FS as file system
// http://elm-chan.org/fsw/ff/00index_e.html
#include "fatfs/source/ff.h"

class FatFS: public BluefruitFS::FileSystemClass
{
  public:
    FatFS ();
    virtual ~FatFS ();

    bool begin (void);
    BluefruitFS::File open (char const *filepath, uint8_t mode = FILE_READ);
    bool exists (char const *filepath);
    bool mkdir (char const *filepath);
    bool remove (char const *filepath);
    bool rmdir (char const *filepath);
    bool rmdir_r (char const *filepath);
    bool format (bool eraseall);

    // Internal API: shouldn't be used by Arduino sketch
    virtual size_t _f_write (void* fhdl, uint8_t const *buf, size_t size);
    virtual int _f_read (void* fhdl, void *buf, uint16_t nbyte);
    virtual void _f_flush (void* fhdl);
    virtual void _f_close (void* fhdl, bool is_dir);
    virtual bool _f_seek (void* fhdl, uint32_t pos);
    virtual uint32_t _f_position (void* fhdl);
    virtual uint32_t _f_size (void* fhdl);

    virtual BluefruitFS::File _f_openNextFile (void* fhdl, char const* cwd, uint8_t mode);
    virtual void _f_rewindDirectory (void* fhdl);

    // Should call when flash contents is changed out of the awareness of ExternalFS such as USB MSC write
    void updateCache (uint32_t lba, void const* buffer, uint32_t bufsize);

  private:
    FATFS* _fs;
    bool _begun;

    BluefruitFS::File _open_file (char const *filepath, uint8_t mode);
    BluefruitFS::File _open_dir (char const *filepath);
};

extern FatFS ExternalFS;

#endif /* EXTERNALFS_H_ */
