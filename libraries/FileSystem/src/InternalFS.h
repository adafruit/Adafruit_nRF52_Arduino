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

  protected:
    bool _begun;
    bool _mounted;

    LittleFS (lfs_size_t read_size, lfs_size_t prog_size, lfs_size_t block_size, lfs_size_t block_count, lfs_size_t lookahead);
    
    // Internal API: shouldn't be used by Arduino sketch
    virtual size_t _f_write (void* fhdl, uint8_t const *buf, size_t size);
    virtual int _f_read (void* fhdl, void *buf, uint16_t nbyte);
    virtual void _f_flush (void* fhdl);
    virtual void _f_close (void* fhdl, bool is_dir);
    virtual bool _f_seek (void* fhdl, uint32_t pos);
    virtual uint32_t _f_position (void* fhdl);
    virtual uint32_t _f_size (void* fhdl);

    // Raw flash opperations, override to use an external flash chip
    virtual int _flash_read (lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
    virtual int _flash_prog (lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
    virtual int _flash_erase (lfs_block_t block);
    virtual void _flash_erase_all();
    virtual int _flash_sync ();

    virtual BluefruitFS::File _f_openNextFile (void* fhdl, char const* cwd, uint8_t mode);
    virtual void _f_rewindDirectory (void* fhdl);
  private:
    struct lfs_config _lfs_cfg;
    lfs_t _lfs;

    BluefruitFS::File _open_file (char const *filepath, uint8_t mode);
    BluefruitFS::File _open_dir (char const *filepath);

    static int _iflash_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
    static int _iflash_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
    static int _iflash_erase (const struct lfs_config *c, lfs_block_t block);
    static int _iflash_sync (const struct lfs_config *c);
};

extern LittleFS InternalFS;

#endif /* INTERNALFS_H_ */
