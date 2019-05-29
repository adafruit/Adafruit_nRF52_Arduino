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

#ifndef ADAFRUIT_LITTLEFS_H_
#define ADAFRUIT_LITTLEFS_H_

#include <Stream.h>

// Internal Flash uses ARM Little FileSystem
// https://github.com/ARMmbed/littlefs
#include "littlefs/lfs.h"

#include "Adafruit_LittleFS_File.h"

class Adafruit_LittleFS
{
  public:
    Adafruit_LittleFS (void);
    virtual ~Adafruit_LittleFS ();

    bool begin (void);

    // Open the specified file/directory with the supplied mode (e.g. read or
    // write, etc). Returns a File object for interacting with the file.
    // Note that currently only one file can be open at a time.
    LittleFilesystem::File open (char const *filename, uint8_t mode = FILE_READ);

    // Methods to determine if the requested file path exists.
    bool exists (char const *filepath);

    // Create the requested directory hierarchy--if intermediate directories
    // do not exist they will be created.
    bool mkdir (char const *filepath);

    // Delete the file.
    bool remove (char const *filepath);

    // Delete a folder (must be empty)
    bool rmdir (char const *filepath);

    // Delete a folder (recursively)
    bool rmdir_r (char const *filepath);

    // format whole file system
    bool format (bool eraseall);

    lfs_t* getFS(void)
    {
      return &_lfs;
    }

  protected:
    bool _begun;
    bool _mounted;

    Adafruit_LittleFS (uint32_t read_size, uint32_t prog_size, uint32_t block_size, uint32_t block_count, uint32_t lookahead);
    
    // Internal API: shouldn't be used by Arduino sketch
    // Raw flash opperations, override to use an external flash chip
    virtual int _flash_read (lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
    virtual int _flash_prog (lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
    virtual int _flash_erase (lfs_block_t block);
    virtual void _flash_erase_all();
    virtual int _flash_sync ();

  private:
    struct lfs_config _lfs_cfg;
    lfs_t _lfs;

    static int _iflash_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
    static int _iflash_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
    static int _iflash_erase (const struct lfs_config *c, lfs_block_t block);
    static int _iflash_sync (const struct lfs_config *c);
};

extern Adafruit_LittleFS InternalFS;

#if !CFG_DEBUG
  #define VERIFY_LFS(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, NULL)
  #define PRINT_LFS_ERR(_err)
#else
  #define VERIFY_LFS(...)       _GET_3RD_ARG(__VA_ARGS__, VERIFY_ERR_2ARGS, VERIFY_ERR_1ARGS)(__VA_ARGS__, dbg_strerr_lfs)
  #define PRINT_LFS_ERR(_err)   VERIFY_MESS(_err, dbg_strerr_lfs)

  const char* dbg_strerr_lfs (int32_t err);
#endif

#endif /* ADAFRUIT_LITTLEFS_H_ */
