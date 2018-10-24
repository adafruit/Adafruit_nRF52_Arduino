/*
 Copyright (c) 2013 Arduino LLC. All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**************************************************************************/
/*!
    @file     FileIO.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)
    Copyright (c) 2018, Adafruit Industries (adafruit.com)
*/
/**************************************************************************/

#ifndef __FILEIO_H__
#define __FILEIO_H__

#include <Stream.h>

#define FILE_READ 0
#define FILE_WRITE 1

#define FILE_NAME_MAX 255

class LittleFS;
class FatFS;

namespace BluefuritLib
{

class FileSystemClass;

class File: public Stream
{
  public:
    File (FileSystemClass &fs);
    File (char const *filename, uint8_t mode, FileSystemClass &fs);
    File & operator = (const File &rhs);
    virtual ~File ();

    bool open (char const *filename, uint8_t mode);

    virtual size_t write (uint8_t ch);
    virtual size_t write (uint8_t const *buf, size_t size);

    virtual int read (void);
    int read (void *buf, uint16_t nbyte);

    virtual int peek (void);
    virtual int available (void);
    virtual void flush (void);

    bool seek (uint32_t pos);
    uint32_t position (void);
    uint32_t size (void);

    void close (void);
    operator bool (void);
    char const* name (void);
    char const* path (void);

    bool isDirectory (void);
    File openNextFile (uint8_t mode = FILE_READ);
    void rewindDirectory (void);

    using Print::write;

  private:
    FileSystemClass* _fs;
    void* _hdl;
    char* _path;
    bool _is_dir;

    friend class ::LittleFS;
    friend class ::FatFS;
};

// Abstract Interface Class
class FileSystemClass
{
  public:
    virtual bool begin () = 0;

    // Open the specified file/directory with the supplied mode (e.g. read or
    // write, etc). Returns a File object for interacting with the file.
    // Note that currently only one file can be open at a time.
    virtual File open (char const *filename, uint8_t mode) = 0;

    // Methods to determine if the requested file path exists.
    virtual bool exists (char const *filepath) = 0;

    // Create the requested directory hierarchy--if intermediate directories
    // do not exist they will be created.
    virtual bool mkdir (char const *filepath) = 0;

    // Delete the file.
    virtual bool remove (char const *filepath) = 0;

    // Delete a folder (must be empty)
    virtual bool rmdir (char const *filepath) = 0;

    // Delete a folder (recursively)
    virtual bool rmdir_r (char const *filepath) = 0;

    //--------------------------------------------------------------------+
    // Internal API, shouldn't call directly
    //--------------------------------------------------------------------+
    virtual size_t _f_write (void* fhdl, uint8_t const *buf, size_t size) = 0;
    virtual int _f_read (void* fhdl, void *buf, uint16_t nbyte) = 0;
    virtual void _f_flush (void* fhdl) = 0;
    virtual void _f_close (void* fhdl, bool is_dir) = 0;

    virtual bool _f_seek (void* fhdl, uint32_t pos) = 0;
    virtual uint32_t _f_position (void* fhdl) = 0;
    virtual uint32_t _f_size (void* fhdl) = 0;

    virtual File _f_openNextFile (void* fhdl, char const* cwd, uint8_t mode) = 0;
    virtual void _f_rewindDirectory (void* fhdl) = 0;
};

}

// We enclose File and FileSystem classes in namespace BluefuritLib to avoid
// conflicts with legacy SD library.

// This ensure compatibility with older sketches that uses only Bridge lib
// (the user can still use File instead of BridgeFile)
using namespace BluefuritLib;

// This allows sketches to use BluefuritLib::File together with SD library
// (you must use BridgeFile instead of File when needed to disambiguate)
typedef BluefuritLib::File BluefruitFile;

#include "InternalFS.h"
#include "ExternalFS.h"

#endif
