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
#define FILE_APPEND 2

class LittleFS;

namespace BluefuritLib
{

class FileSystemClass;

class File: public Stream
{
  public:
    File (FileSystemClass &fs);
    File (char const *_filename, uint8_t _mode, FileSystemClass &fs);
    ~File ();

    virtual size_t write (uint8_t);
    virtual size_t write (uint8_t const *buf, size_t size);
    virtual int read ();
    virtual int peek ();
    virtual int available ();
    virtual void flush ();
    int read (void *buf, uint16_t nbyte);
    bool seek (uint32_t pos);
    uint32_t position ();
    uint32_t size ();
    void close ();
    operator bool ();
    char const * name ();
    bool isDirectory ();
    File openNextFile (uint8_t mode = FILE_READ);
    void rewindDirectory (void);

    //using Print::write;

  private:
    FileSystemClass &_fs;
    void* _hdl;

//    String filename;

    friend class ::LittleFS;
};

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

    virtual bool rmdir (char const *filepath) = 0;
};

}

// We enclose File and FileSystem classes in namespace BridgeLib to avoid
// conflicts with legacy SD library.

// This ensure compatibility with older sketches that uses only Bridge lib
// (the user can still use File instead of BridgeFile)
//using namespace BridgeLib;
//
//// This allows sketches to use BridgeLib::File together with SD library
//// (you must use BridgeFile instead of File when needed to disambiguate)
//typedef BridgeLib::File            BridgeFile;
//typedef BridgeLib::FileSystemClass BridgeFileSystemClass;
//#define BridgeFileSystem           BridgeLib::FileSystem

#endif
