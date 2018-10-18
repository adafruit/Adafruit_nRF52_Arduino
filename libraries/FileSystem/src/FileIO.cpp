/**************************************************************************/
/*!
    @file     FileIO.cpp
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

#include "rtos.h"
#include "FileIO.h"

namespace BluefuritLib
{

File::File (FileSystemClass &fs)
{
  _fs = &fs;
  _hdl = NULL;
  _path = NULL;
  _is_dir = false;
}

File::File (char const *filename, uint8_t mode, FileSystemClass &fs)
{
  _fs = &fs;
  _hdl = NULL;
  _path = NULL;
  _is_dir = false;

  open(filename, mode);
}

File& File::operator = (const File &rhs)
{
  // close if currently opened
  if ( _hdl ) close();

  memcpy(this, &rhs, sizeof(File));

  return *this;
}

File::~File ()
{

}

bool File::open (char const *filename, uint8_t mode)
{
  // close if currently opened
  if ( _hdl ) close();

  *this = _fs->open(filename, mode);
  return _hdl != NULL;
}

size_t File::write (uint8_t ch)
{
  return write(&ch, 1);
}

size_t File::write (uint8_t const *buf, size_t size)
{
  return _fs->_f_write(_hdl, buf, size);
}

int File::read (void)
{
  uint8_t ch;
  return (read(&ch, 1) > 0) ? ch : -1;
}

int File::read (void *buf, uint16_t nbyte)
{
  return _fs->_f_read(_hdl, buf, nbyte);
}

int File::peek (void)
{
  int ch = read();
  seek((position() > 0) ? (position() - 1) : 0);
  return ch;
}

int File::available (void)
{
  return size() - position();
}

bool File::seek (uint32_t pos)
{
  return _fs->_f_seek(_hdl, pos);
}

uint32_t File::position (void)
{
  return _fs->_f_position(_hdl);
}

uint32_t File::size (void)
{
  return _fs->_f_size(_hdl);
}

void File::flush (void)
{
  _fs->_f_flush(_hdl);
}

void File::close (void)
{
  if ( _hdl )
  {
    _fs->_f_close(_hdl, _is_dir);
    rtos_free(_hdl);
  }

  if ( _path ) rtos_free(_path);

  _hdl = NULL;
  _path = NULL;
}

File::operator bool (void)
{
  return _hdl != NULL;
}

char const* File::name (void)
{
  // return barename only
  char* barename = strrchr(_path, '/');
  return barename ? (barename + 1) : _path;
}

char const* File::path (void)
{
  return _path;
}

bool File::isDirectory (void)
{
  return _is_dir;
}

File File::openNextFile (uint8_t mode)
{
  if ( !_is_dir ) return File(*_fs);
  return _fs->_f_openNextFile(_hdl, _path, mode);
}

void File::rewindDirectory (void)
{
  _fs->_f_rewindDirectory(_hdl);
}


}
