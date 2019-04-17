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

#include <Arduino.h>
#include "Bluefruit_FileIO.h"

namespace BluefruitFS
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
  VERIFY(!_is_dir, 0);
  return write(&ch, 1);
}

size_t File::write (uint8_t const *buf, size_t size)
{
  VERIFY(!_is_dir, 0);
  return _fs->_f_write(_hdl, buf, size);
}

int File::read (void)
{
  VERIFY(!_is_dir, -1);
  uint8_t ch;
  return (read(&ch, 1) > 0) ? ch : -1;
}

int File::read (void *buf, uint16_t nbyte)
{
  VERIFY(!_is_dir, 0);
  return _fs->_f_read(_hdl, buf, nbyte);
}

int File::peek (void)
{
  VERIFY(!_is_dir, -1);

  int ch = read();
  uint32_t pos = position();
  seek((pos > 0) ? (pos - 1) : 0);
  return ch;
}

int File::available (void)
{
  return size() - position();
}

bool File::seek (uint32_t pos)
{
  VERIFY(!_is_dir, false);
  return _fs->_f_seek(_hdl, pos);
}

uint32_t File::position (void)
{
  VERIFY(!_is_dir, 0);
  return _fs->_f_position(_hdl);
}

uint32_t File::size (void)
{
  VERIFY(!_is_dir, 0);
  return _fs->_f_size(_hdl);
}

void File::flush (void)
{
  VERIFY(!_is_dir,);
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
