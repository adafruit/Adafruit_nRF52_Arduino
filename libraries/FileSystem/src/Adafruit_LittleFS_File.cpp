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
#include "Adafruit_LittleFS.h"
#include "littlefs/lfs.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

using namespace LittleFilesystem;


File::File (Adafruit_LittleFS &fs)
{
  _fs = &fs;
  _hdl = NULL;
  _dir = NULL;
  _path = NULL;
  _is_dir = false;
}

File::File (char const *filename, uint8_t mode, Adafruit_LittleFS &fs)
{
  _fs = &fs;
  _hdl = NULL;
  _dir = NULL;
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

File File::_open_file (char const *filepath, uint8_t mode)
{
  File file(*_fs);

  int flags = (mode == FILE_READ) ? LFS_O_RDONLY :
              (mode == FILE_WRITE) ? (LFS_O_RDWR | LFS_O_CREAT) : 0;

  if ( flags )
  {
    lfs_file_t* fhdl = (lfs_file_t*) rtos_malloc(sizeof(lfs_file_t));
    VERIFY(fhdl, file);

    int rc = lfs_file_open(_fs->getFS(), fhdl, filepath, flags);

    if ( rc )
    {
      rtos_free(fhdl);
      PRINT_LFS_ERR(rc);
    }
    else
    {
      // move to end of file
      if ( mode == FILE_WRITE ) lfs_file_seek(_fs->getFS(), fhdl, 0, LFS_SEEK_END);

      file._hdl = fhdl;
      file._is_dir = false;

      file._path = (char*) rtos_malloc(strlen(filepath) + 1);
      strcpy(file._path, filepath);
    }
  }

  return file;
}

File File::_open_dir (char const *filepath)
{
  File file(*_fs);

  lfs_dir_t* fhdl = (lfs_dir_t*) rtos_malloc(sizeof(lfs_dir_t));
  int rc = lfs_dir_open(_fs->getFS(), fhdl, filepath);

  if ( rc )
  {
    rtos_free(fhdl);
    PRINT_LFS_ERR(rc);
  }
  else
  {
    file._dir = fhdl;
    file._is_dir = true;

    file._path = (char*) rtos_malloc(strlen(filepath) + 1);
    strcpy(file._path, filepath);
  }

  return file;
}

bool File::open (char const *filepath, uint8_t mode)
{
  // close if currently opened
  if ( _hdl ) close();

  File file(*_fs);
  struct lfs_info info;

  int rc = lfs_stat(_fs->getFS(), filepath, &info);
  if ( LFS_ERR_OK == rc )
  {
    // file existed, open file or directory accordingly
    file = (info.type == LFS_TYPE_REG) ? _open_file(filepath, mode) : _open_dir(filepath);
  }
  else if ( LFS_ERR_NOENT == rc )
  {
    // file not existed, only proceed with FILE_WRITE mode
    if ( mode == FILE_WRITE ) file = _open_file(filepath, mode);
  }
  else
  {
    PRINT_LFS_ERR(rc);
  }

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

  lfs_ssize_t wrcount = lfs_file_write(_fs->getFS(), _hdl, buf, size);
  VERIFY(wrcount > 0, 0);
  return wrcount;
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
  return lfs_file_read(_fs->getFS(), _hdl, buf, nbyte);
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
  return lfs_file_seek(_fs->getFS(), _hdl, pos, LFS_SEEK_SET) >= 0;
}

uint32_t File::position (void)
{
  VERIFY(!_is_dir, 0);
  return lfs_file_tell(_fs->getFS(), _hdl);
}

uint32_t File::size (void)
{
  VERIFY(!_is_dir, 0);
  return lfs_file_size(_fs->getFS(), _hdl);
}

void File::flush (void)
{
  VERIFY(!_is_dir,);
  lfs_file_sync(_fs->getFS(), _hdl);
}

void File::close (void)
{
  if ( _hdl )
  {
    if ( _is_dir )
    {
      lfs_dir_close(_fs->getFS(), _dir);
    }
    else
    {
      lfs_file_close(_fs->getFS(), _hdl);
    }

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
  File file(*_fs);

  if ( !_is_dir ) return file;

  struct lfs_info info;

  int rc;

  // lfs_dir_read return 0 when reaching end of directory, 1 if found an entry
  // skip "." and ".." entries
  do
  {
    rc = lfs_dir_read(_fs->getFS(), _dir, &info);
  } while ( rc == 1 && (!strcmp(".", info.name) || !strcmp("..", info.name)) );

  if ( rc == 1 )
  {
    // string cat name with current folder
    char filepath[strlen(_path) + 1 + strlen(info.name) + 1];

    strcpy(filepath, _path);
    if ( !(_path[0] == '/' && _path[1] == 0) ) strcat(filepath, "/");    // only add '/' if cwd is not root
    strcat(filepath, info.name);

    file.open(filepath, mode);
  }
  else if ( rc < 0 )
  {
    PRINT_LFS_ERR(rc);
  }

  return file;
}

void File::rewindDirectory (void)
{
  VERIFY_LFS(lfs_dir_rewind(_fs->getFS(), _dir),);
}
