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

using namespace Adafruit_LittleFS_Namespace;

File::File (Adafruit_LittleFS &fs)
{
  _fs = &fs;
  _is_dir = false;
  _name[0] = 0;
  _dir_path = NULL;

  _dir = NULL;
  _file = NULL;
}

File::File (char const *filename, uint8_t mode, Adafruit_LittleFS &fs)
 : File(fs)
{
  this->open(filename, mode);
}

bool File::_open_file (char const *filepath, uint8_t mode)
{
  int flags = (mode == FILE_O_READ) ? LFS_O_RDONLY :
              (mode == FILE_O_WRITE) ? (LFS_O_RDWR | LFS_O_CREAT) : 0;

  if ( flags )
  {
    _file = (lfs_file_t*) rtos_malloc(sizeof(lfs_file_t));
    if (!_file) return false;

    int rc = lfs_file_open(_fs->getFS(), _file, filepath, flags);

    if ( rc )
    {
      // failed to open
      PRINT_LFS_ERR(rc);
      return false;
    }

    // move to end of file
    if ( mode == FILE_O_WRITE ) lfs_file_seek(_fs->getFS(), _file, 0, LFS_SEEK_END);

    _is_dir = false;
  }

  return true;
}

bool File::_open_dir (char const *filepath)
{
  _dir = (lfs_dir_t*) rtos_malloc(sizeof(lfs_dir_t));
  if (!_dir) return false;

  int rc = lfs_dir_open(_fs->getFS(), _dir, filepath);

  if ( rc )
  {
    // failed to open
    PRINT_LFS_ERR(rc);
    return false;
  }

  _is_dir = true;

  _dir_path = (char*) rtos_malloc(strlen(filepath) + 1);
  strcpy(_dir_path, filepath);

  return true;
}

bool File::open (char const *filepath, uint8_t mode)
{
  bool ret = false;

  // close if currently opened
  if ( (*this) ) close();

  struct lfs_info info;
  int rc = lfs_stat(_fs->getFS(), filepath, &info);

  if ( LFS_ERR_OK == rc )
  {
    // file existed, open file or directory accordingly
    ret = (info.type == LFS_TYPE_REG) ? _open_file(filepath, mode) : _open_dir(filepath);
  }
  else if ( LFS_ERR_NOENT == rc )
  {
    // file not existed, only proceed with FILE_O_WRITE mode
    if ( mode == FILE_O_WRITE ) ret = _open_file(filepath, mode);
  }
  else
  {
    PRINT_LFS_ERR(rc);
  }

  // save bare file name
  if (ret)
  {
    char const* splash = strrchr(filepath, '/');
    strncpy(_name, splash ? (splash + 1) : filepath, LFS_NAME_MAX);
  }

  return ret;
}

size_t File::write (uint8_t ch)
{
  VERIFY(!_is_dir, 0);
  return write(&ch, 1);
}

size_t File::write (uint8_t const *buf, size_t size)
{
  VERIFY(!_is_dir, 0);

  lfs_ssize_t wrcount = lfs_file_write(_fs->getFS(), _file, buf, size);
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
  return lfs_file_read(_fs->getFS(), _file, buf, nbyte);
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
  return lfs_file_seek(_fs->getFS(), _file, pos, LFS_SEEK_SET) >= 0;
}

uint32_t File::position (void)
{
  VERIFY(!_is_dir, 0);
  return lfs_file_tell(_fs->getFS(), _file);
}

uint32_t File::size (void)
{
  VERIFY(!_is_dir, 0);
  return lfs_file_size(_fs->getFS(), _file);
}

void File::flush (void)
{
  VERIFY(!_is_dir,);
  lfs_file_sync(_fs->getFS(), _file);
}

void File::close (void)
{
  if ( (*this) )
  {
    if ( _is_dir )
    {
      lfs_dir_close(_fs->getFS(), _dir);
      rtos_free(_dir);
      _dir = NULL;

      if ( _dir_path ) rtos_free(_dir_path);
      _dir_path = NULL;
    }
    else
    {
      lfs_file_close(_fs->getFS(), _file);
      rtos_free(_file);
      _file = NULL;
    }
  }
}

File::operator bool (void)
{
  return (_file != NULL) || (_dir != NULL);
}

char const* File::name (void)
{
  return _name;
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
    char filepath[strlen(_dir_path) + 1 + strlen(info.name) + 1];

    strcpy(filepath, _dir_path);
    if ( !(_dir_path[0] == '/' && _dir_path[1] == 0) ) strcat(filepath, "/");    // only add '/' if cwd is not root
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
