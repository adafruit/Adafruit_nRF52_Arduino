/**************************************************************************/
/*!
    @file     NffsFile.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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

#include "NewtNffs.h"

void NffsFile::_init(void)
{
  _file = NULL;
  errnum = 0;
}

NffsFile::NffsFile(void)
{
  _init();
}

NffsFile::NffsFile(const char* path, uint8_t flags)
{
  _init();
  open(path, flags);
}

bool NffsFile::open(const char* path, uint8_t flags)
{
  errnum = fs_open(path, flags, &_file);

  return (errnum == FS_EOK);
}

bool NffsFile::existed(void)
{
  return _file != NULL;
}

bool NffsFile::close(void)
{
  errnum = fs_close(_file);

  return (errnum == FS_EOK);
}

uint32_t NffsFile::size(void)
{
  uint32_t size = 0;
  errnum = fs_filelen(_file, &size);

  return size;
}

uint32_t NffsFile::tell(void)
{
  return fs_getpos(_file);
}

bool NffsFile::seek(uint32_t offset)
{
  errnum = fs_seek(_file, offset);

  return (errnum == FS_EOK);
}

bool NffsFile::seekForward (uint32_t offset)
{
  errnum = fs_seek(_file, tell() + offset);

  return (errnum == FS_EOK);
}

bool NffsFile::seekBackward(uint32_t offset)
{
  errnum = fs_seek(_file, tell() - offset);

  return (errnum == FS_EOK);
}


int NffsFile::read( void )
{
  uint8_t data;
  return (this->read(&data, 1) == 1) ? data : EOF;
}

size_t NffsFile::read(uint8_t * buf, size_t size)
{
  size_t readlen = 0;
  errnum = fs_read(_file, (uint32_t) size, buf, (uint32_t*) &readlen);

  return readlen;
}

size_t NffsFile::write( uint8_t b )
{
  return this->write(&b, 1);
}

size_t NffsFile::write( const uint8_t *data, size_t len )
{
  errnum = fs_write(_file, data, len);

  return (errnum == FS_EOK) ? len : 0;
}

int NffsFile::available( void )
{
  return size() - tell();
}

int NffsFile::peek( void )
{
  int ch = read();
  seekBackward(1);
  return ch;
}

void NffsFile::flush( void )
{
  // do nothing
}


