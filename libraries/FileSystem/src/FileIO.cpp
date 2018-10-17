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
}

File::File (char const *filename, uint8_t mode, FileSystemClass &fs)
{
  _fs = &fs;
  _hdl = NULL;
  open(filename, mode);
}

File::~File ()
{

}

bool File::open (char const *filename, uint8_t mode)
{
  // close previous opened
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

}

int File::available (void)
{

}

void File::flush (void)
{
  _fs->_f_flush(_hdl);
}



bool File::seek (uint32_t pos)
{

}

uint32_t File::position (void)
{

}

uint32_t File::size (void)
{

}

void File::close (void)
{
  if ( _hdl )
  {
    _fs->_f_close(_hdl);
    rtos_free(_hdl);
  }

  _hdl = NULL;
}

File::operator bool ()
{
  return _hdl != NULL;
}

char const * File::name (void)
{

}

bool File::isDirectory (void)
{

}

File File::openNextFile (uint8_t mode)
{

}

void File::rewindDirectory (void)
{

}


}
