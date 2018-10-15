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

#include "FileIO.h"

namespace BluefuritLib
{

File::File (FileSystemClass &fs)
 : _fs(fs)
{
  _hdl = NULL;
}

File::File (char const *_filename, uint8_t _mode, FileSystemClass &fs)
  : _fs(fs)
{
  _hdl = NULL;
}

File::~File ()
{

}

size_t File::write (uint8_t)
{

}

size_t File::write (uint8_t const *buf, size_t size)
{

}

int File::read ()
{

}

int File::peek ()
{

}

int File::available ()
{

}

void File::flush ()
{

}

int File::read (void *buf, uint16_t nbyte)
{

}

bool File::seek (uint32_t pos)
{

}

uint32_t File::position ()
{

}

uint32_t File::size ()
{

}

void File::close ()
{

}

File::operator bool ()
{

}

char const * File::name ()
{

}

bool File::isDirectory ()
{

}

File File::openNextFile (uint8_t mode)
{

}

void File::rewindDirectory (void)
{

}


}
