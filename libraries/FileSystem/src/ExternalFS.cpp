/**************************************************************************/
/*!
    @file     ExternalFS.cpp
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

#ifdef NRF52840_XXAA

#include <Arduino.h>
#include <string.h>
#include "ExternalFS.h"

FatFS::FatFS ()
{
}
FatFS::~FatFS ()
{
}

bool FatFS::begin ()
{
}
File FatFS::open (char const *filename, uint8_t mode)
{
}
bool FatFS::exists (char const *filepath)
{
}
bool FatFS::mkdir (char const *filepath)
{
}
bool FatFS::remove (char const *filepath)
{
}
bool FatFS::rmdir (char const *filepath)
{
}
bool FatFS::rmdir_r (char const *filepath)
{
}

size_t FatFS::_f_write (void* fhdl, uint8_t const *buf, size_t size)
{
}
int FatFS::_f_read (void* fhdl, void *buf, uint16_t nbyte)
{
}
void FatFS::_f_flush (void* fhdl)
{
}
void FatFS::_f_close (void* fhdl, bool is_dir)
{
}
bool FatFS::_f_seek (void* fhdl, uint32_t pos)
{
}
uint32_t FatFS::_f_position (void* fhdl)
{
}
uint32_t FatFS::_f_size (void* fhdl)
{
}

File FatFS::_f_openNextFile (void* fhdl, char const* cwd, uint8_t mode)
{
}
void FatFS::_f_rewindDirectory (void* fhdl)
{
}

#endif
