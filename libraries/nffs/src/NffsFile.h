/**************************************************************************/
/*!
    @file     NffsFile.h
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
#ifndef NFFSFILE_H_
#define NFFSFILE_H_

#include <Arduino.h>

class NffsFile : public Stream
{
private:
  struct fs_file* _file;

  void _init(void);

public:
  int errnum;

  NffsFile(void);
  virtual ~NffsFile();

  NffsFile(const char* path, uint8_t flags = FS_ACCESS_READ);

  bool open(const char* path, uint8_t flags = FS_ACCESS_READ);
  bool open(const char* parent_dir, NffsDirEntry& dirent, uint8_t flags = FS_ACCESS_READ);

  bool close(void);
  bool exists(void);

  uint32_t size(void);
  uint32_t tell(void);
  bool     seek(int32_t offset);

  bool     seekForward (uint32_t offset);
  bool     seekBackward(uint32_t offset);

  // Stream API
  virtual size_t    write      ( uint8_t b );
  virtual size_t    write      ( const uint8_t *data, size_t len );
  virtual int       available  ( void );
  virtual int       peek       ( void );
  virtual void      flush      ( void );
  virtual int       read       ( void );

          size_t    read       ( uint8_t * buf, size_t size );
          size_t    read       ( char    * buf, size_t size )
          {
            return read( (uint8_t*) buf, size);
          }


  // pull in write(str) and write(buf, size) from Print
  using Print::write;
};

#endif /* NFFSFILE_H_ */
