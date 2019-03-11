/**************************************************************************/
/*!
    @file     EddyStone.cpp
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

#include "bluefruit.h"

/*------------------------------------------------------------------*/
/* EddyStone URL
 *------------------------------------------------------------------*/
char const * const prefix_scheme[] =
{
    [0] = "http://www."  ,
    [1] = "https://www." ,
    [2] = "http://"      ,
    [3] = "https://"
};
enum { PREFIX_COUNT = sizeof(prefix_scheme)/sizeof(char*) };

char const * const url_expansion[] =
{
    [0 ] = ".com/"  ,
    [1 ] = ".org/"  ,
    [2 ] = ".edu/"  ,
    [3 ] = ".net/"  ,
    [4 ] = ".info/" ,
    [5 ] = ".biz/"  ,
    [6 ] = ".gov/"  ,

    [7 ] = ".com"  ,
    [8 ] = ".org"  ,
    [9 ] = ".edu"  ,
    [10] = ".net"  ,
    [11] = ".info" ,
    [12] = ".biz"  ,
    [13] = ".gov"  ,
};

enum { EXPANNSION_COUNT = sizeof(url_expansion)/sizeof(char*) };

EddyStoneUrl::EddyStoneUrl(void)
{
  _rssi = 0;
  _url  = NULL;
}

EddyStoneUrl::EddyStoneUrl(int8_t rssiAt0m, const char* url)
{
  _rssi = rssiAt0m;
  _url  = url;
}

void EddyStoneUrl::setUrl(const char* url)
{
  _url = url;
}

void EddyStoneUrl::setRssi(int8_t rssiAt0m)
{
  _rssi = rssiAt0m;
}

char const* findExpansion(char const* p_url, uint8_t * p_idx)
{
  for(uint8_t i=0; i<EXPANNSION_COUNT; i++)
  {
    char const * substr = strstr(p_url, url_expansion[i]);

    if ( substr )
    {
      *p_idx = i;
      return substr;
    }
  }

  return NULL;
}

bool EddyStoneUrl::start(void)
{
  enum { URL_MAXLEN = 17 };
  struct ATTR_PACKED {
    uint16_t eddy_uuid;

    uint8_t  frame_type;
    int8_t   rssi;
    uint8_t  url_scheme;
    uint8_t  urlencode[URL_MAXLEN];
  }eddy =
  {
      .eddy_uuid  = UUID16_SVC_EDDYSTONE,
      .frame_type = EDDYSTONE_TYPE_URL,
      .rssi       = _rssi,
      .url_scheme = 0xff
  };

  const char* url = _url;

  // Detect url scheme
  for(uint8_t i=0; i<PREFIX_COUNT; i++)
  {
    uint8_t prelen = strlen(prefix_scheme[i]);
    if ( !memcmp(url, prefix_scheme[i], prelen) )
    {
      eddy.url_scheme = i;
      url += prelen;

      break;
    }
  }
  VERIFY( eddy.url_scheme < PREFIX_COUNT );

  // Encode url data
  uint8_t len = 0;

  while(*url)
  {
    uint8_t ex_code;
    char const * expansion = findExpansion(url, &ex_code);

    // copy url up to the found expansion, if expansion is found, one more
    // byte must be reserved for it
    uint8_t cp_num = (expansion) ?  (expansion-url) : strlen(url);
    if ( cp_num > URL_MAXLEN-(len + (expansion ? 1:0)) )
    {
      LOG_LV1("EDDYS", "url is too long");
      return false;
    }

    memcpy(eddy.urlencode+len, url, cp_num);
    url += cp_num;
    len += cp_num;

    // copy expansion code if found
    if (expansion)
    {
      eddy.urlencode[len++] = ex_code;
      url += strlen(url_expansion[ex_code]);
    }
  }

  // Add UUID16 list with EddyStone
  VERIFY ( Bluefruit.Advertising.addUuid(UUID16_SVC_EDDYSTONE) );

  // Add Eddystone Service Data
  VERIFY ( Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_SERVICE_DATA, &eddy, len + 5) );

  return true;
}
