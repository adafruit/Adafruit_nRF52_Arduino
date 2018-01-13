/**************************************************************************/
/*!
    @file     HAPProcedure.cpp
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

#include <Arduino.h>
#include "HAPUuid.h"
#include "HAPProcedure.h"

/**
 * Decode a TLV8 item
 * @param pp_data   payload pointer, got update to next item when done
 * @param p_len     len of payload, got updated to remaining item when done
 * @param buf       optional buffer, required if the decoded payload > 255
 *                  to assembly data into continuous buffer
 * @param bufsize   size of buffer (optional)
 *
 * @return  TLV8 item
 */
TLV8_t tlv8_decode_next(uint8_t const** pp_data, uint16_t* p_len, void* buf, uint16_t bufsize)
{
  TLV8_t tlv;
  varclr(&tlv);

  if ( (*p_len) )
  {
    if ( buf && bufsize ) tlv.value = buf;

    tlv.type = (**pp_data);

    uint8_t fraqlen;
    do {
      uint8_t const* data = *pp_data;

      // different item type
      if ( tlv.type != data[0] ) break;

      fraqlen  = data[1];

      tlv.len += fraqlen;

      if ( buf )
      {
        uint16_t cp = min16(bufsize, fraqlen);

        if ( cp )
        {
          memcpy(buf, data+2, cp);

          buf     += cp;
          bufsize -= cp;
        }
      } else
      {
        tlv.value = data+2;
      }

      // next item
      (*p_len)   -= fraqlen+2;
      (*pp_data) += fraqlen+2;

    }while( (fraqlen == 255) && (*p_len) );
  }

  return tlv;
}

bool tlv8_encode_next(uint8_t** pp_buf, uint16_t* p_buflen, TLV8_t tlv)
{
  while ( tlv.len )
  {
    uint8_t* buf = *pp_buf;
    uint8_t const fraqlen = (uint8_t) min16(tlv.len, 255);

    buf[0] = tlv.type;
    buf[1] = fraqlen;

    VERIFY( (*p_buflen) >= fraqlen + 2 );
    memcpy(buf+2, tlv.value, fraqlen);

    tlv.len     -= fraqlen;
    tlv.value   += fraqlen;

    (*p_buflen) -= fraqlen+2;
    (*pp_buf)   += fraqlen+2;
  }

  return true;
}
