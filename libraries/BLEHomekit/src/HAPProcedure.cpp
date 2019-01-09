/**************************************************************************/
/*!
    @file     HAPProcedure.cpp
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

#include <Arduino.h>
#include "HAPUuid.h"
#include "HAPProcedure.h"

/*------------------------------------------------------------------*/
/* Decode
 *------------------------------------------------------------------*/
static uint16_t determine_next_len(uint8_t const* data, uint16_t bufsize)
{
  uint16_t len = 0;
  uint8_t  type = data[0];

  while (bufsize)
  {
    if ( type != data[0] ) break; // different item type

    uint8_t fraqlen = data[1];

    len     += fraqlen;

    // next fragment/item
    bufsize -= minof(fraqlen+2, bufsize); // in case data is corruption or missing.
    data    += fraqlen+2;

    if (fraqlen < 255) break;
  }

  return len;
}

/**
 * Decode a TLV8 item
 *
 * @param pp_data   payload pointer, got update to next item when done
 * @param p_len     len of payload, got updated to remaining item when done
 *
 * @return  TLV8 item.
 *
 * @note If the decoded payload > 255, function will automatically malloced
 * a continuous buffer to assembly data. User should not change the tlv.value
 * pointer and call tlv8_decode_cleanup(tlv) when done processing the tlv.
 */
TLV8_t tlv8_decode_next(uint8_t const** pp_data, uint16_t* p_len)
{
  TLV8_t tlv;
  varclr(&tlv);

  uint8_t const* data = *pp_data;
  uint16_t       dlen = *p_len;

  if ( dlen )
  {
    uint8_t* mbuf = NULL;

    tlv.len = determine_next_len(data, dlen);

    // malloc to assembly fragments in continuous buffer
    if ( tlv.len > 0xff )
    {
      mbuf = (uint8_t*) rtos_malloc( tlv.len );
      VERIFY(mbuf, tlv);

      tlv.value = mbuf;
    }else
    {
      tlv.value = data+2;
    }

    tlv.type = data[0];

    while(dlen)
    {
      // different item type
      if ( tlv.type != data[0] ) break;

      uint8_t fraqlen  = data[1];

      if ( mbuf )
      {
        memcpy(mbuf, data+2, fraqlen);
        mbuf += fraqlen;
      }

      // next fragment/item
      dlen -= minof(fraqlen+2, dlen); // in case data is corruption or missing.
      data += fraqlen+2;

      if ( fraqlen < 0xff ) break;
    }
  }

  // update buffer and len
  *p_len   = dlen;
  *pp_data = data;

  return tlv;
}

uint8_t tlv8_decode_n(uint8_t const* buf, uint16_t bufsize, TLV8_t tlv[], uint8_t count)
{
  uint8_t i;
  for(i=0; i<count && bufsize; i++)
  {
    tlv[i] = tlv8_decode_next(&buf, &bufsize);
  }

  return i;
}

void tlv8_decode_cleanup(TLV8_t tlv)
{
  if (tlv.len > 0xff) rtos_free( (void*) tlv.value);
}

/*------------------------------------------------------------------*/
/* Encode
 *------------------------------------------------------------------*/
uint16_t tlv8_encode_calculate_len(TLV8_t tlv_para[], uint8_t count)
{
  uint16_t total_len = 0;
  for(uint8_t i=0; i <count ; i++)
  {
    total_len += tlv_para[i].len + 2 + (tlv_para[i].len/255)*2;

    if ( tlv_para[i].len % 255 == 0 ) total_len -= 2;
  }

  return total_len;
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

uint16_t tlv8_encode_n(uint8_t* buf, uint16_t bufsize, TLV8_t tlv[], uint8_t count)
{
  uint16_t len = bufsize;

  for(uint8_t i=0; i < count; i++)
  {
    if( !tlv8_encode_next(&buf, &bufsize, tlv[i]) ) break;
  }

  return len - bufsize;
}

