/**************************************************************************/
/*!
    @file     utilities.c
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
#include "utilities.h"
#include <string.h>
#include <stdio.h>

#include "nrf_sdm.h"
#include "nrf52/nrf_mbr.h"


/******************************************************************************/
/*!
    @brief Find the corresponding data from the key
    @param
*/
/******************************************************************************/
void const * lookup_find(lookup_table_t const* p_table, uint32_t key)
{
  for(uint16_t i=0; i<p_table->count; i++)
  {
    if (p_table->items[i].key == key) return p_table->items[i].data;
  }

  return NULL;
}
/**
 * Format: SDname SDverion, bootloader version
 * e.g
 *    "s132 2.0.1, 0.5.0"
 *    "s132 5.0.0, 5.0.0 single bank"
 *    "s132 6.1.1 r0"
 * @return
 */
const char* getBootloaderVersion(void)
{
  static char fw_str[30+1] = { 0 };

  // Skip if already created
  if ( fw_str[0] == 0 )
  {
    uint32_t const sd_id      = SD_ID_GET(MBR_SIZE);
    uint32_t const sd_version = SD_VERSION_GET(MBR_SIZE);

    uint32_t const ver1 = sd_version / 1000000;
    uint32_t const ver2 = (sd_version % 1000000)/1000;
    uint32_t const ver3 = sd_version % 1000;

    sprintf(fw_str, "s%lu %lu.%lu.%lu", sd_id, ver1, ver2, ver3);
  }

  return fw_str;
}

const char* getMcuUniqueID(void)
{
  static char serial_str[16+1] = { 0 };

  // Skip if already created
  if ( serial_str[0] == 0 )
  {
    sprintf(serial_str, "%08lX%08lX", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);
  }

  return serial_str;
}

