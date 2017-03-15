/**************************************************************************/
/*!
    @file     utilities.c
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
#include "utilities.h"

#include "nrf_sdm.h"
#include "nrf52/nrf_mbr.h"

static lookup_entry_t const sd_lookup_items[] =
{
    { .key = 0x0088, .data = "S132 2.0.1" },
};

static const lookup_table_t sd_lookup_table =
{
    .count = sizeof(sd_lookup_items)/sizeof(lookup_entry_t),
    .items = sd_lookup_items
};

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
 *    "S132 2.0.1, 0.5.0"
 * @return
 */
const char* getFirmwareVersion(void)
{
  static char fw_str[20+1] = { 0 };

  // Skip if already created
  if ( fw_str[0] == 0 )
  {
    uint32_t sd_id = SD_FWID_GET(MBR_SIZE) & 0x0000ffff;
    char const* p_lookup = (char const*) lookup_find(&sd_lookup_table, sd_id);

    if (p_lookup)
    {
      sprintf(fw_str, "%s, %d.%d.%d", p_lookup,
              U32_BYTE2(bootloaderVersion), U32_BYTE3(bootloaderVersion), U32_BYTE4(bootloaderVersion));
    }else
    {
      // Unknown SD ID --> display ID
      sprintf(fw_str, "0x%04X, %d.%d.%d", (uint16_t) sd_id,
              U32_BYTE2(bootloaderVersion), U32_BYTE3(bootloaderVersion), U32_BYTE4(bootloaderVersion));
    }
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

/*
 * nrfjprog --family NRF52 --memrd 0x0000300C
 *
  SoftDevice          |  FWID  | memory address |
  --------------------|--------|----------------
  S110 v5.2.1         | 0x0043 |   0x10001010   |
  S110 v6.0.0         | 0x0049 |   0x10001010   |
  S110 v6.2.1         | 0x0035 |   0x10001010   |
  S110 v7.0.0         | 0x004F |   0x0000300C   |
  S110 v7.1.0         | 0x005A |   0x0000300C   |
  S110 v7.3.0         | 0x0063 |   0x0000300C   |
  S110 v8.0.0         | 0x0064 |   0x0000300C   |
  ----------------------------------------------
  S120 v1.0.0         | 0x0055 |   0x0000300C   |
  S120 v1.0.1         | 0x0058 |   0x0000300C   |
  S120 v2.0.0-1.alpha | 0x005B |   0x0000300C   |
  S120 v2.0.0         | 0x0060 |   0x0000300C   |
  S120 v2.1.0         | 0x006B |   0x0000300C   |
  ----------------------------------------------
  S130 v0.9.0-1.alpha | 0x005E |   0x0000300C   |
  S130 v1.0.0-3.alpha | 0x0066 |   0x0000300C   |
  S130 v1.0.0         | 0x0067 |   0x0000300C   |
  S130 v2.0.0         | 0x0080 |   0x0000300C   |
  S130 v2.0.1         | 0x0087 |   0x0000300C   |
  ----------------------------------------------
  S210 v3.0.0         | 0x004B |   0x10001010   |
  S210 v4.0.0         | 0x0057 |   0x0000300C   |
  ----------------------------------------------
  S310 v1.0.0         | 0x004D |   0x10001010   |
  S310 v2.0.0         | 0x005D |   0x0000300C   |
  S310 v2.0.1         | 0x005D |   0x0000300C   |
  S310 v3.0.0         | 0x0065 |   0x0000300C   |
  ----------------------------------------------
  S132 v1.0.0-3.alpha | 0x006D |       ?        |
  S132 v2.0.0-4.alpha | 0x0074 |       ?        |
  S132 v2.0.0-7.alpha | 0x0079 |   0x0000300C   |
  S132 v2.0.0         | 0x0081 |   0x0000300C   |
  S132 v2.0.1         | 0x0088 |   0x0000300C   |
  ----------------------------------------------
  S212 v0.6.0.alpha   | 0x007F |   0x0000300C   |
  S212 v0.9.1.alpha   | 0x0083 |   0x0000300C   |
  ----------------------------------------------
  S332 v0.6.0.alpha   | 0x007E |   0x0000300C   |
  S332 v0.9.1.alpha   | 0x0082 |   0x0000300C   |
  ----------------------------------------------
  Development/any     | 0xFFFE
  ----------------------------
 */
