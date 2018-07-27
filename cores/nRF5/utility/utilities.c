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

static lookup_entry_t const sd_lookup_items[] =
{
    // S132
    { .key = 0x0088, .data = "S132 2.0.1" },
    { .key = 0x00A5, .data = "S132 5.1.0" },
    { .key = 0x00A8, .data = "S132 6.0.0" },
    // S140
    { .key = 0x00A9, .data = "S140 6.0.0" },
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
 *    "S132 5.0.0, 5.0.0 single bank"
 *    "S132 5.0.0, 5.0.0 dual banks"
 * @return
 */
const char* getFirmwareVersion(void)
{
  static char fw_str[30+1] = { 0 };

  // Skip if already created
  if ( fw_str[0] == 0 )
  {
    uint32_t sd_id = SD_FWID_GET(MBR_SIZE) & 0x0000ffff;
    char const* p_lookup = (char const*) lookup_find(&sd_lookup_table, sd_id);

    if (p_lookup)
    {
      sprintf(fw_str, "%s, %d.%d.%d %s", p_lookup,
              U32_BYTE1(bootloaderVersion), U32_BYTE2(bootloaderVersion), U32_BYTE3(bootloaderVersion),
              U32_BYTE4(bootloaderVersion) ? "single bank" : "dual banks");
    }else
    {
      // Unknown SD ID --> display ID
      sprintf(fw_str, "0x%04X, %d.%d.%d %s", (uint16_t) sd_id,
              U32_BYTE1(bootloaderVersion), U32_BYTE2(bootloaderVersion), U32_BYTE3(bootloaderVersion),
              U32_BYTE4(bootloaderVersion) ? "single bank" : "dual banks");
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
  S132 v2.0.1         | 0x0088 |   0x0000300C   |
  S132 v5.0.0         | 0x009D |                |
  Development/any     | 0xFFFE |                |
  ----------------------------
 */
