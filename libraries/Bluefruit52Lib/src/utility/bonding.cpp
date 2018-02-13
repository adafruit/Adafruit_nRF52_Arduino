/**************************************************************************/
/*!
    @file     bonding.cpp
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
#include <Nffs.h>
#include "bonding.h"
#include "bluefruit.h"

#if CFG_DEBUG >= 2
//#define printBondDir()    dbgPrintDir(CFG_BOND_NFFS_DIR)
#define printBondDir()
#else
#define printBondDir()
#endif



/*------------------------------------------------------------------*/
/* Saving Bond Data to Nffs in following layout
 * - _bond_data 80 bytes
 * - Name       32 bytes
 * - CCCD       variable
 *------------------------------------------------------------------*/
static void bond_save_keys_dfr(uint16_t conn_hdl, bond_data_t* bdata)
{
  char filename[BOND_FILENAME_LEN];
  sprintf(filename, BOND_FILENAME, bdata->own_enc.master_id.ediv);

  char devname[CFG_MAX_DEVNAME_LEN] = { 0 };
  Bluefruit.Gap.getPeerName(conn_hdl, devname, CFG_MAX_DEVNAME_LEN);

  NffsFile file(filename, FS_ACCESS_WRITE);

  VERIFY( file.exists(), );

  bool result = true;

  // write keys
  if ( !file.write((uint8_t*)bdata, sizeof(bond_data_t)) )
  {
    result = false;
  }

  // write device name
  if ( strlen(devname) && !file.write((uint8_t*) devname, CFG_MAX_DEVNAME_LEN) )
  {
    result = false;
  }

  file.close();

  if (result)
  {
    LOG_LV2("BOND", "Keys for \"%s\" is saved to file %s", devname, filename);
  }else
  {
    LOG_LV1("BOND", "Failed to save keys for \"%s\"", devname);
  }

  printBondDir();
}

void bond_save_keys(uint16_t conn_hdl, bond_data_t* bdata)
{
  uint8_t* buf = (uint8_t*) rtos_malloc( sizeof(bond_data_t) );
  VERIFY(buf, );

  memcpy(buf, bdata, sizeof(bond_data_t));

  // queue to execute in Ada Callback thread
  ada_callback(buf, bond_save_keys_dfr, conn_hdl, buf);
}

void bond_save_cccd(uint16_t cond_hdl, uint16_t ediv)
{

}

bool bond_load_keys(uint16_t ediv, bond_data_t* bdata)
{

}

bool bond_load_cccd(uint16_t cond_hdl, uint16_t ediv)
{

}
