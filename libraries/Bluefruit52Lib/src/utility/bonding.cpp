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

#define SVC_CONTEXT_FLAG                 (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)

#if CFG_DEBUG >= 2
#define printBondDir(role)    dbgPrintDir( role == BLE_GAP_ROLE_PERIPH ? BOND_DIR_PRPH : BOND_DIR_CNTR )
#else
#define printBondDir(role)
#endif

/*------------------------------------------------------------------*/
/* Saving Bond Data to Nffs in following layout
 * - _bond_data 80 bytes
 * - Name       32 bytes
 * - CCCD       variable
 *------------------------------------------------------------------*/

void bond_init(void)
{
  // Initialize nffs for bonding (it is safe to call nffs_pkg_init() multiple time)
  Nffs.begin();

  (void) Nffs.mkdir_p(BOND_DIR_PRPH);
  (void) Nffs.mkdir_p(BOND_DIR_CNTR);
}

/*------------------------------------------------------------------*/
/* Keys
 *------------------------------------------------------------------*/
static void bond_save_keys_dfr(uint8_t role, uint16_t conn_hdl, bond_data_t* bdata)
{
  char filename[BOND_FNAME_LEN];
  if ( role == BLE_GAP_ROLE_PERIPH )
  {
    sprintf(filename, BOND_FNAME_PRPH, bdata->own_enc.master_id.ediv);
  }else
  {
    sprintf(filename, BOND_FNAME_CNTR, bdata->own_enc.master_id.ediv);
  }

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

  // If couldn't get devname use peer mac address
  if ( !strlen(devname) )
  {
    uint8_t* mac = bdata->peer_id.id_addr_info.addr;
    sprintf(devname, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  }

  file.write((uint8_t*) devname, CFG_MAX_DEVNAME_LEN);

  file.close();

  if (result)
  {
    LOG_LV2("BOND", "Keys for \"%s\" is saved to file %s", devname, filename);
  }else
  {
    LOG_LV1("BOND", "Failed to save keys for \"%s\"", devname);
  }

  printBondDir(role);
}

void bond_save_keys(uint8_t role, uint16_t conn_hdl, bond_data_t* bdata)
{
  uint8_t* buf = (uint8_t*) rtos_malloc( sizeof(bond_data_t) );
  VERIFY(buf, );

  memcpy(buf, bdata, sizeof(bond_data_t));

  // queue to execute in Ada Callback thread
  ada_callback(buf, bond_save_keys_dfr, role, conn_hdl, buf);
}

bool bond_load_keys(uint8_t role, uint16_t ediv, bond_data_t* bdata)
{
  char filename[BOND_FNAME_LEN];
  if ( role == BLE_GAP_ROLE_PERIPH )
  {
    sprintf(filename, BOND_FNAME_PRPH, ediv);
  }else
  {
    sprintf(filename, BOND_FNAME_CNTR, ediv);
  }

  bool result = (Nffs.readFile(filename, bdata, sizeof(bond_data_t)) > 0);

  if ( result )
  {
    LOG_LV2("BOND", "Load Keys from file %s", filename);
  }else
  {
    LOG_LV1("BOND", "Keys not found");
  }

  return result;
}


/*------------------------------------------------------------------*/
/* CCCD
 *------------------------------------------------------------------*/
static void bond_save_cccd_dfr (uint8_t role, uint16_t conn_hdl, uint16_t ediv)
{
  uint16_t len=0;
  sd_ble_gatts_sys_attr_get(conn_hdl, NULL, &len, SVC_CONTEXT_FLAG);

  uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );
  VERIFY( sys_attr, );

  if ( ERROR_NONE == sd_ble_gatts_sys_attr_get(conn_hdl, sys_attr, &len, SVC_CONTEXT_FLAG) )
  {
    // save to file
    char filename[BOND_FNAME_LEN];
    if ( role == BLE_GAP_ROLE_PERIPH )
    {
      sprintf(filename, BOND_FNAME_PRPH, ediv);
    }else
    {
      sprintf(filename, BOND_FNAME_CNTR, ediv);
    }

    if ( Nffs.writeFile(filename, sys_attr, len, BOND_FILE_CCCD_OFFSET) )
    {
      LOG_LV2("BOND", "CCCD setting is saved to file %s", filename);
    }else
    {
      LOG_LV1("BOND", "Failed to save CCCD setting");
    }

  }

  rtos_free(sys_attr);
  printBondDir(role);
}

void bond_save_cccd(uint8_t role, uint16_t cond_hdl, uint16_t ediv)
{
  VERIFY( ediv != 0xFFFF, );

  // queue to execute in Ada Callback thread
  ada_callback(NULL, bond_save_cccd_dfr, role, cond_hdl, ediv);
}


bool bond_load_cccd(uint8_t role, uint16_t cond_hdl, uint16_t ediv)
{
  bool loaded = false;

  char filename[BOND_FNAME_LEN];
  if ( role == BLE_GAP_ROLE_PERIPH )
  {
    sprintf(filename, BOND_FNAME_PRPH, ediv);
  }else
  {
    sprintf(filename, BOND_FNAME_CNTR, ediv);
  }

  NffsFile file(filename, FS_ACCESS_READ);

  if ( file.exists() )
  {
    int32_t len = file.size() - BOND_FILE_CCCD_OFFSET;

    if ( len )
    {
      uint8_t* sys_attr = (uint8_t*) rtos_malloc( len );

      if (sys_attr)
      {
        file.seek(BOND_FILE_CCCD_OFFSET);

        if ( file.read(sys_attr, len ) )
        {
          if (ERROR_NONE == sd_ble_gatts_sys_attr_set(cond_hdl, sys_attr, len, SVC_CONTEXT_FLAG) )
          {
            loaded = true;

            LOG_LV2("BOND", "Load CCCD from file %s", filename);
          }else
          {
            LOG_LV1("BOND", "CCCD setting not found");
          }
        }

        rtos_free(sys_attr);
      }
    }
  }

  file.close();

  if ( !loaded )
  {
    sd_ble_gatts_sys_attr_set(cond_hdl, NULL, 0, 0);
  }

  return loaded;
}

void bond_print_list(uint8_t role)
{
  const char* dpath = (role == BLE_GAP_ROLE_PERIPH ? BOND_DIR_PRPH : BOND_DIR_CNTR);

  NffsDir dir(dpath);
  NffsDirEntry dirEntry;

  while( dir.read(&dirEntry) )
  {
    if ( !dirEntry.isDirectory() )
    {
      char name[64];
      dirEntry.getName(name, sizeof(name));

      cprintf("  %s : ", name);

      // open file to read device name
      NffsFile file(dpath, dirEntry, FS_ACCESS_READ);

      varclr(name);

      file.seek(BOND_FILE_DEVNAME_OFFSET);
      if ( file.read(name, CFG_MAX_DEVNAME_LEN) )
      {
        cprintf(name);
      }

      cprintf("\n");
      file.close();
    }
  }
  cprintf("\n");
}

/*------------------------------------------------------------------*/
/* DELETE
 *------------------------------------------------------------------*/
void bond_clear_prph(void)
{
  // Detele bonds dir
  Nffs.remove(BOND_DIR_PRPH);

  // Create an empty one
  (void) Nffs.mkdir_p(BOND_DIR_PRPH);
}

void bond_clear_cntr(void)
{
  // Detele bonds dir
  Nffs.remove(BOND_DIR_CNTR);

  // Create an empty one
  (void) Nffs.mkdir_p(BOND_DIR_CNTR);
}


void bond_clear_all(void)
{
  // Detele bonds dir
  Nffs.remove(BOND_DIR_ROOT);

  // Create an empty one for prph and central
  (void) Nffs.mkdir_p(BOND_DIR_PRPH);
  (void) Nffs.mkdir_p(BOND_DIR_CNTR);
}
