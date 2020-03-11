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
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#include "bonding.h"
#include "bluefruit.h"

using namespace Adafruit_LittleFS_Namespace;

#define BOND_DEBUG        0

#if (CFG_DEBUG == 1 && BOND_DEBUG == 1) || (CFG_DEBUG >= 2)
#define BOND_LOG(...)   LOG_LV1("BOND", __VA_ARGS__)
#else
#define BOND_LOG(...)
#endif

/*------------------------------------------------------------------*/
/* Bond Key is saved in following layout
 * - Bond Data : 80 bytes
 * - Name      : variable (including null char)
 * - CCCD      : variable
 *
 * Each field has an 1-byte preceding length
 *------------------------------------------------------------------*/
#define SVC_CONTEXT_FLAG      (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)
#define BOND_FNAME_LEN        max(sizeof(BOND_FNAME_PRPH), sizeof(BOND_FNAME_CNTR))

static void get_fname (char* fname, uint8_t role, uint16_t ediv)
{
  sprintf(fname, (role == BLE_GAP_ROLE_PERIPH) ? BOND_FNAME_PRPH : BOND_FNAME_CNTR, ediv);
}

static bool bdata_skip_field(File* file)
{
  int len = file->read();
  VERIFY(len > 0);

  file->seek(len + file->position());
  return true;
}

static void bdata_write(File* file, void const* buffer, uint16_t bufsize)
{
  file->write( (uint8_t) bufsize );
  file->write( (uint8_t const*) buffer, bufsize);
}

void bond_init(void)
{
  InternalFS.begin();

  // Create prph and central bond folder if not existed
  if ( !InternalFS.exists(BOND_DIR_PRPH) ) InternalFS.mkdir(BOND_DIR_PRPH);
  if ( !InternalFS.exists(BOND_DIR_CNTR) ) InternalFS.mkdir(BOND_DIR_CNTR);
}

/*------------------------------------------------------------------*/
/* Keys
 *------------------------------------------------------------------*/
static void bond_save_keys_dfr (uint8_t role, uint16_t conn_hdl, bond_keys_t* bkeys)
{
  uint16_t const ediv = (role == BLE_GAP_ROLE_PERIPH) ? bkeys->own_enc.master_id.ediv : bkeys->peer_enc.master_id.ediv;

  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, ediv);

  // delete if file already exists
  if ( InternalFS.exists(filename) ) InternalFS.remove(filename);

  File file(filename, FILE_O_WRITE, InternalFS);
  VERIFY(file,);

  //------------- save keys -------------//
  bdata_write(&file, bkeys, sizeof(bond_keys_t));

  //------------- save device name -------------//
  char devname[CFG_MAX_DEVNAME_LEN] = { 0 };
  Bluefruit.Connection(conn_hdl)->getPeerName(devname, CFG_MAX_DEVNAME_LEN);

  // If couldn't get devname then use peer mac address
  if ( !devname[0] )
  {
    uint8_t* mac = bkeys->peer_id.id_addr_info.addr;
    sprintf(devname, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  }

  bdata_write(&file, devname, strlen(devname)+1); // save also null char

  BOND_LOG("Saved keys for \"%s\" to file %s ( %ld bytes )", devname, filename, file.size());

  file.close();
}

bool bond_save_keys (uint8_t role, uint16_t conn_hdl, bond_keys_t* bkeys)
{
  // queue to execute in Ada Callback thread
  return ada_callback(bkeys, sizeof(bond_keys_t), bond_save_keys_dfr, role, conn_hdl, bkeys);
}

bool bond_load_keys(uint8_t role, uint16_t ediv, bond_keys_t* bkeys)
{
  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, ediv);

  File file(filename, FILE_O_READ, InternalFS);
  VERIFY(file);

  int keylen = file.read();
  VERIFY(keylen == sizeof(bond_keys_t));

  file.read(bkeys, keylen);
  file.close();

  BOND_LOG("Loaded keys from file %s", filename);

  return true;
}


/*------------------------------------------------------------------*/
/* CCCD
 *------------------------------------------------------------------*/
static void bond_save_cccd_dfr (uint8_t role, uint16_t conn_hdl, uint16_t ediv)
{
  uint16_t len=0;
  sd_ble_gatts_sys_attr_get(conn_hdl, NULL, &len, SVC_CONTEXT_FLAG);
  VERIFY(len, );

  uint8_t sys_attr[len];
  VERIFY_STATUS(sd_ble_gatts_sys_attr_get(conn_hdl, sys_attr, &len, SVC_CONTEXT_FLAG),);

  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, ediv);

  File file(filename, FILE_O_WRITE, InternalFS);
  VERIFY(file,);

  file.seek(0); // write mode start at the end, seek to beginning
  bdata_skip_field(&file); // skip key
  bdata_skip_field(&file); // skip name

  bdata_write(&file, sys_attr, len);

  BOND_LOG("Saved CCCD setting to file %s ( offset = %ld, len = %d bytes )", filename, file.size() - (len + 1), len);

  file.close();
}

bool bond_save_cccd (uint8_t role, uint16_t conn_hdl, uint16_t ediv)
{
  VERIFY(ediv != 0xFFFF);

  // queue to execute in Ada Callback thread
  return ada_callback(NULL, 0, bond_save_cccd_dfr, role, conn_hdl, ediv);
}

bool bond_load_cccd(uint8_t role, uint16_t conn_hdl, uint16_t ediv)
{
  bool loaded = false;

  if ( ediv != 0xFFFF )
  {
    char filename[BOND_FNAME_LEN];
    get_fname(filename, role, ediv);

    File file(filename, FILE_O_READ, InternalFS);

    if ( file )
    {
      bdata_skip_field(&file); // skip key
      bdata_skip_field(&file); // skip name

      int len = file.read();
      if ( len > 0 )
      {
        uint8_t sys_attr[len];

        file.read(sys_attr, len);

        if ( ERROR_NONE == sd_ble_gatts_sys_attr_set(conn_hdl, sys_attr, len, SVC_CONTEXT_FLAG) )
        {
          loaded = true;
          BOND_LOG("Loaded CCCD from file %s ( offset = %ld, len = %d bytes )", filename, file.size() - (len + 1), len);
        }
      }
    }

    file.close();
  }

  if ( !loaded )
  {
    LOG_LV1("BOND", "CCCD setting not found");
  }

  return loaded;
}

void bond_print_list(uint8_t role)
{
  char const * dpath = (role == BLE_GAP_ROLE_PERIPH ? BOND_DIR_PRPH : BOND_DIR_CNTR);

  File dir(dpath, FILE_O_READ, InternalFS);
  File file(InternalFS);

  while ( (file = dir.openNextFile(FILE_O_READ)) )
  {
    if ( !file.isDirectory() && bdata_skip_field(&file) ) // skip key
    {
      int len = file.read();
      if ( len > 0 )
      {
        char devname[len];
        file.read(devname, len);

        PRINTF("  %s : %s (%lu bytes)\n", file.name(), devname, file.size());
      }
    }

    file.close();
  }

  PRINTF("\n");

  file.close();
  dir.close();
}


bool bond_find_cntr(ble_gap_addr_t const * addr, bond_keys_t* bkeys)
{
  bool found = false;

  File dir(BOND_DIR_CNTR, FILE_O_READ, InternalFS);
  File file(InternalFS);

  while ( (file = dir.openNextFile(FILE_O_READ)) )
  {
    // Read bond data of each stored file
    int keylen = file.read();
    if ( keylen == sizeof(bond_keys_t) )
    {
      file.read((uint8_t*) bkeys, keylen);

      // Compare static address
      if ( !memcmp(addr->addr, bkeys->peer_id.id_addr_info.addr, 6) )
      {
        found = true;
      }
      else if ( addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE )
      {
        // Resolving private address
      }
    }

    file.close();

    if ( found ) break;
  }

  file.close();
  dir.close();

  return found;
}

/*------------------------------------------------------------------*/
/* DELETE
 *------------------------------------------------------------------*/
void bond_clear_prph(void)
{
  // delete bonds dir
  InternalFS.rmdir_r(BOND_DIR_PRPH);

  // Create an empty one
  InternalFS.mkdir(BOND_DIR_PRPH);
}

void bond_clear_cntr(void)
{
  // delete bonds dir
  InternalFS.rmdir_r(BOND_DIR_CNTR);

  // Create an empty one
  InternalFS.mkdir(BOND_DIR_CNTR);

}

void bond_clear_all(void)
{
  // delete bonds dir
  InternalFS.rmdir_r(BOND_DIR_PRPH);
  InternalFS.rmdir_r(BOND_DIR_CNTR);

  // Create an empty one
  InternalFS.mkdir(BOND_DIR_PRPH);
  InternalFS.mkdir(BOND_DIR_CNTR);
}

void bond_remove_key(uint8_t role, uint16_t ediv)
{
  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, ediv);

  InternalFS.remove(filename);
}
