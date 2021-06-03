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

#define BOND_DEBUG        1

#if (CFG_DEBUG == 1 && BOND_DEBUG == 1) || (CFG_DEBUG >= 2)
#define BOND_LOG(...)   LOG_LV1("BOND", __VA_ARGS__)
#else
#define BOND_LOG(...)
#endif

/*------------------------------------------------------------------*/
/* Bond Key is saved in following layout
 * - Keyset : 80 bytes (sizeof(bond_keys_t))
 * - Name   : variable (including null char)
 * - CCCD   : variable
 *
 * Each field has an 1-byte preceding length
 *------------------------------------------------------------------*/
#define SVC_CONTEXT_FLAG      (BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS)
#define BOND_FNAME_LEN        max(sizeof(BOND_FNAME_PRPH), sizeof(BOND_FNAME_CNTR))

static void get_fname (char* fname, uint8_t role, uint8_t const mac[6])
{
  sprintf(fname, (role == BLE_GAP_ROLE_PERIPH) ? BOND_FNAME_PRPH : BOND_FNAME_CNTR,
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
static void bond_save_keys_dfr (uint8_t role, uint16_t conn_hdl, bond_keys_t const * bkeys)
{
  uint8_t const * mac = bkeys->peer_id.id_addr_info.addr;

  // Bond store keys using peer mac address e.g 1a2b3c4d5e6f
  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, mac);

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
    sprintf(devname, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  }

  bdata_write(&file, devname, strlen(devname)+1); // save also null char

  BOND_LOG("Saved keys for \"%s\" to file %s ( %ld bytes )", devname, filename, file.size());

  file.close();
}

bool bond_save_keys (uint8_t role, uint16_t conn_hdl, bond_keys_t const* bkeys)
{
  // queue to execute in Ada Callback thread
  return ada_callback(NULL, 0, bond_save_keys_dfr, role, conn_hdl, bkeys);
}

bool bond_load_keys(uint8_t role, ble_gap_addr_t* addr, bond_keys_t* bkeys)
{
  bool ret = false;

  switch(addr->addr_type)
  {
    case BLE_GAP_ADDR_TYPE_PUBLIC:
    case BLE_GAP_ADDR_TYPE_RANDOM_STATIC:
    {
      // Peer probably uses RANDOM_STATIC or in rarer case PUBLIC
      // Address is used as identity

      char filename[BOND_FNAME_LEN];
      get_fname(filename, role, addr->addr);

      File file(InternalFS);
      if( file.open(filename, FILE_O_READ) )
      {
        int keylen = file.read();
        if ( keylen > 0 )
        {
          file.read((uint8_t*) bkeys, keylen);

          ret = true;
          BOND_LOG("Loaded keys from file %s", filename);
        }
      }

      file.close();
    }
    break;

    case BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE:
    {
      // Resolvable address, we have to go through the whole list to perform IRK Address matching

      char const * dpath = (role == BLE_GAP_ROLE_PERIPH ? BOND_DIR_PRPH : BOND_DIR_CNTR);
      File dir(dpath, FILE_O_READ, InternalFS);
      File file(InternalFS);

      while ( !ret && (file = dir.openNextFile(FILE_O_READ)) )
      {
        int keylen = file.read();
        if ( keylen == sizeof(bond_keys_t) )
        {
          file.read((uint8_t*) bkeys, keylen);
          if ( Bluefruit.Security.resolveAddress(addr, &bkeys->peer_id.id_info) )
          {
            ret = true;
            BOND_LOG("Loaded keys from file %s/%s", dpath, file.name());
          }
        }

        file.close();
      }

      dir.close();
    }
    break;

    default: return false; // Non Resolvable & Anonymous are not supported
  }

  return ret;
}


/*------------------------------------------------------------------*/
/* CCCD
 *------------------------------------------------------------------*/
static void bond_save_cccd_dfr (uint8_t role, uint16_t conn_hdl, ble_gap_addr_t const* id_addr)
{
  uint16_t len=0;
  sd_ble_gatts_sys_attr_get(conn_hdl, NULL, &len, SVC_CONTEXT_FLAG);
  VERIFY(len, );

  uint8_t sys_attr[len];
  VERIFY_STATUS(sd_ble_gatts_sys_attr_get(conn_hdl, sys_attr, &len, SVC_CONTEXT_FLAG),);

  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, id_addr->addr);

  File file(filename, FILE_O_WRITE, InternalFS);
  VERIFY(file,);

  file.seek(0); // write mode start at the end, seek to beginning
  bdata_skip_field(&file); // skip key
  bdata_skip_field(&file); // skip name

  // only write if there is any data changes
  bool do_write = true;

  if ( len == ((uint16_t) file.read()) )
  {
    uint8_t old_data[len];
    file.read(old_data, len);

    if ( 0 == memcmp(sys_attr, old_data, len) )
    {
      do_write = false;
      BOND_LOG("CCCD matches file %s contents, no need to write", filename);
    }
    else
    {
      // restore the position to the state before reading old_data for writing
      file.seek(file.size() - (len + 1));
    }
  }

  if (do_write)
  {
    bdata_write(&file, sys_attr, len);
    BOND_LOG("Saved CCCD to file %s ( offset = %ld, len = %d bytes )", filename, file.size() - (len + 1), len);
  }

  file.close();
}

bool bond_save_cccd (uint8_t role, uint16_t conn_hdl, ble_gap_addr_t const* id_addr)
{
  // queue to execute in Ada Callback thread
  return ada_callback(id_addr, sizeof(ble_gap_addr_t), bond_save_cccd_dfr, role, conn_hdl, id_addr);
}

bool bond_load_cccd(uint8_t role, uint16_t conn_hdl, ble_gap_addr_t const* id_addr)
{
  bool loaded = false;

  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, id_addr->addr);

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

  if ( !loaded ) {
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

void bond_remove_key(uint8_t role, ble_gap_addr_t const* id_addr)
{
  char filename[BOND_FNAME_LEN];
  get_fname(filename, role, id_addr->addr);

  InternalFS.remove(filename);

  BOND_LOG("Removed keys from file %s", filename);
}
