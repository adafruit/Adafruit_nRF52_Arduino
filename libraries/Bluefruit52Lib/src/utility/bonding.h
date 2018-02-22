/**************************************************************************/
/*!
    @file     bonding.h
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
#ifndef BONDING_H_
#define BONDING_H_

#include "bluefruit_common.h"

#define CFG_BOND_NFFS_DIR                "/adafruit/bond"
#define BOND_FILENAME                    CFG_BOND_NFFS_DIR "/%04x"
#define BOND_FILENAME_LEN                (sizeof(CFG_BOND_NFFS_DIR) + 10)

// Shared keys with bonded device, size = 80 bytes
typedef struct
{
  ble_gap_enc_key_t own_enc;
  ble_gap_enc_key_t peer_enc;
  ble_gap_id_key_t  peer_id;
} bond_data_t;

enum
{
  BOND_FILE_DEVNAME_OFFSET = sizeof(bond_data_t),
  BOND_FILE_CCCD_OFFSET    = BOND_FILE_DEVNAME_OFFSET + CFG_MAX_DEVNAME_LEN
};

void bond_init(void);
void bond_clear(void);

void bond_save_keys(uint16_t conn_hdl, bond_data_t* bdata);
bool bond_load_keys(uint16_t ediv, bond_data_t* bdata);

void bond_save_cccd(uint16_t cond_hdl, uint16_t ediv);
bool bond_load_cccd(uint16_t cond_hdl, uint16_t ediv);



#endif /* BONDING_H_ */
