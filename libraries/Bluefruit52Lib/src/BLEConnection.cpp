/**************************************************************************/
/*!
    @file     BLEConnection.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019, Adafruit Industries (adafruit.com)
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

#include "BLEConnection.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+


BLEConnection::BLEConnection(uint16_t conn_hdl, ble_gap_evt_connected_t const* evt_connected, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _conn_hdl = conn_hdl;

  _mtu = BLE_GATT_ATT_MTU_DEFAULT;
  _addr = evt_connected->peer_addr;
  _role = evt_connected->role;

  _hvn_sem   = xSemaphoreCreateCounting(hvn_qsize, hvn_qsize);
  _wrcmd_sem = xSemaphoreCreateCounting(wrcmd_qsize, wrcmd_qsize);

  _paired = false;
  hvc_sem = NULL;
  hvc_received = false;
  pair_sem = NULL;
  ediv = 0xFFFF; // invalid ediv value
  bond_keys = NULL;
}

BLEConnection::~BLEConnection()
{
  vSemaphoreDelete( _hvn_sem );
  vSemaphoreDelete( _wrcmd_sem );
}

void BLEConnection::giveHvnPacket(uint8_t count)
{
  for(uint8_t i=0; i<count; i++) xSemaphoreGive(_hvn_sem);
}

void BLEConnection::giveWriteCmdPacket(uint8_t count)
{
  for(uint8_t i=0; i<count; i++) xSemaphoreGive(_wrcmd_sem);
}

uint16_t BLEConnection::handle (void)
{
  return _conn_hdl;
}

bool BLEConnection::paired (void)
{
  return _paired;
}

uint8_t BLEConnection::getRole (void)
{
  return _role;
}

uint16_t BLEConnection::getMTU (void)
{
  return _mtu;
}

void BLEConnection::setMTU (uint16_t mtu)
{
  _mtu = mtu;
}

ble_gap_addr_t BLEConnection::getPeerAddr (void)
{
  return _addr;
}

uint8_t BLEConnection::getPeerAddr (uint8_t addr[6])
{
  memcpy(addr, _addr.addr, BLE_GAP_ADDR_LEN);
  return _addr.addr_type;
}

bool BLEConnection::getHvnPacket (void)
{
  return xSemaphoreTake(_hvn_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

bool BLEConnection::getWriteCmdPacket (void)
{
  return xSemaphoreTake(_wrcmd_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}
