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

#include "bluefruit.h"

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
  _hvc_sem = NULL;
  _hvc_received = false;
  _pair_sem = NULL;
  _ediv = 0xFFFF; // invalid ediv value
  _bond_keys = NULL;
}

BLEConnection::~BLEConnection()
{
  vSemaphoreDelete( _hvn_sem );
  vSemaphoreDelete( _wrcmd_sem );

  if ( _hvc_sem ) vSemaphoreDelete( _hvc_sem );
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

bool BLEConnection::requestPairing(void)
{
  // skip if already paired
  if ( _paired ) return true;

  ble_gap_sec_params_t sec_param = Bluefruit.Gap.getSecureParam();

  // on-the-fly semaphore
  _pair_sem = xSemaphoreCreateBinary();

  if ( _role == BLE_GAP_ROLE_PERIPH )
  {
    VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);
    xSemaphoreTake(_pair_sem, portMAX_DELAY);
  }
  else
  {
    uint16_t cntr_ediv = 0xFFFF;
    bond_keys_t bkeys;

    // Check to see if we did bonded with current prph previously
    // TODO currently only matches key using fixed address
    if ( bond_find_cntr(&_addr, &bkeys) )
    {
      cntr_ediv = bkeys.peer_enc.master_id.ediv;
      LOG_LV2("BOND", "Load Keys from file " BOND_FNAME_CNTR, cntr_ediv);
      VERIFY_STATUS( sd_ble_gap_encrypt(_conn_hdl, &bkeys.peer_enc.master_id, &bkeys.peer_enc.enc_info), false);

    }else
    {
      VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);
    }

    xSemaphoreTake(_pair_sem, portMAX_DELAY);

    // Failed to pair using central stored keys, this happens when
    // Prph delete bonds while we did not --> let's remove the obsolete keyfile and retry
    if ( !_paired && (cntr_ediv != 0xffff) )
    {
      bond_remove_key(BLE_GAP_ROLE_CENTRAL, cntr_ediv);

      // Re-try with a fresh session
      VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);

      xSemaphoreTake(_pair_sem, portMAX_DELAY);
    }
  }

  vSemaphoreDelete(_pair_sem);
  _pair_sem = NULL;

  return _paired;
}

bool BLEConnection::waitForIndicateConfirm(void)
{
  // on the fly semaphore
  _hvc_sem = xSemaphoreCreateBinary();

  _hvc_received = false;
  xSemaphoreTake(_hvc_sem, portMAX_DELAY);

  vSemaphoreDelete(_hvc_sem);
  _hvc_sem = NULL;

  return _hvc_received;
}

void BLEConnection::_eventHandler(ble_evt_t* evt)
{
  switch(evt->header.evt_id)
  {
    //--------------------------------------------------------------------+
    /* First-time Pairing
     * 1. Either we or peer initiate the process
     * 2. Peer ask for Secure Parameter ( I/O Caps ) BLE_GAP_EVT_SEC_PARAMS_REQUEST
     * 3. Pair Key exchange ( PIN code)
     * 4. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
     * 5. Long term Keys exchanged BLE_GAP_EVT_AUTH_STATUS
     *
     * Reconnect using bonded key
     * 1. Either we or peer initiate the process
     * 2. Peer ask for Secure Info ( bond keys ) BLE_GAP_EVT_SEC_INFO_REQUEST
     * 3. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
     */
    //--------------------------------------------------------------------+
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    {
      // Pairing in progress, Peer asking for our info
      _bond_keys = (bond_keys_t*) rtos_malloc( sizeof(bond_keys_t));
      VERIFY(_bond_keys, );
      memclr(_bond_keys, sizeof(bond_keys_t));

      _ediv = 0xFFFF; // invalid value for ediv

      /* Step 1: Pairing/Bonding
       * - Central supplies its parameters
       * - We replies with our security parameters
       */
      // ble_gap_sec_params_t* peer = &evt->evt.gap_evt.params.sec_params_request.peer_params;
      COMMENT_OUT(
          // Change security parameter according to authentication type
          if ( _auth_type == BLE_GAP_AUTH_KEY_TYPE_PASSKEY)
          {
            sec_para.mitm    = 1;
            sec_para.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
          }
      )

      ble_gap_sec_keyset_t keyset =
      {
          .keys_own = {
              .p_enc_key  = &_bond_keys->own_enc,
              .p_id_key   = NULL,
              .p_sign_key = NULL,
              .p_pk       = NULL
          },

          .keys_peer = {
              .p_enc_key  = &_bond_keys->peer_enc,
              .p_id_key   = &_bond_keys->peer_id,
              .p_sign_key = NULL,
              .p_pk       = NULL
          }
      };

      ble_gap_sec_params_t sec_param = Bluefruit.Gap.getSecureParam();
      VERIFY_STATUS(sd_ble_gap_sec_params_reply(_conn_hdl,
                                                BLE_GAP_SEC_STATUS_SUCCESS,
                                                _role == BLE_GAP_ROLE_PERIPH ? &sec_param : NULL,
                                                &keyset),
      );
    }
    break;

    case BLE_GAP_EVT_AUTH_STATUS:
    {
      // Pairing process completed
      ble_gap_evt_auth_status_t* status = &evt->evt.gap_evt.params.auth_status;

      // Pairing succeeded --> save encryption keys ( Bonding )
      if (BLE_GAP_SEC_STATUS_SUCCESS == status->auth_status)
      {
        _paired = true;
        _ediv   = _bond_keys->own_enc.master_id.ediv;

        bond_save_keys(_role, _conn_hdl, _bond_keys);
      }else
      {
        PRINT_HEX(status->auth_status);
      }

      rtos_free(_bond_keys);
      _bond_keys = NULL;
    }
    break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
    {
      // Peer asks for the stored keys.
      // - load key and return if bonded previously.
      // - Else return NULL --> Initiate key exchange
      ble_gap_evt_sec_info_request_t* sec_req = (ble_gap_evt_sec_info_request_t*) &evt->evt.gap_evt.params.sec_info_request;

      bond_keys_t bkeys;
      varclr(&bkeys);

      if ( bond_load_keys(_role, sec_req->master_id.ediv, &bkeys) )
      {
        sd_ble_gap_sec_info_reply(_conn_hdl, &bkeys.own_enc.enc_info, &bkeys.peer_id.id_info, NULL);

        _ediv = bkeys.own_enc.master_id.ediv;
      } else
      {
        sd_ble_gap_sec_info_reply(_conn_hdl, NULL, NULL, NULL);
      }
    }
    break;

    case BLE_GAP_EVT_PASSKEY_DISPLAY:
    {
      // ble_gap_evt_passkey_display_t const* passkey_display = &evt->evt.gap_evt.params.passkey_display;
      // PRINT_INT(passkey_display->match_request);
      // PRINT_BUFFER(passkey_display->passkey, 6);

      // sd_ble_gap_auth_key_reply
    }
    break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
    {
      const ble_gap_conn_sec_t* conn_sec = &evt->evt.gap_evt.params.conn_sec_update.conn_sec;

      // Connection is secured (paired) if encryption level > 1
      if ( !( conn_sec->sec_mode.sm == 1 && conn_sec->sec_mode.lv == 1) )
      {
        // Previously bonded --> secure by re-connection process --> Load & Set SysAttr (Apply Service Context)
        // Else Init SysAttr (first bonded)
        if ( !bond_load_cccd(_role, _conn_hdl, _ediv) )
        {
          sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
        }

        _paired = true;
      }

      if (_pair_sem) xSemaphoreGive(_pair_sem);
    }
    break;

    //--------------------------------------------------------------------+
    //
    //--------------------------------------------------------------------+
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      for(uint8_t i=0; i<evt->evt.gatts_evt.params.hvn_tx_complete.count; i++) xSemaphoreGive(_hvn_sem);
    break;

    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      for(uint8_t i=0; i<evt->evt.gattc_evt.params.write_cmd_tx_complete.count; i++) xSemaphoreGive(_wrcmd_sem);
    break;

    case BLE_GATTS_EVT_HVC:
    {
      LOG_LV2("GATTS", "Confirm received handle = 0x%04X", evt->evt.gatts_evt.params.hvc.handle);

      _hvc_received = true;
      if ( _hvc_sem ) xSemaphoreGive(_hvc_sem);
    }
    break;

    case BLE_GATTS_EVT_TIMEOUT:
    {
      uint8_t timeout_src = evt->evt.gatts_evt.params.timeout.src;
      LOG_LV2("GATTS", "Timeout Source = %d", timeout_src);

      if (BLE_GATT_TIMEOUT_SRC_PROTOCOL == timeout_src)
      {
        _hvc_received = false;
        if ( _hvc_sem ) xSemaphoreGive(_hvc_sem);
      }
    }
    break;

    default: break;
  }
}
