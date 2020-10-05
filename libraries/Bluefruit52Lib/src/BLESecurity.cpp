/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "bluefruit.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#ifdef NRF_CRYPTOCELL
  #define LESC_SUPPORTED    1
#else
  #define LESC_SUPPORTED    0
#endif

// default is Just Work
static const ble_gap_sec_params_t _sec_param_default =
{
  .bond         = 1,
  .mitm         = 0,
  .lesc         = LESC_SUPPORTED,
  .keypress     = 0,
  .io_caps      = BLE_GAP_IO_CAPS_NONE,
  .oob          = 0,
  .min_key_size = 7,
  .max_key_size = 16,
  .kdist_own    = { .enc = 1, .id = 1},
  .kdist_peer   = { .enc = 1, .id = 1}
};

//------------- IMPLEMENTATION -------------//

// convert N-byte Number from Big <-> Little Endian to use with BLE
// Public Key = 32-byte N1 + 32-byte N2
static void swap_endian(uint8_t data[], uint32_t nbytes)
{
  for(uint8_t i=0; i<nbytes/2; i++)
  {
    uint8_t const p1 = i;
    uint8_t const p2 = (nbytes-1-i);

    uint8_t temp = data[p1];
    data[p1] = data[p2];
    data[p2] = temp;
  }
}

static void _passkey_display_cabllack_dfr(BLESecurity::pair_passkey_cb_t func, uint16_t conn_hdl, uint8_t const passkey[6], bool match_request)
{
  bool matched = func(conn_hdl, passkey, match_request);

  if (match_request)
  {
    // Match request require to report the match (numberic comparison)
    sd_ble_gap_auth_key_reply(conn_hdl, matched ? BLE_GAP_AUTH_KEY_TYPE_PASSKEY : BLE_GAP_AUTH_KEY_TYPE_NONE, NULL);
  }
}

BLESecurity::BLESecurity(void)
{
  _sec_param = _sec_param_default;
  _passkey_cb = NULL;
  _complete_cb = NULL;
  _secured_cb = NULL;
}

bool BLESecurity::begin(void)
{
#ifdef NRF_CRYPTOCELL
  // Initalize Crypto lib for LESC (safe to call multiple times)
  nRFCrypto.begin();

  // Init Private key
  _private_key.begin(CRYS_ECPKI_DomainID_secp256r1);

  // init public key
  nRFCrypto_ECC_PublicKey pubKey;
  pubKey.begin(CRYS_ECPKI_DomainID_secp256r1);

  // Generate ECC Key pair
  nRFCrypto_ECC::genKeyPair(_private_key, pubKey);

  // Convert to Raw bytes to response to LESC event
  pubKey.toRaw(_pubkey, sizeof(_pubkey));
  pubKey.end();

  // BLE use Little Endian, swap public key endian
  // Public Key = 32-byte N1 + 32-byte N2
  swap_endian(_pubkey+1,    32);
  swap_endian(_pubkey+1+32, 32);
#endif

  return true;
}

void BLESecurity::setIOCaps(bool display, bool yes_no, bool keyboard)
{
  uint8_t io_caps = BLE_GAP_IO_CAPS_NONE;

  if (display)
  {
    if (keyboard)
    {
      io_caps = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
    }
    else if (yes_no)
    {
      io_caps = BLE_GAP_IO_CAPS_DISPLAY_YESNO;
    }else
    {
      io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
    }
  }else
  {
    if (keyboard) io_caps = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
  }

  _sec_param.io_caps = io_caps;
}

void BLESecurity::setMITM(bool enabled)
{
  _sec_param.mitm = (enabled ? 1 : 0);
}

/* Resolvable Address = Hash (24 bit) | Random (24 bit)
 * in which
 *  - Hash = AES(random) using IRK
 *
 * To check if it matches we recreate local AES Hash with IRK to compare with
*/
bool BLESecurity::resolveAddress(ble_gap_addr_t const * p_addr, ble_gap_irk_t const * irk)
{
  VERIFY(p_addr->addr_type == BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE);

  uint8_t const* hash = p_addr->addr;
  uint8_t const* rand = p_addr->addr+3;

  nrf_ecb_hal_data_t ecb_data;
  memclr(&ecb_data, sizeof(nrf_ecb_hal_data_t));

  // Swap Endian for IRK (other padding with 0s)
  memcpy(ecb_data.key, irk->irk, SOC_ECB_KEY_LENGTH);
  swap_endian(ecb_data.key, SOC_ECB_KEY_LENGTH);

  // Swap input endian
  memcpy(ecb_data.cleartext, rand, 3);
  swap_endian(ecb_data.cleartext, SOC_ECB_CLEARTEXT_LENGTH);

  // compute using HW AES peripherals
  (void) sd_ecb_block_encrypt(&ecb_data);

  // Swap output endian
  swap_endian(ecb_data.ciphertext, SOC_ECB_CIPHERTEXT_LENGTH);

  return 0 == memcmp(hash, ecb_data.ciphertext, 3);
}

// Use Legacy SC static Passkey
bool BLESecurity::setPIN(const char* pin)
{
  VERIFY(pin && strlen(pin) == BLE_GAP_PASSKEY_LEN);

  // Static Passkey requires using
  // - Legacy SC
  // - IO cap: Display
  // - MITM is on
  _sec_param.mitm = 1;
  _sec_param.lesc = 0;
  _sec_param.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;

  ble_opt_t opt;
  opt.gap_opt.passkey.p_passkey = (const uint8_t*) pin;
  VERIFY_STATUS(sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt), false);

  return true;
}

// Pairing using LESC with peripheral display
bool BLESecurity::setPairPasskeyCallback(pair_passkey_cb_t fp)
{
  _passkey_cb = fp;

  // mitm is required to trigger passkey generation
  _sec_param.mitm = 1;

  return true;
}

void BLESecurity::setPairCompleteCallback(pair_complete_cb_t fp)
{
  _complete_cb = fp;
}

void BLESecurity::setSecuredCallback(secured_conn_cb_t fp)
{
 _secured_cb = fp;
}

bool BLESecurity::_authenticate(uint16_t conn_hdl)
{
  VERIFY_STATUS(sd_ble_gap_authenticate(conn_hdl, &_sec_param ), false);
  return true;
}

bool BLESecurity::_encrypt(uint16_t conn_hdl, bond_keys_t const* ltkey)
{
  // LESC use own key, Legacy use peer key
  ble_gap_enc_key_t const* enc_key = ltkey->own_enc.enc_info.lesc ? &ltkey->own_enc : &ltkey->peer_enc;
  VERIFY_STATUS(sd_ble_gap_encrypt(conn_hdl, &enc_key->master_id, &enc_key->enc_info), false);
  return true;
}

//--------------------------------------------------------------------+
/* First-time Pairing
 *
 * Connect -> SEC_PARAMS_REQUEST -> PASSKEY_DISPLAY -> BLE_GAP_EVT_LESC_DHKEY_REQUEST ->
 *            CONN_SEC_UPDATE -> AUTH_STATUS
 * 1. Either we or peer initiate the process
 * 2. Peer ask for Secure Parameter ( I/O Caps ) BLE_GAP_EVT_SEC_PARAMS_REQUEST
 * 3. Pair PassKey exchange
 * 4. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
 * 5. Long term Keys exchanged BLE_GAP_EVT_AUTH_STATUS
 *
 * Reconnect using bonded key
 * Connect -> SEC_INFO_REQUEST -> CONN_SEC_UPDATE
 * 1. Either we or peer initiate the process
 * 2. Peer ask for Secure Info ( bond keys ) BLE_GAP_EVT_SEC_INFO_REQUEST
 * 3. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
 */
//--------------------------------------------------------------------+
void BLESecurity::_eventHandler(ble_evt_t* evt)
{
  uint16_t const conn_hdl = evt->evt.common_evt.conn_handle;
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);

  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    {
      /* Pairing is started, Peer is asking for our info
       * Step 1: Pairing/Bonding
       * - Central supplies its parameters
       * - We replies with our security parameters
       */
      ble_gap_sec_params_t const* peer = &evt->evt.gap_evt.params.sec_params_request.peer_params;
      (void) peer;
      LOG_LV2("PAIR", "Peer Params: bond = %d, mitm = %d, lesc = %d, io_caps = %d",
                                    peer->bond, peer->mitm, peer->lesc, peer->io_caps);

      ble_gap_sec_keyset_t keyset =
      {
          .keys_own = {
              .p_enc_key  = &_bond_keys.own_enc,
              .p_id_key   = NULL,
              .p_sign_key = NULL,
              .p_pk       = NULL
          },

          .keys_peer = {
              .p_enc_key  = &_bond_keys.peer_enc,
              .p_id_key   = &_bond_keys.peer_id,
              .p_sign_key = NULL,
              .p_pk       = NULL
          }
      };

      #ifdef NRF_CRYPTOCELL
      keyset.keys_own.p_pk  = (ble_gap_lesc_p256_pk_t*) (_pubkey+1);
      keyset.keys_peer.p_pk = (ble_gap_lesc_p256_pk_t*) (_peer_pubkey+1);
      #endif

      VERIFY_STATUS(sd_ble_gap_sec_params_reply(conn_hdl, BLE_GAP_SEC_STATUS_SUCCESS,
                                                conn->getRole() == BLE_GAP_ROLE_PERIPH ? &_sec_param : NULL, &keyset), );
    }
    break;

    case BLE_GAP_EVT_PASSKEY_DISPLAY:
    {
       ble_gap_evt_passkey_display_t const* passkey_display = &evt->evt.gap_evt.params.passkey_display;
       LOG_LV2("PAIR", "Passkey = %.6s, match request = %d", passkey_display->passkey, passkey_display->match_request);

       // Invoke display callback
       if ( _passkey_cb )
       {
         ada_callback(passkey_display->passkey, 6, _passkey_display_cabllack_dfr, _passkey_cb, conn_hdl, passkey_display->passkey, passkey_display->match_request);
       }
    }
    break;

#ifdef NRF_CRYPTOCELL
    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
    {
      ble_gap_evt_lesc_dhkey_request_t* dhkey_req = &evt->evt.gap_evt.params.lesc_dhkey_request;

      if ( dhkey_req->oobd_req )
      {
        // Out of Band not supported yet
      }

      // Raw public key from peer device is uncompressed
      // _peer_pubkey + 1 == dhkey_req->p_pk_peer->pk
      _peer_pubkey[0] = CRYS_EC_PointUncompressed;

      // Swap Endian data from air
      swap_endian(_peer_pubkey+1   , 32);
      swap_endian(_peer_pubkey+1+32, 32);

      // Create nRFCrypto pubkey from raw format
      nRFCrypto_ECC_PublicKey peerPublickKey;
      peerPublickKey.begin(CRYS_ECPKI_DomainID_secp256r1);
      peerPublickKey.fromRaw(_peer_pubkey, 1+BLE_GAP_LESC_P256_PK_LEN);

      // Create shared secret derivation primitive using ECC Diffie-Hellman
      ble_gap_lesc_dhkey_t dhkey;
      nRFCrypto_ECC::SVDP_DH(_private_key, peerPublickKey, dhkey.key, sizeof(dhkey.key));

      peerPublickKey.end();

      // Swap Endian before sending to air
      swap_endian(dhkey.key, 32);

      // Swap back the peer pubkey since SoftDevice still need this until Authentication is complete
      swap_endian(_peer_pubkey+1   , 32);
      swap_endian(_peer_pubkey+1+32, 32);

      sd_ble_gap_lesc_dhkey_reply(conn_hdl, &dhkey);
    }
    break;
#endif

    // Pairing process completed
    case BLE_GAP_EVT_AUTH_STATUS:
    {
      ble_gap_evt_auth_status_t* status = &evt->evt.gap_evt.params.auth_status;

      LOG_LV2("PAIR", "Auth Status = 0x%02X, Bonded = %d, LESC = %d, Our Kdist = 0x%02X, Peer Kdist = 0x%02X ",
              status->auth_status, status->bonded, status->lesc, *((uint8_t*) &status->kdist_own), *((uint8_t*) &status->kdist_peer));

      // Pairing succeeded --> save encryption keys ( Bonding )
      if (BLE_GAP_SEC_STATUS_SUCCESS == status->auth_status)
      {
        conn->saveBondKey(&_bond_keys);
      }

      // Invoke callback
      if (_complete_cb) ada_callback(NULL, 0, _complete_cb, conn_hdl, status->auth_status);
    }
    break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
    {
      // Peer asks for the stored keys.
      // - load key and return if bonded previously.
      // - Else return NULL --> Initiate key exchange
      ble_gap_evt_sec_info_request_t* sec_info = (ble_gap_evt_sec_info_request_t*) &evt->evt.gap_evt.params.sec_info_request;
      (void) sec_info;

      LOG_LV2("PAIR", "Address: ID = %d, Type = 0x%02X, %02X:%02X:%02X:%02X:%02X:%02X",
              sec_info->peer_addr.addr_id_peer, sec_info->peer_addr.addr_type,
              sec_info->peer_addr.addr[0], sec_info->peer_addr.addr[1], sec_info->peer_addr.addr[2],
              sec_info->peer_addr.addr[3], sec_info->peer_addr.addr[4], sec_info->peer_addr.addr[5]);

      bond_keys_t bkeys;

      if ( conn->loadBondKey(&bkeys) )
      {
        VERIFY_STATUS(sd_ble_gap_sec_info_reply(conn_hdl, &bkeys.own_enc.enc_info, &bkeys.peer_id.id_info, NULL), );
      } else
      {
        VERIFY_STATUS(sd_ble_gap_sec_info_reply(conn_hdl, NULL, NULL, NULL), );
      }
    }
    break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
    {
      const ble_gap_conn_sec_t* conn_sec = &evt->evt.gap_evt.params.conn_sec_update.conn_sec;
      (void) conn_sec;
      LOG_LV2("PAIR", "Security Mode = %d, Level = %d", conn_sec->sec_mode.sm, conn_sec->sec_mode.lv);

      if ( _secured_cb )
      {
        ada_callback(NULL, 0, _secured_cb, conn_hdl);
      }
    }
    break;

    default: break;
  }
}
