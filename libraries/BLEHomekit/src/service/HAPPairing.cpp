/**************************************************************************/
/*!
    @file     HAPPairing.cpp
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

#include <bluefruit.h>
#include "HAPUuid.h"
#include "HAPPairing.h"
#include "Bluefruit_FileIO.h"

#include "crypto/crypto.h"

#define DEBUG_HAP_PAIRING   0

/* SRP design detail http://srp.stanford.edu/design.html
 */

// kTLV type for pairing
enum {
  PAIRING_TYPE_METHOD = 0     , // 0
  PAIRING_TYPE_IDENTIFIER     , // 1
  PAIRING_TYPE_SALT           , // 2
  PAIRING_TYPE_PUBLIC_KEY     , // 3
  PAIRING_TYPE_PROOF          , // 4
  PAIRING_TYPE_ENCRYPTED_DATA , // 5
  PAIRING_TYPE_STATE          , // 6
  PAIRING_TYPE_ERROR          , // 7
  PAIRING_TYPE_RETRY_DELAY    , // 8
  PAIRING_TYPE_CERTIFICATE    , // 9
  PAIRING_TYPE_SIGNATURE      , // 10
  PAIRING_TYPE_PERMISSIONS    , // 11
  PAIRING_TYPE_FRAGMENT_DATA  , // 12
  PAIRING_TYPE_FRAGMENT_LAST  , // 13
  PAIRING_TYPE_SEPARATOR = 0xff
};

// kTLV value for pairing method
enum
{
  PAIRING_METHOD_SETUP = 1,
  PAIRING_METHOD_VERIFY,
  PAIRING_METHOD_ADD,
  PAIRING_METHOD_REMOVE,
  PAIRING_METHOD_LIST
};

// kTLV value for pairing error
enum
{
  PAIRING_ERROR_UNKNOWN = 1,
  PAIRING_ERROR_AUTHENTICATION,
  PAIRING_ERROR_BACKOFF,
  PAIRING_ERROR_MAX_PEERS,
  PAIRING_ERROR_MAX_TRIES,
  PAIRING_ERROR_UNAVAILABLE,
  PAIRING_ERROR_BUSY
};

#if CFG_DEBUG >= 2
static const char* pairing_method_str[] =
{
    "", "Pair Setup", "Pair Verify", "Add Pairing", "Remove Pairing", "List Pairings"
};

static const char* pairing_type_str[] =
{
    "Method", "Identifier", "Salt", "Public Key", "Proof",
    "Encrypted Data", "State", "Error", "Retry Delay", "Certificate",
    "Signature", "Permissions", "Fragment Data", "Fragment Last",
};

#endif

void _pair_setup_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req);
void _pair_verify_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req);

void gatt_reply_now(uint16_t conn_hdl)
{
  ble_gatts_rw_authorize_reply_params_t reply =
  {
      .type = BLE_GATTS_AUTHORIZE_TYPE_WRITE,
      .params = {
          .write = {
              .gatt_status = BLE_GATT_STATUS_SUCCESS,
              .update      = 1,
          }
      }
  };
  VERIFY_STATUS( sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply), );
}

HAPPairing::HAPPairing(void)
  : HAPService(HAP_UUID_SVC_PAIRING),
    _setup    (HAP_UUID_CHR_PAIR_SETUP   , BLE_GATT_CPF_FORMAT_STRUCT),
    _verify   (HAP_UUID_CHR_PAIR_VERIFY  , BLE_GATT_CPF_FORMAT_STRUCT),
    _features (HAP_UUID_SVC_PAIR_FEATURE , BLE_GATT_CPF_FORMAT_UINT8 ),
    _pairing  (HAP_UUID_SVC_PAIR_PAIRING , BLE_GATT_CPF_FORMAT_STRUCT)
{
  varclr(_pair_id);
}

err_t HAPPairing::begin(void)
{
  VERIFY_STATUS( HAPService::begin() ); // Invoke base class begin()

  _setup.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _setup.setHapWriteCallback(_pair_setup_write_cb);
  _setup.setMaxLen(200); // FIXME change later
  VERIFY_STATUS( _setup.begin() );

  _verify.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _verify.setHapWriteCallback(_pair_verify_write_cb);
  _verify.setMaxLen(200); // FIXME change later
  VERIFY_STATUS( _verify.begin() );

  _features.setHapProperties(HAP_CHR_PROPS_READ);
  VERIFY_STATUS( _features.begin() );
  // Must be 0x00, 0x01 (support HAP pairing) is for MFi license, iOS will reject pairing attempt without MFi
  _features.writeHapValue( (uint32_t) 0x00);

  _pairing.setHapProperties(HAP_CHR_PROPS_SECURE_READ | HAP_CHR_PROPS_SECURE_WRITE);
  VERIFY_STATUS( _pairing.begin() );

  // Make PairID based on MAC address
  uint8_t mac[6];
  Bluefruit.getAddr(mac);
  setDeviceID(mac);

  // Init cryptography
  if ( !InternalFS.exists("/adafruit/homekit") ) InternalFS.mkdir("/adafruit/homekit");

  crypto_init();

  return ERROR_NONE;
}

void HAPPairing::setDeviceID(uint8_t dev_id[6])
{
  char str[sizeof(_pair_id) + 1];
  sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X", dev_id[0], dev_id[1], dev_id[2], dev_id[3], dev_id[4], dev_id[5]);
  memcpy(_pair_id, str, sizeof(_pair_id));
}

void HAPPairing::createSrpResponse(uint16_t conn_hdl, uint8_t status, TLV8_t ktlv[], uint8_t count)
{
  uint16_t srplen = tlv8_encode_calculate_len(ktlv, count);
  uint8_t* srpbuf = (uint8_t*) rtos_malloc(srplen);
  VERIFY( srpbuf != NULL, );

  if( srplen == tlv8_encode_n(srpbuf, srplen, ktlv, count) )
  {
#if DEBUG_HAP_PAIRING
    LOG_LV2_BUFFER("srpbuf", srpbuf, srplen);
#endif

    TLV8_t tlv = { .type = HAP_PARAM_VALUE, .len = srplen, .value = srpbuf };
    _setup.createHapResponse(conn_hdl, status, &tlv, 1);
  }

  rtos_free(srpbuf);
}

/*------------------------------------------------------------------*/
/* PAIR SETUP
 *------------------------------------------------------------------*/
void HAPPairing::pair_setup_m1(uint16_t conn_hdl, HAPRequest_t const* hap_req)
{
  // if paired return PAIRING_ERROR_UNAVAILABLE
  // tries more than 100 time return PAIRING_ERROR_MAX_TRIES
  // pairing with other iOS return PAIRING_ERROR_BUSY

  // step 4
  srp_start();

  // step 5 : username (I) = "Pair-Setup"
  // step 6 : 16 bytes salt already created in srp_init()
  // step 7,8 : password (p = setup code) done in srp_init()
  // step 9 : public key (B) done in srp_init()

  uint8_t mstate = 2;

  TLV8_t tlv_para[] =
  {
      { .type  = PAIRING_TYPE_STATE      , .len = 1  , .value = &mstate       },
      { .type  = PAIRING_TYPE_PUBLIC_KEY , .len = 384, .value = srp_getB()    },
      { .type  = PAIRING_TYPE_SALT       , .len = 16 , .value = srp_getSalt() },
  };

  #if DEBUG_HAP_PAIRING
  LOG_LV2_BUFFER("SRP State"  , tlv_para[0].value, tlv_para[0].len);
  LOG_LV2_BUFFER("SRP Pub Key", tlv_para[1].value, tlv_para[1].len);
  LOG_LV2_BUFFER("SRP Salt"   , tlv_para[2].value, tlv_para[2].len);
  #endif

  createSrpResponse(conn_hdl, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

void HAPPairing::pair_setup_m3(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t pubkey, TLV8_t proof)
{
  uint8_t mstate = 4;
  uint8_t pair_error = PAIRING_ERROR_AUTHENTICATION;

  TLV8_t tlv_para[2] =
  {
      { .type  = PAIRING_TYPE_STATE, .len = 1, .value = &mstate },
      { 0 }
  };

  #if DEBUG_HAP_PAIRING
  LOG_LV2_BUFFER("SRP State", tlv_para[0].value, tlv_para[0].len);
  #endif

  // Set iOS public key
  srp_setA( (uint8_t*) pubkey.value, pubkey.len, gatt_reply_now, conn_hdl);

  // Check proof
  if ( srp_checkM1( (uint8_t*) proof.value, proof.len) )
  {
    tlv_para[1].type  = PAIRING_TYPE_PROOF;
    tlv_para[1].len   = 64;
    tlv_para[1].value = srp_getM2();

    #if DEBUG_HAP_PAIRING
    LOG_LV2_BUFFER("SRP Proof", tlv_para[1].value, tlv_para[1].len);
    #endif
  }else
  {
    tlv_para[1].type  = PAIRING_TYPE_ERROR;
    tlv_para[1].len   = 1;
    tlv_para[1].value = &pair_error;

    #if DEBUG_HAP_PAIRING
    LOG_LV2_BUFFER("SRP Error", tlv_para[1].value, tlv_para[1].len);
    #endif
  }

  createSrpResponse(conn_hdl, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

void HAPPairing::pair_setup_m5(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t encrypted)
{
  uint8_t const pair_error = PAIRING_ERROR_AUTHENTICATION;
  uint8_t const mstate = 6;

  TLV8_t tlv_para[2] =
  {
      { .type  = PAIRING_TYPE_STATE, .len = 1, .value = &mstate },
      { 0 }
  };

  bool failed = false;

  /*------------------------------------------------------------------*/
  /* M5 Verification
   *------------------------------------------------------------------*/
  uint8_t session_key[64];

  // Step 0: Create Session Key
  crypto_hkdf(session_key, (uint8_t*) "Pair-Setup-Encrypt-Salt", 23, (uint8_t*) "Pair-Setup-Encrypt-Info\001", 24, srp_getK(), 64);

  // Step 1 + 2: Verify Auth Tag and Decrypt using ChaCha20-Poly1305
  if (crypto_verifyAndDecrypt(session_key, (uint8_t*) "PS-Msg05", (uint8_t*)encrypted.value, encrypted.len - 16, (uint8_t*) encrypted.value, ((uint8_t*) encrypted.value) + encrypted.len - 16))
  {
    uint16_t blength = encrypted.len - 16;
    uint8_t const* buf = (uint8_t const*) encrypted.value;

    uint8_t* client = NULL;
    uint8_t* signature = NULL;
    uint8_t* ltpk = NULL;

    // Parse iOS's ID, Public Key (ED25519) and Signature
    while(blength)
    {
      TLV8_t tlv = tlv8_decode_next(&buf, &blength);

      LOG_LV2("M5", "type = %s", tlv.type == PAIRING_TYPE_SEPARATOR ? "Separator" : pairing_type_str[tlv.type]);
      LOG_LV2_BUFFER(NULL, tlv.value, tlv.len);

      switch(tlv.type)
      {
        case PAIRING_TYPE_IDENTIFIER:
          VERIFY(tlv.len == 36, );
          client = (uint8_t*) tlv.value;
        break;

        case PAIRING_TYPE_PUBLIC_KEY:
          VERIFY(tlv.len == 32, );
          ltpk = (uint8_t*) tlv.value;
        break;

        case PAIRING_TYPE_SIGNATURE:
          VERIFY(tlv.len == 64, );
          signature = (uint8_t*) tlv.value;
        break;

        default:
          failed = true;
        break;
      }

      // There is no value > 255, there is no need to clean up. Put here for reference only
      // tlv8_decode_cleanup(tlv);
    }

    if (client && signature && ltpk)
    {
      // iOSDeviceInfo = iOSDeviceX + iOSDevicePairingID, iOSDeviceLTPK
      uint8_t message[64 + 32 + 36 + 32];
      memcpy(message, signature, 64);

      // Step 3: Derive iOSDeviceX
      crypto_hkdf(message + 64, (uint8_t*) "Pair-Setup-Controller-Sign-Salt", 31, (uint8_t*) "Pair-Setup-Controller-Sign-Info\001", 32, srp_getK(), 64);

      // Step 4: Construct iOSDeviceInfo
      memcpy(message + 64 + 32, client, 36);
      memcpy(message + 64 + 32 + 36, ltpk, 32);

      uint8_t result[sizeof(message)];
      uint64_t rlen = 0;

      // Step 5: Verify iOSDeviceInfo with ED25519
      if (crypto_sign_open(result, &rlen, message, sizeof(message), ltpk) < 0)
      {
        failed = true;
      }
      else
      {
        // Step 6: save PairingID and LTPK
        memcpy(crypto_keys.client.ltpk, ltpk, sizeof(crypto_keys.client.ltpk));
        memcpy(crypto_keys.client.name, client, 36);
        crypto_scheduleStoreKeys();

        // If failed to save ---> MaxPeers error
      }
    }
  }
  else
  {
    // Error
    failed = true;
  }

  if (failed)
  {
    tlv_para[1].type  = PAIRING_TYPE_ERROR;
    tlv_para[1].len   = 1;
    tlv_para[1].value = &pair_error;

    createSrpResponse(conn_hdl, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));

    return;
  }

  /*------------------------------------------------------------------*/
  /* M6 Response Generation
   *------------------------------------------------------------------*/
  uint8_t smessage[64 + 32 + sizeof(_pair_id) + 32];

  // Step 2: derive AccessoryX using HKDF_SHA512
  crypto_hkdf(smessage + 64, (uint8_t*) "Pair-Setup-Accessory-Sign-Salt", 30, (uint8_t*) "Pair-Setup-Accessory-Sign-Info\001", 31, srp_getK(), 64);

  // Step 3: Concat AccessoryX + PairingID + AccessoryLTPK
  memcpy(smessage + 64 + 32, _pair_id, sizeof(_pair_id));
  memcpy(smessage + 64 + 32 + sizeof(_pair_id), crypto_keys.sign.pub, sizeof(crypto_keys.sign.pub));

  // Step 4: Generate AccessorySignature with Ed25519
  uint64_t slen = 0;
  crypto_sign(smessage, &slen, smessage + 64, sizeof(smessage) - 64, crypto_keys.sign.secret);

  // Step 5: Construct sub-tlv
  TLV8_t ktlv[] =
  {
      { .type  = PAIRING_TYPE_IDENTIFIER, .len = sizeof(_pair_id), .value = _pair_id },
      { .type  = PAIRING_TYPE_PUBLIC_KEY, .len = 32, .value = crypto_keys.sign.pub },
      { .type  = PAIRING_TYPE_SIGNATURE , .len = 64, .value = smessage }
  };

  uint16_t const lbuffer = tlv8_encode_calculate_len(ktlv, arrcount(ktlv));
  uint8_t* buffer = (uint8_t*) rtos_malloc(lbuffer + 16); // Additional 16 byte for Auth Tag
  VERIFY( buffer != NULL, );

  tlv8_encode_n(buffer, lbuffer, ktlv, arrcount(ktlv));

  // Step 6: Encrypt above sub-tlv and generate 16-byte auth tag.
  crypto_encryptAndSeal(session_key, (uint8_t*) "PS-Msg06", buffer, lbuffer, buffer, buffer + lbuffer);

  tlv_para[1].type  = PAIRING_TYPE_ENCRYPTED_DATA;
  tlv_para[1].len   = lbuffer+16;
  tlv_para[1].value = buffer;

  createSrpResponse(conn_hdl, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));

  rtos_free(buffer);
}

void _pair_setup_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req)
{
  uint16_t       body_len  = hap_req->body_len;
  uint8_t const* body_data = hap_req->body_data;

  // Parse TLV data
  while (body_len)
  {
    TLV8_t tlv = tlv8_decode_next(&body_data, &body_len);

    LOG_LV2_BUFFER("Decoded HAP", tlv.value, tlv.len);

    // ignore HAP_PARAM_RETURN_RESP, it should always been 1 for pair setup sequence
    if (HAP_PARAM_VALUE == tlv.type)
    {
      uint8_t  mstate = 0;

      // M1 parameters
      uint8_t  method = 0;

      // M3 parameters
      TLV8_t ipubkey = { 0 }; // (A ) 384 bytes
      TLV8_t iproof  = { 0 }; // (M1) 64 bytes
      TLV8_t iencrypted = { 0 }; // Encrypted data

      /*------------- Parse sub TLV (kTLV) data -------------*/
      uint8_t  const* param_val = (uint8_t  const*) tlv.value;
      uint16_t        param_len = tlv.len;
      while(param_len)
      {
        TLV8_t ktlv = tlv8_decode_next(&param_val, &param_len);

        LOG_LV2("Pair-Setup", "type = %s", ktlv.type == PAIRING_TYPE_SEPARATOR ? "Separator" : pairing_type_str[ktlv.type]);
        LOG_LV2_BUFFER(NULL, ktlv.value, ktlv.len);

        switch (ktlv.type)
        {
          case PAIRING_TYPE_METHOD:
            memcpy(&method, ktlv.value, 1);
            LOG_LV2("HAP", "Method %s", pairing_method_str[method]);
          break;

            // TODO multiple pairing support
          case PAIRING_TYPE_STATE:
            memcpy(&mstate, ktlv.value, 1);
            LOG_LV2("HAP", "State = M%d", mstate);
          break;

          case PAIRING_TYPE_PUBLIC_KEY:
            ipubkey = ktlv;
          break;

          case PAIRING_TYPE_PROOF:
            iproof = ktlv;
          break;

          case PAIRING_TYPE_ENCRYPTED_DATA:
            iencrypted = ktlv;
          break;

            // ignore other type and clean up
          default:
            tlv8_decode_cleanup(ktlv);
          break;
        }

        // Purposely skip tlv8_decode_cleanup() here, will call it later after processing
      }

      /*------------- Processing setup sequence -------------*/
      HAPPairing& svc = (HAPPairing&) chr->parentService();
      switch (mstate)
      {
        case 1:
          svc.pair_setup_m1(conn_hdl, hap_req);
          break;

        case 3:
          svc.pair_setup_m3(conn_hdl, hap_req, ipubkey, iproof);

          tlv8_decode_cleanup(ipubkey);
          tlv8_decode_cleanup(iproof);
          break;

        case 5:

          svc.pair_setup_m5(conn_hdl, hap_req, iencrypted);

          tlv8_decode_cleanup(iencrypted);
          break;

        default: break;
      }
    }

    tlv8_decode_cleanup(tlv);
  }
}

/*------------------------------------------------------------------*/
/* PAIR VERIFY
 *------------------------------------------------------------------*/
void _pair_verify_write_cb (uint16_t conn_hdl, HAPCharacteristic* chr, HAPRequest_t const* hap_req)
{
  uint16_t       body_len  = hap_req->body_len;
  uint8_t const* body_data = hap_req->body_data;

  // Parse TLV data
  while (body_len)
  {
    TLV8_t tlv = tlv8_decode_next(&body_data, &body_len);

    LOG_LV2_BUFFER("Decoded HAP", tlv.value, tlv.len);

    // ignore HAP_PARAM_RETURN_RESP, it should always been 1 for pair setup sequence
    if (HAP_PARAM_VALUE == tlv.type)
    {
      uint8_t mstate = 0;
      TLV8_t  tlvparam = { 0 };

      /*------------- Parse sub TLV (kTLV) data -------------*/
      uint8_t  const* param_val = (uint8_t  const*) tlv.value;
      uint16_t        param_len = tlv.len;
      while(param_len)
      {
        TLV8_t ktlv = tlv8_decode_next(&param_val, &param_len);

        LOG_LV2("Pair-Verify", "type = %s", ktlv.type == PAIRING_TYPE_SEPARATOR ? "Separator" : pairing_type_str[ktlv.type]);
        LOG_LV2_BUFFER(NULL, ktlv.value, ktlv.len);

        switch (ktlv.type)
        {
          case PAIRING_TYPE_STATE:
            memcpy(&mstate, ktlv.value, 1);
            LOG_LV2("HAP", "State = M%d", mstate);
          break;

          case PAIRING_TYPE_PUBLIC_KEY:
            tlvparam = ktlv;
          break;

          case PAIRING_TYPE_ENCRYPTED_DATA:
            tlvparam = ktlv;
          break;

            // ignore other type and clean up
          default:
            tlv8_decode_cleanup(ktlv);
          break;
        }

        // Purposely skip tlv8_decode_cleanup() here, will call it later after processing
      }

      /*------------- Processing VERIFY sequence -------------*/
      HAPPairing& svc = (HAPPairing&) chr->parentService();
      switch (mstate)
      {
        case 1:
          svc.pair_verify_m1(conn_hdl, hap_req, tlvparam);
        break;

        case 3:
          svc.pair_verify_m3(conn_hdl, hap_req, tlvparam);
        break;

        default: break;
      }

      tlv8_decode_cleanup(tlvparam);
    }

    tlv8_decode_cleanup(tlv);
  }
}

void HAPPairing::pair_verify_m1(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t pubkey)
{

}

void HAPPairing::pair_verify_m3(uint16_t conn_hdl, HAPRequest_t const* hap_req, TLV8_t encrypted)
{

}

