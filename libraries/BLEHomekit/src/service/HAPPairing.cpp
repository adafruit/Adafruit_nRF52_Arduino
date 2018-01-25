/**************************************************************************/
/*!
    @file     HAPPairing.cpp
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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
#include <Nffs.h>

#include "crypto/crypto.h"

#define DEBUG_HAP_PAIRING   0

/*
The following is a description of SRP-6 and 6a, the latest versions of SRP:

  N    A large safe prime (N = 2q+1, where q is prime)
       All arithmetic is done modulo N.
  g    A generator modulo N
  k    Multiplier parameter (k = H(N, g) in SRP-6a, k = 3 for legacy SRP-6)
  s    User's salt
  I    Username
  p    Cleartext Password
  H()  One-way hash function
  ^    (Modular) Exponentiation
  u    Random scrambling parameter
  a,b  Secret ephemeral values
  A,B  Public ephemeral values
  x    Private key (derived from p and s)
  v    Password verifier

The host stores passwords using the following formula:
  x = H(s, p)               (s is chosen randomly)
  v = g^x                   (computes password verifier)
The host then keeps {I, s, v} in its password database. The authentication protocol itself goes as follows:
User -> Host:  I, A = g^a                  (identifies self, a = random number)
Host -> User:  s, B = kv + g^b             (sends salt, b = random number)

        Both:  u = H(A, B)

        User:  x = H(s, p)                 (user enters password)
        User:  S = (B - kg^x) ^ (a + ux)   (computes session key)
        User:  K = H(S)

        Host:  S = (Av^u) ^ b              (computes session key)
        Host:  K = H(S)
Now the two parties have a shared, strong session key K. To complete authentication, they need to prove to each other that their keys match. One possible way:
User -> Host:  M = H(H(N) xor H(g), H(I), s, A, B, K)
Host -> User:  H(A, M, K)
The two parties also employ the following safeguards:
The user will abort if he receives B == 0 (mod N) or u == 0.
The host will abort if it detects that A == 0 (mod N).
The user must show his proof of K first. If the server detects that the user's proof is incorrect, it must abort without showing its own proof of K.
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

HAPResponse_t* _pairing_setup_write_cb (HAPCharacteristic* chr, HAPRequest_t const* hap_req);

HAPPairing::HAPPairing(void)
  : HAPService(HAP_UUID_SVC_PAIRING),
    _setup    (HAP_UUID_CHR_PAIR_SETUP   , BLE_GATT_CPF_FORMAT_STRUCT),
    _verify   (HAP_UUID_CHR_PAIR_VERIFY  , BLE_GATT_CPF_FORMAT_STRUCT),
    _features (HAP_UUID_SVC_PAIR_FEATURE , BLE_GATT_CPF_FORMAT_UINT8 ),
    _pairing  (HAP_UUID_SVC_PAIR_PAIRING , BLE_GATT_CPF_FORMAT_STRUCT)
{

}

err_t HAPPairing::begin(void)
{
  VERIFY_STATUS( HAPService::begin() ); // Invoke base class begin()

  _setup.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _setup.setHapWriteCallback(_pairing_setup_write_cb);
  _setup.setMaxLen(BLE_GATTS_VAR_ATTR_LEN_MAX);
  VERIFY_STATUS( _setup.begin() );

  _verify.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _verify.setMaxLen(100/*BLE_GATTS_VAR_ATTR_LEN_MAX*/); // FIXME change later
  VERIFY_STATUS( _verify.begin() );

  _features.setHapProperties(HAP_CHR_PROPS_READ);
  VERIFY_STATUS( _features.begin() );
  _features.writeHapValue(0x01); // support HAP pairing

  _pairing.setHapProperties(HAP_CHR_PROPS_SECURE_READ | HAP_CHR_PROPS_SECURE_WRITE);
  VERIFY_STATUS( _pairing.begin() );

  // Init cryptography
  Nffs.mkdir_p("/adafruit/homekit");
  crypto_init();

  return ERROR_NONE;
}

static HAPResponse_t* createSrpResponse(uint8_t tid, uint8_t status, TLV8_t ktlv[], uint8_t count)
{
  HAPResponse_t* hap_resp = NULL;

  uint16_t srplen = tlv8_encode_calculate_len(ktlv, count);
  uint8_t* srpbuf = (uint8_t*) rtos_malloc(srplen);
  VERIFY( srpbuf != NULL, NULL );

  if( srplen == tlv8_encode_n(srpbuf, srplen, ktlv, count) )
  {
    #if DEBUG_HAP_PAIRING
    LOG_LV2_BUFFER("M1 Response", srpbuf, srplen);
    #endif

    TLV8_t tlv = { .type = HAP_PARAM_VALUE, .len = srplen, .value = srpbuf };
    hap_resp = createHapResponse(tid, status, &tlv, 1);
  }

  rtos_free(srpbuf);
  return hap_resp;
}

HAPResponse_t* HAPPairing::pair_setup_m1(HAPRequest_t const* hap_req)
{
  // if paired return PAIRING_ERROR_UNAVAILABLE
  // tries more than 100 time return PAIRING_ERROR_MAX_TRIES
  // pairing with other iOS return PAIRING_ERROR_BUSY

  // step 4
  srp_start();

  // step 5 : username (I = "Pair-Setup"
  // step 6 : 16 bytes salt already created in srp_init()
  // step 7,8 : password (p = setup code) done in srp_init()
  // step 9 : public key (B) done in srp_init()

  uint8_t mstate = 2;

  TLV8_t tlv_para[] =
  {
      { .type  = PAIRING_TYPE_STATE      , .len = 1  , .value = &mstate       },
      { .type  = PAIRING_TYPE_SALT       , .len = 16 , .value = srp_getSalt() },
      { .type  = PAIRING_TYPE_PUBLIC_KEY , .len = 384, .value = srp_getB()    },
  };

  #if DEBUG_HAP_PAIRING
  LOG_LV2_BUFFER("SRP State"  , tlv_para[0].value, tlv_para[0].len);
  LOG_LV2_BUFFER("SRP Salt"   , tlv_para[1].value, tlv_para[1].len);
  LOG_LV2_BUFFER("SRP Pub Key", tlv_para[2].value, tlv_para[2].len);
  #endif

  return createSrpResponse(hap_req->header.tid, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

HAPResponse_t* HAPPairing::pair_setup_m3(HAPRequest_t const* hap_req, TLV8_t pubkey, TLV8_t proof)
{
  uint8_t mstate = 4;
  uint8_t pair_error = PAIRING_ERROR_AUTHENTICATION;

  TLV8_t tlv_para[2] =
  {
      { .type  = PAIRING_TYPE_STATE, .len = 1, .value = &mstate },
  };

  // Set iOS public key
  srp_setA( (uint8_t*) pubkey.value, pubkey.len, NULL);

  // Check proof
  if ( srp_checkM1( (uint8_t*) proof.value, proof.len) )
  {
    tlv_para[1].type  = PAIRING_TYPE_PROOF;
    tlv_para[1].len   = 64;
    tlv_para[1].value = srp_getM2();
  }else
  {
    tlv_para[1].type  = PAIRING_TYPE_ERROR;
    tlv_para[1].len   = 1;
    tlv_para[1].value = &pair_error;

    LOG_LV2("SRP", "M3 proof check failed");
  }

  #if DEBUG_HAP_PAIRING || 1
  LOG_LV2_BUFFER("SRP State", tlv_para[0].value, tlv_para[0].len);
  LOG_LV2_BUFFER("SRP Proof/Error", tlv_para[1].value, tlv_para[1].len);
  #endif

  return createSrpResponse(hap_req->header.tid, HAP_STATUS_SUCCESS, tlv_para, arrcount(tlv_para));
}

HAPResponse_t* _pairing_setup_write_cb (HAPCharacteristic* chr, HAPRequest_t const* hap_req)
{
  HAPResponse_t* hap_resp = NULL;

  uint16_t       body_len  = hap_req->body_len;
  uint8_t const* body_data = hap_req->body_data;

  bool is_write_resp = false;

  // Parse TLV data
  while (body_len)
  {
    TLV8_t tlv = tlv8_decode_next(&body_data, &body_len);

    LOG_LV2_BUFFER("Decoded HAP", tlv.value, tlv.len);

    switch (tlv.type)
    {
      case HAP_PARAM_RETURN_RESP:
        memcpy(&is_write_resp, tlv.value, 1);
      break;

      case HAP_PARAM_VALUE:
      {
        uint8_t  mstate = 0;

        // M1 parameters
        uint8_t  method = 0;

        // M3 parameters
        TLV8_t ipubkey = { 0 }; // (A ) 384 bytes
        TLV8_t iproof  = { 0 }; // (M1) 64 bytes

        // Parse sub TLV (kTLV) data
        uint8_t  const* param_val = (uint8_t  const*) tlv.value;
        uint16_t        param_len = tlv.len;
        while(param_len)
        {
          TLV8_t ktlv = tlv8_decode_next(&param_val, &param_len);

          LOG_LV2("PAIRSETUP", "type = %s", ktlv.type == PAIRING_TYPE_SEPARATOR ? "Separator" : pairing_type_str[ktlv.type]);
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

            // ignore other type and clean up
            default:
              tlv8_decode_cleanup(ktlv);
            break;
          }

          // Purposely skip tlv8_decode_cleanup() --> need to manually
          // free all the pointer > 255 after done processing
        }

        HAPPairing& svc = (HAPPairing&) chr->parentService();
        switch (mstate)
        {
          case 1:
            hap_resp = svc.pair_setup_m1(hap_req);
          break;

          case 3:
            hap_resp = svc.pair_setup_m3(hap_req, ipubkey, iproof);

            tlv8_decode_cleanup(ipubkey);
            tlv8_decode_cleanup(iproof);
          break;

          default: break;
        }
      }
      break;

      // ignore other type
      default: break;
    }

    tlv8_decode_cleanup(tlv);
  }

  return hap_resp;
}
