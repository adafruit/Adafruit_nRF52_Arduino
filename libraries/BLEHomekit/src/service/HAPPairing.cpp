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

#include "crypto/crypto.h"

// kTLV type for pairing
enum {
  PAIRING_TYPE_METHOD = 0,
  PAIRING_TYPE_IDENTIFIER,
  PAIRING_TYPE_SALT,
  PAIRING_TYPE_PUBLIC_KEY,
  PAIRING_TYPE_PROOF,
  PAIRING_TYPE_ENCRYPTED_DATA,
  PAIRING_TYPE_STATE,
  PAIRING_TYPE_ERROR,
  PAIRING_TYPE_RETRY_DELAY,
  PAIRING_TYPE_CERTIFICATE,
  PAIRING_TYPE_SIGNATURE,
  PAIRING_TYPE_PERMISSIONS,
  PAIRING_TYPE_FRAGMENT_DATA,
  PAIRING_TYPE_FRAGMENT_LAST,
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

// kTLV value for state
enum
{
  PAIRING_STATE_M1 = 1,
  PAIRING_STATE_M2,
  PAIRING_STATE_M3,
  PAIRING_STATE_M4,
};

#if CFG_DEBUG >= 2
static const char* pairing_method_str[] =
{
    "", "Pair Setup", "Pair Verify", "Add Pairing", "Remove Pairing", "List Pairings"
};

#endif

static HAPResponse_t* pairing_setup_write_cb (HAPCharacteristic* chr, ble_gatts_evt_write_t const* gatt_req, HAPRequest_t const* hap_req);

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
  _setup.setHapWriteCallback(pairing_setup_write_cb);
  _setup.setMaxLen(BLE_GATTS_VAR_ATTR_LEN_MAX);
  VERIFY_STATUS( _setup.begin() );

  _verify.setHapProperties(HAP_CHR_PROPS_READ | HAP_CHR_PROPS_WRITE);
  _verify.setMaxLen(BLE_GATTS_VAR_ATTR_LEN_MAX);
  VERIFY_STATUS( _verify.begin() );

  _features.setHapProperties(HAP_CHR_PROPS_READ);
  VERIFY_STATUS( _features.begin() );
  _features.writeHapValue(0x01); // support HAP pairing

  _pairing.setHapProperties(HAP_CHR_PROPS_SECURE_READ | HAP_CHR_PROPS_SECURE_WRITE);
  VERIFY_STATUS( _pairing.begin() );

  // Init cryptography
  crypto_init();

  return ERROR_NONE;
}


static HAPResponse_t* pairing_setup_write_cb (HAPCharacteristic* chr, ble_gatts_evt_write_t const* gatt_req, HAPRequest_t const* hap_req)
{
  VERIFY(gatt_req->len > 3, NULL);

  HAPResponse_t* hap_resp = NULL;

  uint16_t       body_len  = hap_req->body_len;
  uint8_t const* body_data = hap_req->body_data;

  uint8_t  const* param_val = NULL;
  uint16_t        param_len = 0;

  bool is_write_resp = false;

  // TODO HAP Fragmentation

  // Parse TLV data
  while (body_len)
  {
    TLV8_t tlv = tlv8_decode_next(&body_data, &body_len);

    switch (tlv.type)
    {
      case HAP_PARAM_VALUE:
        param_val = (uint8_t  const*) tlv.value;
        param_len = (uint16_t) tlv.len;
      break;

      case HAP_PARAM_RETURN_RESP:
        memcpy(&is_write_resp, tlv.value, 1);
      break;

      // ignore other type
      default: break;
    }
  }

  // Parse sub TLV (kTLV) data
  while(param_len)
  {
    TLV8_t ktlv = tlv8_decode_next(&param_val, &param_len);

    switch (ktlv.type)
    {
      case PAIRING_TYPE_METHOD:
        LOG_LV2("HAP", "Method %s", pairing_method_str[ *((uint8_t const*)ktlv.value) ]);
      break;

      case PAIRING_TYPE_STATE:
      {
        uint8_t state = *((uint8_t const*)ktlv.value);
        LOG_LV2("HAP", "State = M%d", state);
        switch (state)
        {
          case 1: // M1
            // if paired return PAIRING_ERROR_UNAVAILABLE
            // tries more than 100 time return PAIRING_ERROR_MAX_TRIES
            // pairing with other iOS return PAIRING_ERROR_BUSY

          break;

        }
      }
      break;

      // ignore other type
      default: break;
    }
  }

  return hap_resp;
}
