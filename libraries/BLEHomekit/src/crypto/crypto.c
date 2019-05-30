/*
 * crypto.c
 *
 *  Created on: Jun 25, 2015
 *      Author: tim
 */

#include <stdint.h>
#include <string.h>

//#include <nordic_common.h>
//#include <app_error.h>
//#include <pstorage.h>
//#include <app_scheduler.h>
#include <nrf_soc.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "utility/debug.h"

#include "rtos.h"
#include "utility/AdaCallback.h"

#include "crypto.h"

using namespace Adafruit_LittleFS_Namespace;

#define CRYPTO_INSTANCE  2   // Change this to force key regeneration on next run

crypto_keys_t crypto_keys;

static const uint8_t zeros64[64];
//static pstorage_handle_t crypto_store_handle;
//static volatile uint8_t crypto_storing;
static uint8_t crypto_loadKeys(void);
//static void crypto_pstorage_callback(pstorage_handle_t *p_handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len);

#define CRYPTO_KEYDIR    "/adafruit/homekit"
#define CRYPTO_KEYFILE   CRYPTO_KEYDIR "/key"

typedef struct
{
  uint8_t valid0;
  uint8_t instance;

  // Local keys
  uint8_t srp_b[32];
  uint8_t srp_salt[16];
  uint8_t srp_v[384];
  uint8_t srp_B[384];
  uint8_t sign_secret[64];

  // Client keys
  uint8_t clientname[36];
  uint8_t ltpk[32];

  uint8_t valid1;
  uint8_t __padding__[1];
} crypto_persistent_keys_t;

void crypto_printkey(void)
{
  crypto_persistent_keys_t keys = {};

  uint32_t keylen = sizeof(keys);
  fsutil_read_file(CRYPTO_KEYFILE, 0, keylen, &keys, &keylen);

  if (keys.valid0 == 0x55 && keys.valid1 == 0xAA && keys.instance == CRYPTO_INSTANCE)
  {
      dbgDumpMemoryCFormat("uint8_t keys[] = ", &keys, sizeof(keys));
  }
}

void crypto_init(void)
{
//  uint32_t err_code;
//
//  static const pstorage_module_param_t param =
//  {
//    .cb = crypto_pstorage_callback,
//    .block_size = sizeof(crypto_persistent_keys_t),
//    .block_count = 1
//  };
//  err_code = pstorage_register((pstorage_module_param_t*)&param, &crypto_store_handle);
//  APP_ERROR_CHECK(err_code);

  if (!crypto_loadKeys())
  {
    srp_init();

    crypto_sign_keypair(crypto_keys.sign.pub, crypto_keys.sign.secret);

    // Store for reuse
//    crypto_scheduleStoreKeys();
    crypto_storeKeys();
  }
}

static uint8_t crypto_loadKeys(void)
{
//  uint32_t err_code;
//
//  pstorage_handle_t handle;
//  err_code = pstorage_block_identifier_get(&crypto_store_handle, 0, &handle);
//  APP_ERROR_CHECK(err_code);

//  crypto_persistent_keys_t keys = {};
//  err_code = pstorage_load((uint8_t*)&keys, &handle, sizeof(keys), 0);
//  APP_ERROR_CHECK(err_code);

  crypto_persistent_keys_t keys = {};

  uint32_t keylen = sizeof(keys);

  File file(CRYPTO_KEYFILE, FILE_O_READ, InternalFS);
  VERIFY(file, 0);

  keylen = file.read(&keys, keylen);

  file.close();

  if (keys.valid0 == 0x55 && keys.valid1 == 0xAA && keys.instance == CRYPTO_INSTANCE)
  {
    // Valid
    memcpy(srp.b, keys.srp_b, 32);
    memcpy(srp.salt, keys.srp_salt, 16);
    memcpy(srp.v, keys.srp_v, 384);
    memcpy(srp.B, keys.srp_B, 384);
    memcpy(crypto_keys.sign.secret, keys.sign_secret, 64);
    memcpy(crypto_keys.client.name, keys.clientname, 36);
    memcpy(crypto_keys.client.ltpk, keys.ltpk, 32);
    return 1;
  }
  else
  {
    return 0;
  }
}

void crypto_scheduleStoreKeys(void)
{
//    crypto_storing = 1;
  ada_callback(NULL, 0, crypto_storeKeys);
}

void crypto_storeKeys(void)
{
//  uint32_t err_code;

//  if (crypto_storing)
  {
//    pstorage_handle_t handle;
//    err_code = pstorage_block_identifier_get(&crypto_store_handle, 0, &handle);
//    APP_ERROR_CHECK(err_code);

    crypto_persistent_keys_t keys = {};
    memcpy(keys.srp_b, srp.b, 32);
    memcpy(keys.srp_salt, srp.salt, 16);
    memcpy(keys.srp_v, srp.v, 384);
    memcpy(keys.srp_B, srp.B, 384);
    memcpy(keys.sign_secret, crypto_keys.sign.secret, 64);
    memcpy(keys.clientname, crypto_keys.client.name, 36);
    memcpy(keys.ltpk, crypto_keys.client.ltpk, 32);

    keys.instance = CRYPTO_INSTANCE;
    keys.valid0 = 0x55;
    keys.valid1 = 0xAA;

    File file(CRYPTO_KEYFILE, FILE_O_WRITE, InternalFS);
    VERIFY(file,);

    file.write(&keys, sizeof(keys));

    file.close();

//    err_code = pstorage_update(&handle, (uint8_t*)&keys, sizeof(keys), 0);
//    APP_ERROR_CHECK(err_code);
//
//    // Pump events until the store is done
//    while (crypto_storing)
//    {
//      err_code = sd_app_evt_wait();
//      APP_ERROR_CHECK(err_code);
//      app_sched_execute();
//    }
  }
}

//static void crypto_pstorage_callback(pstorage_handle_t *p_handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len)
//{
//  switch (op_code)
//  {
//  case PSTORAGE_UPDATE_OP_CODE:
//    crypto_storing = 0;
//    break;
//
//  default:
//    break;
//  }
//}

uint8_t crypto_advertise(void)
{
  // If we have a client name, we don't advertise, otherwise we do.
  if (crypto_keys.client.name[0])
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

uint8_t crypto_verifyAndDecrypt(const uint8_t* key, uint8_t* nonce, uint8_t* encrypted, uint8_t length, uint8_t* output_buf, uint8_t* mac)
{
  uint8_t polykey[sizeof(zeros64)];
  crypto_stream_chacha20_xor(polykey, zeros64, sizeof(zeros64), nonce, key, 0);

  uint8_t padding = (16 - length % 16) % 16;
  uint8_t message[length + padding + 16];
  memcpy(message, encrypted, length);
  memset(message + length, 0, padding + 16);
  message[length + padding + 8] = (uint8_t)length;
  message[length + padding + 9] = (uint8_t)(length >> 8);

  if (crypto_onetimeauth_poly1305_verify(mac, message, sizeof(message), polykey) != 0)
  {
    // Fail
    return 0;
  }
  else
  {
    crypto_stream_chacha20_xor(output_buf, message, length, nonce, key, 1);
    return 1;
  }
}

void crypto_encryptAndSeal(const uint8_t* key, uint8_t* nonce, uint8_t* plain, uint16_t length, uint8_t* output_buf, uint8_t* output_mac)
{
  uint8_t polykey[sizeof(zeros64)];
  crypto_stream_chacha20_xor(polykey, zeros64, sizeof(zeros64), nonce, key, 0);

  uint8_t padding = (16 - length % 16) % 16;
  uint8_t message[length + padding + 16];

  crypto_stream_chacha20_xor(message, plain, length, nonce, key, 1);
  memset(message + length, 0, padding + 16);
  message[length + padding + 8] = (uint8_t)length;
  message[length + padding + 9] = (uint8_t)(length >> 8);

  crypto_onetimeauth_poly1305(output_mac, message, sizeof(message), polykey);

  memcpy(output_buf, message, length);
}

void crypto_sha512hmac(uint8_t* hash, uint8_t* salt, uint8_t salt_length, uint8_t* data, uint8_t data_length)
{
  uint8_t message1[128 + data_length];
  uint8_t message2[128 + 64];

  memset(message1, 0x36, 128);
  memset(message2, 0x5C, 128);
  for (unsigned i = salt_length; i--; )
  {
    message1[i] = 0x36 ^ salt[i];
    message2[i] = 0x5C ^ salt[i];
  }
  memcpy(message1 + 128, data, data_length);
  crypto_hash_sha512(message2 + 128, message1, sizeof(message1));
  crypto_hash_sha512(hash, message2, sizeof(message2));
}

void crypto_hkdf(uint8_t* target, uint8_t* salt, uint8_t salt_length, uint8_t* info, uint8_t info_length, uint8_t* ikm, uint8_t ikm_length)
{
  crypto_sha512hmac(target, salt, salt_length, ikm, ikm_length);
  crypto_sha512hmac(target, target, 64, info, info_length);
}

void crypto_transportEncrypt(uint8_t* key, uint8_t* nonce, uint8_t* plaintext, uint16_t plength, uint8_t* ciphertext, uint16_t* clength)
{
  crypto_encryptAndSeal(key, nonce, plaintext, plength, ciphertext, ciphertext + plength);
  for (unsigned i = 0; i < 8 && !++nonce[i]; i++)
    ;
  *clength = plength + 16;
}

uint8_t crypto_transportDecrypt(uint8_t* key, uint8_t* nonce, uint8_t* ciphertext, uint16_t clength, uint8_t* plaintext, uint16_t* plength)
{
  uint8_t r = crypto_verifyAndDecrypt(key, nonce, ciphertext, clength - 16, plaintext, ciphertext + clength - 16);
  for (unsigned i = 0; i < 8 && !++nonce[i]; i++)
    ;
  *plength = clength - 16;
  return r;
}
