/*
 * crypto.h
 *
 *  Created on: Jun 25, 2015
 *      Author: tim
 */

#ifndef HOMEKIT_CRYPTO_H_
#define HOMEKIT_CRYPTO_H_

//#define USE_UNACL_SCALARMULT  1
#define USE_TWEETNACL_SCALARMULT  1

#if defined(USE_UNACL_SCALARMULT)
#include "uNaCl/api.h"
#endif
#include "tweetnacl-modified/tweetnacl.h"
#include "srp/srp.h"
#include "random.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
  struct
  {
    // Public key is the last 32-bytes of the secret key
    union
    {
      uint8_t secret[64];
      struct
      {
        uint8_t __ignore__[32];
        uint8_t pub[32];
      };
    };
  } sign;

  // Client we are paired with
  struct
  {
    uint8_t name[36];
    uint8_t ltpk[32];
  } client;
} crypto_keys_t;

extern crypto_keys_t crypto_keys;


extern void crypto_init(void);
extern void crypto_scheduleStoreKeys(void);
extern void crypto_storeKeys(void);
extern uint8_t crypto_verifyAndDecrypt(const uint8_t* key, uint8_t* nonce, uint8_t* encrypted, uint8_t length, uint8_t* output_buf, uint8_t* mac);
extern void crypto_encryptAndSeal(const uint8_t* key, uint8_t* nonce, uint8_t* plain, uint16_t length, uint8_t* output_buf, uint8_t* output_mac);
extern void crypto_sha512hmac(uint8_t* hash, uint8_t* salt, uint8_t salt_length, uint8_t* data, uint8_t data_length);
extern void crypto_hkdf(uint8_t* target, uint8_t* salt, uint8_t salt_length, uint8_t* info, uint8_t info_length, uint8_t* ikm, uint8_t ikm_length);
extern void crypto_transportEncrypt(uint8_t* key, uint8_t* nonce, uint8_t* plaintext, uint16_t plength, uint8_t* ciphertext, uint16_t* clength);
extern uint8_t crypto_transportDecrypt(uint8_t* key, uint8_t* nonce, uint8_t* ciphertext, uint16_t clength, uint8_t* plaintext, uint16_t* plength);
extern uint8_t crypto_advertise(void);

void crypto_printkey(void);

#ifdef __cplusplus
}
#endif

#endif /* HOMEKIT_CRYPTO_H_ */
