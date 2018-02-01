/*
 * tweetnacl.h
 *
 *  Created on: Jun 21, 2015
 *      Author: tim
 */

#ifndef HOMEKIT_TWEETNACL_TWEETNACL_H_
#define HOMEKIT_TWEETNACL_TWEETNACL_H_

#define crypto_sign_keypair     crypto_sign_ed25519_keypair
#define crypto_sign_open        crypto_sign_ed25519_open
#define crypto_sign             crypto_sign_ed25519
#define crypto_scalarmult_base  crypto_scalarmult_curve25519_base
#define crypto_scalarmult       crypto_scalarmult_curve25519

#ifdef __cplusplus
extern "C"
{
#endif

extern int crypto_hashblocks_sha512(unsigned char *,const unsigned char *,unsigned long long);
extern int crypto_hash_sha512(unsigned char *,const unsigned char *,unsigned long long);

extern int crypto_onetimeauth_poly1305(unsigned char *,const unsigned char *,unsigned long,const unsigned char *);
extern int crypto_onetimeauth_poly1305_verify(const unsigned char *,const unsigned char *,unsigned long,const unsigned char *);

extern int crypto_sign_ed25519(unsigned char *,unsigned long long *,const unsigned char *,unsigned long long,const unsigned char *);
extern int crypto_sign_ed25519_open(unsigned char *,unsigned long long *,const unsigned char *,unsigned long long,const unsigned char *);
extern int crypto_sign_ed25519_keypair(unsigned char *,unsigned char *);

extern int crypto_stream_chacha20_xor(unsigned char *,const unsigned char *,unsigned long long,const unsigned char *,const unsigned char *,const unsigned char);

extern int crypto_verify_16(const unsigned char *,const unsigned char *);
extern int crypto_verify_32(const unsigned char *,const unsigned char *);

#if defined(USE_TWEETNACL_SCALARMULT)
extern int crypto_scalarmult_curve25519(unsigned char *,const unsigned char *,const unsigned char *);
extern int crypto_scalarmult_curve25519_base(unsigned char *,const unsigned char *);
#endif

#ifdef __cplusplus
}
#endif

#endif /* HOMEKIT_TWEETNACL_TWEETNACL_H_ */
