/*
 * srp.c
 *
 * SRP algorithm (see http://srp.stanford.edu)
 * Uses bits of the ARM mbed TSL library (https://tls.mbed.org).
 *  Notes: You can probably do better here since N and G are known at compile time.
 *         This library uses a massive amount of RAM during calculations.
 *         The SRP handshake takes 30+ seconds.
 *
 *  Created on: Jun 10, 2015
 *      Author: tim
 */

#include <stdint.h>
#include <string.h>

//#include <app_error.h>

//#include "homekit/homekit-config.h"
#include "bignum.h"
#include "memory_buffer_alloc.h"
#include "../random.h"
#include "srp.h"
#include "../tweetnacl-modified/tweetnacl.h"
//#include "homekit/pairing.h"

#include "common_inc.h"
#include "rtos.h"
#include "nrf_error.h"

#define E(V)  (((V) << 24) | ((V) >> 24) | (((V) >> 8) & 0x0000FF00) | (((V) << 8) & 0x00FF0000))
static const uint32_t srp_N[] =
{
    E(0xFFFFFFFF), E(0xFFFFFFFF), E(0xC90FDAA2), E(0x2168C234), E(0xC4C6628B), E(0x80DC1CD1), E(0x29024E08),
    E(0x8A67CC74), E(0x020BBEA6), E(0x3B139B22), E(0x514A0879), E(0x8E3404DD), E(0xEF9519B3), E(0xCD3A431B),
    E(0x302B0A6D), E(0xF25F1437), E(0x4FE1356D), E(0x6D51C245), E(0xE485B576), E(0x625E7EC6), E(0xF44C42E9),
    E(0xA637ED6B), E(0x0BFF5CB6), E(0xF406B7ED), E(0xEE386BFB), E(0x5A899FA5), E(0xAE9F2411), E(0x7C4B1FE6),
    E(0x49286651), E(0xECE45B3D), E(0xC2007CB8), E(0xA163BF05), E(0x98DA4836), E(0x1C55D39A), E(0x69163FA8),
    E(0xFD24CF5F), E(0x83655D23), E(0xDCA3AD96), E(0x1C62F356), E(0x208552BB), E(0x9ED52907), E(0x7096966D),
    E(0x670C354E), E(0x4ABC9804), E(0xF1746C08), E(0xCA18217C), E(0x32905E46), E(0x2E36CE3B), E(0xE39E772C),
    E(0x180E8603), E(0x9B2783A2), E(0xEC07A28F), E(0xB5C55DF0), E(0x6F4C52C9), E(0xDE2BCBF6), E(0x95581718),
    E(0x3995497C), E(0xEA956AE5), E(0x15D22618), E(0x98FA0510), E(0x15728E5A), E(0x8AAAC42D), E(0xAD33170D),
    E(0x04507A33), E(0xA85521AB), E(0xDF1CBA64), E(0xECFB8504), E(0x58DBEF0A), E(0x8AEA7157), E(0x5D060C7D),
    E(0xB3970F85), E(0xA6E1E4C7), E(0xABF5AE8C), E(0xDB0933D7), E(0x1E8C94E0), E(0x4A25619D), E(0xCEE3D226),
    E(0x1AD2EE6B), E(0xF12FFA06), E(0xD98A0864), E(0xD8760273), E(0x3EC86A64), E(0x521F2B18), E(0x177B200C),
    E(0xBBE11757), E(0x7A615D6C), E(0x770988C0), E(0xBAD946E2), E(0x08E24FA0), E(0x74E5AB31), E(0x43DB5BFC),
    E(0xE0FD108E), E(0x4B82D120), E(0xA93AD2CA), E(0xFFFFFFFF), E(0xFFFFFFFF)
};
#undef E
static const uint32_t srp_N_sizeof = 384;
static const uint8_t srp_G = 5;
// Pre-compute hashes where we can k = H(N | PAD(g))
static const uint8_t srp_N_G_hash[] =
{
    0xA9, 0xC2, 0xE2, 0x55, 0x9B, 0xF0, 0xEB, 0xB5, 0x3F, 0x0C, 0xBB, 0xF6, 0x22, 0x82, 0x90, 0x6B,
    0xED, 0xE7, 0xF2, 0x18, 0x2F, 0x00, 0x67, 0x82, 0x11, 0xFB, 0xD5, 0xBD, 0xE5, 0xB2, 0x85, 0x03,
    0x3A, 0x49, 0x93, 0x50, 0x3B, 0x87, 0x39, 0x7F, 0x9B, 0xE5, 0xEC, 0x02, 0x08, 0x0F, 0xED, 0xBC,
    0x08, 0x35, 0x58, 0x7A, 0xD0, 0x39, 0x06, 0x08, 0x79, 0xB8, 0x62, 0x1E, 0x8C, 0x36, 0x59, 0xE0
};
static const uint8_t srp_N_hash_srp_G_hash[] =
{
    0xB3, 0xD6, 0x3E, 0xF6, 0xAA, 0xFB, 0x2E, 0x97, 0x96, 0xDA, 0xE0, 0x06, 0xFE, 0x60, 0xF2, 0x0E,
    0x53, 0x26, 0xFE, 0x1E, 0x1C, 0x52, 0xA5, 0x02, 0x1E, 0xB2, 0x17, 0x47, 0xCA, 0x33, 0xDF, 0xEA,
    0x8F, 0xF2, 0x03, 0xBD, 0x45, 0xA2, 0x54, 0x43, 0x95, 0xD7, 0x3D, 0x19, 0x89, 0x9C, 0x17, 0x4A,
    0x68, 0x3C, 0x9E, 0x78, 0x32, 0x96, 0xDD, 0x16, 0x93, 0xEB, 0xC7, 0x1C, 0xF5, 0xA5, 0x3D, 0xA3
};


#if TEST_APPLE

#define HOMEKIT_CONFIG_PINCODE  "password123"
#define HOMEKIT_USERNAME        "alice"

static const uint8_t pincode[17] = HOMEKIT_USERNAME ":" HOMEKIT_CONFIG_PINCODE;

const uint8_t dbg_salt[] =
{
    0xBE, 0xB2, 0x53, 0x79, 0xD1, 0xA8, 0x58, 0x1E, 0xB5, 0xA7, 0x27, 0x67, 0x3A, 0x24, 0x41, 0xEE
};

const uint8_t dbg_b[] =
{
    0xE4, 0x87, 0xCB, 0x59, 0xD3, 0x1A, 0xC5, 0x50, 0x47, 0x1E, 0x81, 0xF0, 0x0F, 0x69, 0x28, 0xE0,
    0x1D, 0xDA, 0x08, 0xE9, 0x74, 0xA0, 0x04, 0xF4, 0x9E, 0x61, 0xF5, 0xD1, 0x05, 0x28, 0x4D, 0x20
};

#else

#define HOMEKIT_CONFIG_PINCODE  "111-22-333"
#define HOMEKIT_USERNAME        "Pair-Setup"
static const uint8_t pincode[21] = HOMEKIT_USERNAME ":" HOMEKIT_CONFIG_PINCODE;

#if DEBUG_SRP
const uint8_t dbg_salt[] =
{
    0x6E, 0xFC, 0xCB, 0x4A, 0x0D, 0x20, 0xDA, 0x55,
    0x57, 0xE9, 0xDF, 0xD5, 0x44, 0x2B, 0x59, 0x48
};

const uint8_t dbg_b[] =
{
    0xCB, 0x13, 0xF7, 0xDB, 0x5A, 0x8A, 0xF0, 0xFB,
    0xE3, 0xB7, 0x85, 0x89, 0xEA, 0x89, 0xC5, 0x3C,
    0x61, 0x10, 0x21, 0x0B, 0x6F, 0xE5, 0x31, 0xD4,
    0xCE, 0x7D, 0xF6, 0x72, 0x59, 0x18, 0x95, 0xA1
};
#endif
#endif

#if DEBUG_SRP
void print_mpi(const char* str, mpi* m)
{
  uint32_t len = mpi_size(m) ;
  uint8_t* buf = (uint8_t*) rtos_malloc ( len );
  VERIFY(buf, );

  mpi_write_binary(m, buf, len);
  LOG_LV2_BUFFER(str, buf, len);

  rtos_free(buf);
}
#endif

srp_keys_t srp;

//static void MPI_ERROR_CHECK(int CODE)
//{
//  if (CODE != 0)
//  {
//    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
//  }
//}

#define MPI_ERROR_CHECK(_code)     VERIFY_STATUS(_code, )

/**
 * Adafruit: re-calculate x if p changes (setup code)
 */
void srp_init(void)
{
  int err_code;

  // The MPI library uses a ridiculous amount of memory. We use the stack allocator
  // so we don't tie this memory up except when we absolutely need to.
//  uint8_t memory[11 * 1024];
//  memory_buffer_alloc_init(memory, sizeof(memory));

#if DEBUG_SRP
  memcpy(srp.salt, dbg_salt, sizeof(srp.salt));
  memcpy(srp.b, dbg_b, 32);
#else
  // Generate salt
  random_create(srp.salt, sizeof(srp.salt));

  // Generate 'b' - a random value
  random_create(srp.b, 32);
#endif

  // Calculate 'x' = H(s | H(I | ":" | P))
  mpi x;
  mpi_init(&x);
  {
    uint8_t message[sizeof(srp.salt) + 64];
    memcpy(message, srp.salt, sizeof(srp.salt));
    crypto_hash_sha512(message + sizeof(srp.salt), pincode, sizeof(pincode));
    crypto_hash_sha512(message, message, sizeof(message));
    err_code = mpi_read_binary(&x, message, 64);
    MPI_ERROR_CHECK(err_code);
  }

  // Calculate 'v' = g ^ x mod N
  mpi g;
  mpi_init(&g);
  err_code = mpi_lset(&g, srp_G);
  MPI_ERROR_CHECK(err_code);

  mpi n;
  mpi_init(&n);
  err_code = mpi_read_binary(&n, (uint8_t*)srp_N, srp_N_sizeof);
  MPI_ERROR_CHECK(err_code);

  mpi tmp;
  mpi_init(&tmp);
  mpi v;
  mpi_init(&v);
  err_code = mpi_exp_mod(&v, &g, &x, &n, &tmp);
  MPI_ERROR_CHECK(err_code);

  // Calculate 'k'
  mpi k;
  mpi_init(&k);
  err_code = mpi_read_binary(&k, srp_N_G_hash, sizeof(srp_N_G_hash));
  MPI_ERROR_CHECK(err_code);

  // Calculate 'B' = k*v + g^b % N
  mpi B;
  mpi_init(&B);
  err_code = mpi_mul_mpi(&B, &k, &v);
  MPI_ERROR_CHECK(err_code);

  err_code = mpi_write_binary(&v, srp.v, sizeof(srp.v));
  MPI_ERROR_CHECK(err_code);
  mpi_free(&v);

  mpi b;
  mpi_init(&b);
  err_code = mpi_read_binary(&b, srp.b, sizeof(srp.b));
  MPI_ERROR_CHECK(err_code);

  err_code = mpi_exp_mod(&x, &g, &b, &n, &tmp);
  MPI_ERROR_CHECK(err_code);
  err_code = mpi_add_abs(&B, &B, &x);
  MPI_ERROR_CHECK(err_code);
  err_code = mpi_mod_mpi(&B, &B, &n);
  MPI_ERROR_CHECK(err_code);
  err_code = mpi_write_binary(&B, srp.B, sizeof(srp.B));

  mpi_free(&b);
  mpi_free(&B);
  mpi_free(&k);
  mpi_free(&tmp);
  mpi_free(&n);
  mpi_free(&g);
  mpi_free(&x);

//  memory_buffer_alloc_free();
}

void srp_start(void)
{
  srp.clientM1 = 0;
  srp.serverM1 = 0;
}

uint8_t srp_setA(uint8_t* abuf, uint16_t length, moretime_t moretime, uint16_t conn_hdl)
{
  int err_code;

  // The MPI library uses a ridiculous amount of memory. We use the stack allocator
  // so we don't tie this memory up except when we absolutely need to.
//  uint8_t memory[11 * 1024];
//  memory_buffer_alloc_init(memory, sizeof(memory));

  // getK
  {
    mpi s;
    mpi_init(&s);

    mpi n;
    mpi_init(&n);
    err_code = mpi_read_binary(&n, (uint8_t*)srp_N, srp_N_sizeof);
    MPI_ERROR_CHECK(err_code);

    {
      // u = H(A | B)
      mpi u;
      mpi_init(&u);
      {
        uint8_t message[length + sizeof(srp.B)];
        memcpy(message, abuf, length);
        memcpy(message + length, srp.B, sizeof(srp.B));
        crypto_hash_sha512(message, message, sizeof(message));

        err_code = mpi_read_binary(&u, message, 64);
        MPI_ERROR_CHECK(err_code);
      }

      mpi v;
      mpi_init(&v);
      err_code = mpi_read_binary(&v, srp.v, sizeof(srp.v));

      // getS = (A * v^u mod N)^b mod N
      err_code = mpi_exp_mod(&s, &v, &u, &n, NULL);
      MPI_ERROR_CHECK(err_code);

      mpi_free(&v);
      mpi_free(&u);
    }

    // These calculations take a long time. To avoid the connection dying we ask for more time now.
    if (moretime)
    {
      moretime(conn_hdl);
    }

    {
      mpi a;
      mpi_init(&a);
      err_code = mpi_read_binary(&a, abuf, length);
      MPI_ERROR_CHECK(err_code);

      err_code = mpi_mul_mpi(&s, &s, &a);
      MPI_ERROR_CHECK(err_code);
      mpi_free(&a);

      mpi b;
      mpi_init(&b);
      err_code = mpi_read_binary(&b, srp.b, sizeof(srp.b));

      err_code = mpi_exp_mod(&s, &s, &b, &n, NULL);
      MPI_ERROR_CHECK(err_code);

      mpi_free(&b);
    }

    mpi_free(&n);

#if DEBUG_SRP > 1
//    print_mpi("S", &s);
#endif

    uint8_t sbuf[384];
    err_code = mpi_write_binary(&s, sbuf, sizeof(sbuf));
    MPI_ERROR_CHECK(err_code);

    mpi_free(&s);

    crypto_hash_sha512(srp.K, sbuf, sizeof(sbuf));

#if DEBUG_SRP > 1
    LOG_LV2_BUFFER("K=H(S)", srp.K, sizeof(srp.K));
#endif
  }

//  memory_buffer_alloc_free();

  // getM1 - username s abuf srp.B K
  {
    uint8_t message[sizeof(srp_N_hash_srp_G_hash) + 64 + sizeof(srp.salt) + length + 384 + sizeof(srp.K)];
    memcpy(message, srp_N_hash_srp_G_hash, sizeof(srp_N_hash_srp_G_hash));
    crypto_hash_sha512(message + sizeof(srp_N_hash_srp_G_hash), pincode, strlen(HOMEKIT_USERNAME)); // First 10 chars only - not the PIN part
    memcpy(message + sizeof(srp_N_hash_srp_G_hash) + 64, srp.salt, sizeof(srp.salt));
    memcpy(message + sizeof(srp_N_hash_srp_G_hash) + 64 + sizeof(srp.salt), abuf, length);
    memcpy(message + sizeof(srp_N_hash_srp_G_hash) + 64 + sizeof(srp.salt) + length, srp.B, sizeof(srp.B));
    memcpy(message + sizeof(srp_N_hash_srp_G_hash) + 64 + sizeof(srp.salt) + length + 384, srp.K, sizeof(srp.K));
    srp.serverM1 = 1;
    if (srp.clientM1)
    {
      uint8_t hash[64];
      crypto_hash_sha512(hash, message, sizeof(message));
      if (memcmp(hash, srp.M1, sizeof(srp.M1)) != 0)
      {
        return 0;
      }
    }
    else
    {
      crypto_hash_sha512(srp.M1, message, sizeof(message));
    }
  }

#if DEBUG_SRP > 1
  LOG_LV2_BUFFER("M1", srp.M1, sizeof(srp.M1));
#endif

  // getM2
  {
    uint8_t message[length + sizeof(srp.M1) + sizeof(srp.K)];
    memcpy(message, abuf, length);
    memcpy(message + length, srp.M1, sizeof(srp.M1));
    memcpy(message + length + sizeof(srp.M1), srp.K, sizeof(srp.K));
    crypto_hash_sha512(srp.M2, message, sizeof(message));
  }

#if DEBUG_SRP > 1
  LOG_LV2_BUFFER("M2", srp.M2, sizeof(srp.M2));
#endif


  return 1;
}

uint8_t* srp_getSalt(void)
{
  return srp.salt;
}

uint8_t* srp_getB(void)
{
  return srp.B;
}

uint8_t srp_checkM1(uint8_t* m1, uint16_t length)
{
  srp.clientM1 = 1;
  if (length != sizeof(srp.M1))
  {
    return 0;
  }
  if (srp.serverM1)
  {
    if (memcmp(srp.M1, m1, sizeof(srp.M1)) != 0)
    {
      return 0;
    }
  }
  else
  {
    memcpy(srp.M1, m1, sizeof(srp.M1));
  }
  return 1;
}

uint8_t* srp_getM2(void)
{
  return srp.M2;
}

uint8_t* srp_getK(void)
{
  return srp.K;
}
