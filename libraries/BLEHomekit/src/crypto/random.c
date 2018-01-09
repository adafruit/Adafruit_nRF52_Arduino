/*
 * random.c
 *
 *  Created on: Jun 10, 2015
 *      Author: tim
 */

//#include <softdevice_handler.h>

#include "nrf_soc.h"
#include "random.h"

#define APP_ERROR_CHECK(_err)   if (_err != NRF_SUCCESS)  return

void random_create(uint8_t* p_result, uint8_t length)
{
  uint32_t err_code;

  while (length)
  {
    uint8_t available = 0;
    err_code = sd_rand_application_bytes_available_get(&available);
    APP_ERROR_CHECK(err_code);
    if (available)
    {
      available = available < length ? available : length;
      err_code = sd_rand_application_vector_get(p_result, available);
      APP_ERROR_CHECK(err_code);
      p_result += available;
      length -= available;
    }
  }
}

void randombytes(uint8_t* p_result, uint64_t length)
{
  random_create(p_result, (uint8_t)length);
}
