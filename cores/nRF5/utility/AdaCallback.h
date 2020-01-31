/**************************************************************************/
/*!
    @file     AdaCallback.h
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
#ifndef ADACALLBACK_H_
#define ADACALLBACK_H_

#include "common_inc.h"

#ifndef CFG_CALLBACK_TIMEOUT
#define CFG_CALLBACK_TIMEOUT            100
#endif

#ifdef __cplusplus
extern "C"{
#endif

typedef struct
{
  void*       malloced_data;
  void const* callback_func;

  uint8_t arg_count;
//  uint8_t _reserved[3];

  uint32_t arguments[1]; // flexible array holder
}ada_callback_t;

VERIFY_STATIC( sizeof(ada_callback_t) == 16 );

/*------------- Defer callback type, determined by number of arguments -------------*/
typedef void (*adacb_0arg_t) (void);
typedef void (*adacb_1arg_t) (uint32_t);
typedef void (*adacb_2arg_t) (uint32_t, uint32_t);
typedef void (*adacb_3arg_t) (uint32_t, uint32_t, uint32_t);
typedef void (*adacb_4arg_t) (uint32_t, uint32_t, uint32_t, uint32_t);
typedef void (*adacb_5arg_t) (uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

/*------------------------------------------------------------------*/
/* Macros to force uint32_t casting to all arguments (up to 5)
 *------------------------------------------------------------------*/
#define _ADA_CB_ARGS_0()                  (uint32_t) 0
#define _ADA_CB_ARGS_1(_1)                (uint32_t) _1
#define _ADA_CB_ARGS_2(_1, _2)            _ADA_CB_ARGS_1(_1), (uint32_t) _2
#define _ADA_CB_ARGS_3(_1, _2, _3)        _ADA_CB_ARGS_2(_1, _2), (uint32_t) _3
#define _ADA_CB_ARGS_4(_1, _2, _3, _4)    _ADA_CB_ARGS_3(_1, _2, _3), (uint32_t) _4
#define _ADA_CB_ARGS_5(_1, _2, _3, _4,_5) _ADA_CB_ARGS_4(_1, _2, _3, _4), (uint32_t) _5

#define _GET_6TH_ARG(_0, _1, _2, _3, _4, _5, _6, ... )  _6

#define _ADA_CB_ARGS(...)  _GET_6TH_ARG(_0, ##__VA_ARGS__, _ADA_CB_ARGS_5, _ADA_CB_ARGS_4, _ADA_CB_ARGS_3, _ADA_CB_ARGS_2, _ADA_CB_ARGS_1, _ADA_CB_ARGS_0)(__VA_ARGS__)

/**
 * Schedule an function and parameters to be invoked in Ada Callback Task
 * Macro can take at least 2 and at max 7 arguments
 * - 1st arg     : data pointer that need to be allocated and copied (e.g local variable). NULL if not used
 *                 Ada callback will malloc, copy and free after complete.
 * - 2nd arg     : data pointer length, zero if not used
 * - 3rd arg     : function to be invoked
 * - 3rd-7th arg : function argument, will be cast to uint32_t
 */
#define ada_callback(_malloc_data, _malloc_len, _func, ... ) \
  ({                                                                                          \
      uint8_t const _count = VA_ARGS_NUM(__VA_ARGS__);                                        \
      uint32_t arguments[] = { _ADA_CB_ARGS(__VA_ARGS__) };                                   \
      ada_callback_invoke(_malloc_data, _malloc_len, (void const*) _func, arguments, _count); \
  })

void ada_callback_init(uint32_t stack_sz);
bool ada_callback_invoke(const void* mdata, uint32_t mlen, const void* func, uint32_t arguments[], uint8_t argcount);
void ada_callback_queue(ada_callback_t* cb_item);
bool ada_callback_queue_resize(uint32_t new_depth);

#ifdef __cplusplus
}
#endif

#endif /* ADACALLBACK_H_ */
