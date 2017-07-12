/**************************************************************************/
/*!
    @file     AdaCallback.h
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
#ifndef ADACALLBACK_H_
#define ADACALLBACK_H_

#include <Arduino.h>
#include "bluefruit.h"

#define CFG_CALLBACK_TASK_STACKSIZE     (512*2)
#define CFG_CALLBACK_QUEUE_LENGTH       10

typedef struct
{
  void*   malloced_data;
  void*   callback_func;

  uint8_t callback_type;
  uint8_t arg_count;
//  uint8_t _reserved[2];

  uint32_t arguments[1]; // flexible array holder
}ada_callback_t;

static_assert(sizeof(ada_callback_t) == 16, "Incorrect Size");


/*------------------------------------------------------------------*/
/* X Macros expansion for callback function
 * If callbacks from separated classes share the same prototype, only
 * one instance is enough (mandatory)
 *------------------------------------------------------------------*/
#define ADA_CB_LOOKUP(XPAND)  \
    /* BLEGap  */                                  \
    XPAND(BLEGap         , connect_callback_t    ) \
    XPAND(BLEGap         , disconnect_callback_t ) \
    XPAND(BLEAdvertising , stop_callback_t       ) \
    /*BLEScanner::stop_callback_t is same as BLEAdvertising::stop_callback_t */\
    XPAND(BLEScanner     , rx_callback_t         ) \
    /* Bluefruit  */                               \
    /* Central */                                  \
    /* Client Characteristic */                    \
    XPAND(BLEClientCharacteristic  , notify_cb_t   ) \
    /*XPAND(BLEClientCharacteristic , indicate_cb_t)*/ \

#define ADA_CB_ENUM_XPAND(_class, _cbname) \
    _class##_##_cbname,


#define ADA_CB_FUNC_XPAND(_class, _cbname) \
    static inline uint8_t ada_callback_type(_class::_cbname _func) { (void) _func; return _class##_##_cbname; }

enum
{
  ADA_CB_LOOKUP(ADA_CB_ENUM_XPAND)
};

ADA_CB_LOOKUP(ADA_CB_FUNC_XPAND)

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
 * Macro function is called by other module with all intended parameters.
 * The first parameter is malloced Pointer (NULL if not), so that callback could know to free memory
 */
#define ada_callback(_malloced, _func , ... ) \
    do { \
      uint8_t const _count = VA_ARGS_NUM(__VA_ARGS__);\
      ada_callback_t* cb_data = (ada_callback_t*) rtos_malloc( sizeof(ada_callback_t) + (_count ? (_count-1)*4 : 0) ); \
      cb_data->malloced_data = _malloced;\
      cb_data->callback_func = (void*)_func;\
      cb_data->callback_type = ada_callback_type(_func);\
      cb_data->arg_count = _count;\
      if ( _count ) {\
        uint32_t arguments[] = { _ADA_CB_ARGS(__VA_ARGS__) };\
        memcpy(cb_data->arguments, arguments, 4*_count);\
      }\
      ada_callback_queue(cb_data);\
    }while(0)

void ada_callback_init(void);
void ada_callback_queue(ada_callback_t* cb_data);

#endif /* ADACALLBACK_H_ */
