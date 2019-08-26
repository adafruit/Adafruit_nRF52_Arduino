/**************************************************************************/
/*!
    @file     AdaCallback.cpp
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

#include "Arduino.h"

static QueueHandle_t _cb_queue = NULL;

void adafruit_callback_task(void* arg)
{
  (void) arg;

  while(1)
  {
    ada_callback_t* cb_data;
    if ( xQueueReceive(_cb_queue, (void*) &cb_data, portMAX_DELAY) )
    {
      const void* func = cb_data->callback_func;
      uint32_t* args = cb_data->arguments;

      switch (cb_data->arg_count)
      {
        case 0: ((adacb_0arg_t) func)();                                            break;
        case 1: ((adacb_1arg_t) func)(args[0]);                                     break;
        case 2: ((adacb_2arg_t) func)(args[0], args[1]);                            break;
        case 3: ((adacb_3arg_t) func)(args[0], args[1], args[2]);                   break;
        case 4: ((adacb_4arg_t) func)(args[0], args[1], args[2], args[3]);          break;
        case 5: ((adacb_5arg_t) func)(args[0], args[1], args[2], args[3], args[4]); break;

        default: VERIFY_MESS(NRF_ERROR_INVALID_PARAM, dbg_err_str); break;
      }

      // free up resource
      if (cb_data->malloced_data) rtos_free(cb_data->malloced_data);
      rtos_free(cb_data);
    }
  }
}

//! Test if in interrupt mode
static inline bool is_isr(void)
{
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

void ada_callback_queue(ada_callback_t* cb_item)
{
  if ( is_isr() )
  {
    xQueueSendFromISR(_cb_queue, (void*) &cb_item, NULL);
  }else
  {
    if ( !xQueueSend(_cb_queue, (void*) &cb_item, CFG_CALLBACK_TIMEOUT) )
    {
      LOG_LV1("MEMORY", "AdaCallback run out of queue item, increase CFG_CALLBACK_QUEUE_LENGTH");
    }
  }
}

void ada_callback_invoke(const void* malloc_data, uint32_t malloc_len, const void* func, uint32_t arguments[], uint8_t argcount)
{
  ada_callback_t* cb_data = (ada_callback_t*) rtos_malloc( sizeof(ada_callback_t) + (argcount ? (argcount-1)*4 : 0) );
  cb_data->malloced_data = NULL;
  cb_data->callback_func = func;
  cb_data->arg_count = argcount;

  if ( malloc_data && malloc_len )
  {
    cb_data->malloced_data = rtos_malloc(malloc_len);
    memcpy(cb_data->malloced_data, malloc_data, malloc_len);
  }

  if ( argcount )
  {
    // argument most likely has _mdata if used, change it to malloced one
    if ( malloc_data && malloc_len )
    {
      for(uint8_t i=0; i<argcount; i++)
      {
        if (arguments[i] == ((uint32_t) malloc_data)) arguments[i] = ((uint32_t) cb_data->malloced_data);
      }
    }

    memcpy(cb_data->arguments, arguments, 4*argcount);
  }

  ada_callback_queue(cb_data);
}

void ada_callback_init(void)
{
  // queue to hold "Pointer to callback data"
  _cb_queue = xQueueCreate(CFG_CALLBACK_QUEUE_LENGTH, sizeof(ada_callback_t*));

  TaskHandle_t callback_task_hdl;
  xTaskCreate( adafruit_callback_task, "Callback", CFG_CALLBACK_TASK_STACKSIZE, NULL, TASK_PRIO_NORMAL, &callback_task_hdl);
}
