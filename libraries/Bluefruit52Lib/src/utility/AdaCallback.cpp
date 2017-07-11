/**************************************************************************/
/*!
    @file     AdaCallback.cpp
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

#include "AdaCallback.h"

static QueueHandle_t _cb_queue = NULL;

void adafruit_callback_task(void* arg)
{
  (void) arg;

  while(1)
  {
    ada_callback_t* cb_data;
    if ( xQueueReceive(_cb_queue, (void*) &cb_data, portMAX_DELAY) )
    {
//      PRINT_HEX(cb_data);
//      PRINT_INT(cb_data->callback_type);
//      PRINT_HEX(cb_data->malloced_data);

      void* func = cb_data->callback_func;
      uint32_t* args = cb_data->arguments;

      switch(cb_data->callback_type)
      {
        case AdafruitBluefruit_connect_callback_t:
          ((AdafruitBluefruit::connect_callback_t) func) ();
        break;

        case BLECentral_connect_callback_t:
          ((BLECentral::connect_callback_t) func)( (uint16_t) args[0]);
        break;

        case BLEClientCharacteristic_notify_cb_t:
          ((BLEClientCharacteristic::notify_cb_t) func) ( *((BLEClientCharacteristic*) args[0]), (uint8_t*) args[1], (uint16_t) args[2] );
        break;

        default: VERIFY_MESS(NRF_ERROR_INVALID_PARAM); break;
      }

      // free up resource
      if (cb_data->malloced_data) rtos_free(cb_data->malloced_data);
      rtos_free(cb_data);
    }
  }
}

void ada_callback_queue(ada_callback_t* cb_data)
{
  xQueueSend(_cb_queue, (void*) &cb_data, BLE_GENERIC_TIMEOUT);
}

void ada_callback_init(void)
{
  // queue to hold "Pointer to callback data"
  _cb_queue = xQueueCreate(CFG_CALLBACK_QUEUE_LENGTH, sizeof(ada_callback_t*));

  TaskHandle_t callback_task_hdl;
  xTaskCreate( adafruit_callback_task, "Callback", CFG_CALLBACK_TASK_STACKSIZE, NULL, TASK_PRIO_NORMAL, &callback_task_hdl);
}
