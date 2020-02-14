/**************************************************************************/
/*!
    @file     rtos.h
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
#ifndef RTOS_H_
#define RTOS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "common_func.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#define DEBUG_MALLOC    1

#define DELAY_FOREVER   portMAX_DELAY

enum
{
  TASK_PRIO_LOWEST  = 0, // Idle task, should not be used
  TASK_PRIO_LOW     = 1, // Loop
  TASK_PRIO_NORMAL  = 2, // Timer Task, Callback Task
  TASK_PRIO_HIGH    = 3, // Bluefruit Task
  TASK_PRIO_HIGHEST = 4,
};

#define ms2tick              pdMS_TO_TICKS

#define tick2ms(tck)         ( ( ((uint64_t)(tck)) * 1000) / configTICK_RATE_HZ )
#define tick2us(tck)         ( ( ((uint64_t)(tck)) * 1000000) / configTICK_RATE_HZ )

#if DEBUG_MALLOC
  #define rtos_malloc_type(_type)   ({ LOG_LV2("MALLOC", #_type " = %d bytes", sizeof(_type)); ((_type*) rtos_malloc(sizeof(_type))); })
#else
  #define rtos_malloc_type(_type)   ((_type*) rtos_malloc(sizeof(_type)))
#endif

static inline void* rtos_malloc(size_t _size)
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? malloc(_size) : pvPortMalloc(_size);
}

static inline void rtos_free( void *pv )
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? free(pv) : vPortFree(pv);
}

// Visible only with C++
#ifdef __cplusplus

#define SCHEDULER_STACK_SIZE_DFLT   (512*2)

class SchedulerRTOS
{
public:
  typedef void (*taskfunc_t)(void);

  SchedulerRTOS(void);

  bool startLoop(taskfunc_t task, uint32_t stack_size = SCHEDULER_STACK_SIZE_DFLT, uint32_t prio = TASK_PRIO_LOW, const char* name = NULL);
};

extern SchedulerRTOS Scheduler;

#endif // __cplusplus

#endif /* RTOS_H_ */
