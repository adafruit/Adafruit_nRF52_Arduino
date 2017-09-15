/**************************************************************************/
/*!
    @file     rtos.h
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
#ifndef RTOS_H_
#define RTOS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <malloc.h>

#include "common_func.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#define DELAY_FOREVER   portMAX_DELAY
enum
{
  TASK_PRIO_LOWEST  = 0, // Idle task, should not be used
  TASK_PRIO_LOW     = 1, // Loop
  TASK_PRIO_NORMAL  = 2, // Timer Task, Callback Task
  TASK_PRIO_HIGH    = 3, // Bluefruit Task
  TASK_PRIO_HIGHEST = 4,
};

#define malloc_type(type)     rtos_malloc( sizeof(type) )

#if 0
#define rtos_malloc(_size)  ({ cprintf("[malloc] %s:%d : %d bytes\r\n", __PRETTY_FUNCTION__, __LINE__, _size); pvPortMalloc(_size); })
#define rtos_free(ptr)      ({ cprintf("[free] %s:%d\r\n"    ,__PRETTY_FUNCTION__, __LINE__/*malloc_usable_size(ptr)*/); vPortFree(ptr); })
#else

#define rtos_malloc_type(_type)   (_type*) rtos_malloc(sizeof(_type))

static inline void* rtos_malloc(size_t _size)
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? malloc(_size) : pvPortMalloc(_size);
}

static inline void rtos_free( void *pv )
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? free(pv) : vPortFree(pv);
}

static inline void* rtos_realloc(void* pv, size_t new_size)
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? realloc(pv, new_size) : pvPortRealloc(pv, new_size);
}
#endif

#ifdef __cplusplus // Visible only with cplusplus

#define SCHEDULER_STACK_SIZE_DFLT   (512*2)

class SchedulerRTOS
{
private:
  uint8_t _num;

public:
  typedef void (*taskfunc_t)(void);

  SchedulerRTOS(void);

  bool startLoop(taskfunc_t task, uint32_t stack_size = SCHEDULER_STACK_SIZE_DFLT);
  bool startLoop(taskfunc_t task, const char* name, uint32_t stack_size = SCHEDULER_STACK_SIZE_DFLT);
};

extern SchedulerRTOS Scheduler;

#endif


#endif /* RTOS_H_ */
