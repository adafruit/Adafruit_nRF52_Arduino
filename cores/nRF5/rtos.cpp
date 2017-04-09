/**************************************************************************/
/*!
    @file     rtos.c
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

#include "Arduino.h"

void yield(void)
{
  taskYIELD();
}

SchedulerRTOS Scheduler;

static void _redirect_task(void* arg)
{
  SchedulerRTOS::taskfunc_t taskfunc = (SchedulerRTOS::taskfunc_t) arg;

  while(1)
  {
    taskfunc();

    // yield() anyway just in case user forgot
    taskYIELD();
  }
}

SchedulerRTOS::SchedulerRTOS(void)
{
  _num = 1; // loop is already created by default
}

bool SchedulerRTOS::startLoop(taskfunc_t task, uint32_t stack_size)
{
  char name[8] = "loop0";
  name[4] += _num;

  if ( startLoop(task, name, stack_size) )
  {
    _num++;
    return true;
  }else
  {
    return false;
  }
}

bool SchedulerRTOS::startLoop(taskfunc_t task, const char* name, uint32_t stack_size)
{
  TaskHandle_t  handle;
  return pdPASS == xTaskCreate( _redirect_task, name, stack_size, (void*) task, TASK_PRIO_NORMAL, &handle);
}


extern "C"
{

void vApplicationIdleHook( void )
{
  // Enter low-power mode will turn of hardware PWM
  // Only go there if no PWM device is active.
  if ( !(PWM0.begun() ||  PWM1.begun() || PWM2.begun() ) )
  {
    waitForEvent();
  }
}

}
