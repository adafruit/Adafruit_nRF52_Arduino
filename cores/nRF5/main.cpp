/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
#include "Arduino.h"

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

TaskHandle_t  _loopHandle;
uint32_t _loopStacksize = 1024*2;

void setLoopStacksize(uint32_t size)
{
  _loopStacksize = size;
}

static void loop_task(void* arg)
{
  (void) arg;

  while (1)
  {
    loop();

    // To compatible with most code where loop is not rtos-aware
    taskYIELD();
  }
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
  init();

  initVariant();

  delay(1);

  setup();

  // Create a task for loop()
  xTaskCreate( loop_task, "loop", _loopStacksize, NULL, TASK_PRIO_NORMAL, &_loopHandle);

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();

  NVIC_SystemReset();

//  for (;;)
//  {
//    loop();
//    if (serialEventRun) serialEventRun();
//  }

  return 0;
}
