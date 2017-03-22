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

#define MEMINFO_INTERVAL    60000

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

uint32_t _loopStacksize = 512*3;

uint32_t setLoopStacksize(void) __attribute__ ((weak));

static void loop_task(void* arg)
{
  (void) arg;

  setup();

#if CFG_DEBUG
  // If Serial is not begin(), call it to avoid hard fault
  if (!Serial) Serial.begin(115200);
  dbgPrintVersion();
#endif

  while (1)
  {
    loop();

    #if CFG_DEBUG > 1 // Full Debug
    static uint32_t meminfo_ms = 0;
    if (meminfo_ms + MEMINFO_INTERVAL < millis())
    {
      meminfo_ms += millis();
      Serial.printf("Memory Info (print every %d seconds)\n", MEMINFO_INTERVAL/1000);
      dbgMemInfo();
    }
    #endif

    // To compatible with most code where loop is not rtos-aware
    taskYIELD(); // vTaskDelay(1);
  }
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
  init();

  initVariant();

  if (setLoopStacksize)
  {
    _loopStacksize = setLoopStacksize();
  }

  // Create a task for loop()
  TaskHandle_t  _loopHandle;
  xTaskCreate( loop_task, "loop", _loopStacksize, NULL, TASK_PRIO_NORMAL, &_loopHandle);

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();

  NVIC_SystemReset();

  return 0;
}
