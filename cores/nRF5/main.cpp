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
#if (CFG_LOGGER == 2)
  #include <SEGGER_RTT.h>
#endif

// DEBUG Level 1
#if CFG_DEBUG
// weak function to avoid compilation error with
// non-Bluefruit library sketch such as ADC read test
void Bluefruit_printInfo() __attribute__((weak));
void Bluefruit_printInfo() {}
#endif

// From the UI, setting debug level to 3 will enable SysView
#if CFG_SYSVIEW
#include "SEGGER_RTT.h"
#include "SEGGER_SYSVIEW.h"
#endif

static TaskHandle_t  _loopHandle;


// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

#define LOOP_STACK_SZ       (256*4)
#define CALLBACK_STACK_SZ   (256*3)

static void loop_task(void* arg)
{
  (void) arg;

#if CFG_DEBUG
  // If Serial is not begin(), call it to avoid hard fault
  if(!Serial) Serial.begin(115200);
#endif

  setup();

#if CFG_DEBUG
  dbgPrintVersion();
  Bluefruit_printInfo();
#endif

  while (1)
  {
    loop();
    yield(); // yield to run other task

    // Serial events
    if (serialEvent && serialEventRun) serialEventRun();
  }
}

// \brief Main entry point of Arduino application
int main( void )
{
  init();
  initVariant();

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_Conf();
#endif

#ifdef USE_TINYUSB
  Adafruit_TinyUSB_Core_init();
#endif

  // Create a task for loop()
  xTaskCreate( loop_task, "loop", LOOP_STACK_SZ, NULL, TASK_PRIO_LOW, &_loopHandle);

  // Initialize callback task
  ada_callback_init(CALLBACK_STACK_SZ);

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();

  NVIC_SystemReset();

  return 0;
}

void suspendLoop(void)
{
  vTaskSuspend(_loopHandle);
}

void resumeLoop(void)
{
  if ( isInISR() ) 
  {
    xTaskResumeFromISR(_loopHandle);
  } 
  else
  {
    vTaskResume(_loopHandle);
  }
}

extern "C"
{

// nanolib printf() retarget
int _write (int fd, const void *buf, size_t count)
{
  (void) fd;
#if (CFG_LOGGER == 2)
  unsigned numBytes = count;
  SEGGER_RTT_Write(0, buf, numBytes);
  return (int)count;
#else
  if ( Serial )
  {
    return Serial.write( (const uint8_t *) buf, count);
  }
#endif
  return 0;
}

}

