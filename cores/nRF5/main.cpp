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

// weak function to avoid compilation error with
// non-Bluefruit library sketch such as ADC read test
void Bluefruit_printInfo() __attribute__((weak));
void Bluefruit_printInfo() {}

// DEBUG Level 3
#if CFG_SYSVIEW
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

  setup();

#if CFG_DEBUG && (CFG_LOGGER & ADALOG_TYPE_SERIAL)
  // If Serial is not begin(), call it to avoid hard fault
  // But, for nRF52832 / nRF52840 boards, Serial is USB CDC, so these are no-op...
  if (!Serial) Serial.begin(115200);
#endif

  dbgPrintVersion();
  // dbgMemInfo();

#if CFG_DEBUG
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

#if (CFG_LOGGER & ADALOG_TYPE_RTT)
  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(0, nullptr, nullptr, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Initialized\n");
#endif

  init();
  initVariant();

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

// DESIGN CHOICE -- Order of preference for logging is first of these enabled:
// 1. RTT, if enabled
// 2. SWO, if enabled
// 3. Serial, if enabled

#if (CFG_LOGGER & ADALOG_TYPE_RTT)

  Segger_RTT_Serial_t SerialRTT;
  Stream& Adalog_Default_Logger = SerialRTT;

  extern "C"
  {
    // RTT printf() retarget
    int _write (int fd, const void *buf, size_t count)
    {
      int retval;
      if (fd < SEGGER_RTT_NUMBER_OF_TERMINALS) {
        retval = SEGGER_RTT_TerminalOutBuffer((unsigned char)fd, buf, count);
        if (retval < 0) {
          retval = -1;
        }
      } else {
        retval = -1;
      }
      return retval;
    }
  }


#elif (CFG_LOGGER & ADALOG_TYPE_SERIAL)

  extern "C"
  {
    // nanolib printf() retarget
    int _write (int fd, const void *buf, size_t count)
    {
      (void) fd;

      if ( Serial )
      {
        return Serial.write( (const uint8_t *) buf, count);
      }
      return 0;
    }
  }
  Stream& Adalog_Default_Logger = Serial;

#endif
