/**************************************************************************/
/*!
    @file     debug.cpp
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

#include <stdint.h>
#include <stdarg.h>
#include <malloc.h>
#include "debug.h"

// defined in linker script
extern uint32_t __data_start__[];
extern uint32_t __data_end__[];

extern uint32_t __bss_start__[];
extern uint32_t __bss_end__[];

extern unsigned char __HeapBase[];
extern unsigned char __HeapLimit[];

extern uint32_t __StackTop[];
extern uint32_t __StackLimit[];

extern "C"
{
  int cprintf(const char * format, ...)
  {
    char buf[256];
    int len;

    va_list ap;
    va_start(ap, format);

    len = vsnprintf(buf, 256, format, ap);
    Serial.write(buf, len);

    va_end(ap);
    return len;
  }

  void vApplicationMallocFailedHook(void)
  {
    Serial.println("Failed to Malloc");
  }

  void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    Serial.printf("%s Stack Overflow !!!", pcTaskName);
  }
}

int dbgHeapTotal(void)
{
  return ((uint32_t) __HeapLimit) - ((uint32_t) __HeapBase);
}

int dbgHeapUsed(void)
{
  return (mallinfo()).uordblks;
}

static void printCenter(const char* str, int width)
{
  int lpad = (width-strlen(str))/2;
  int rpad = width - strlen(str) - lpad;

  Serial.printf("|%*c%s%*c|\n", lpad, ' ', str, rpad, ' ');
}

static void printMemRegion(const char* name, uint32_t top, uint32_t bottom, uint32_t used)
{
  const int WIDTH = 21;

  Serial.printf(" _____________________  0x%08lX\n", top);
  Serial.printf("|                     |\n");

  printCenter(name, WIDTH);

  char buffer[30];
  if ( used )
  {
    sprintf(buffer, "%lu / %lu (%02lu%%)", used, top-bottom, (used*100)/ (top-bottom));
  }else
  {
    sprintf(buffer, "%lu", top-bottom);
  }
  printCenter(buffer, WIDTH);


  Serial.printf("|_____________________| 0x%08lX\n", bottom);
}

void dbgMemInfo(void)
{
  Serial.println("       Memory Map");

  // Pritn SRAM used for Stack executed by S132 and ISR
  printMemRegion("Stack", ((uint32_t) __StackTop), ((uint32_t) __StackLimit), 0);

  // Print Heap usage overall (including memory malloced to tasks)
  printMemRegion("Heap", ((uint32_t) __HeapLimit), ((uint32_t) __HeapBase), dbgHeapUsed());

  // DATA + BSS
  printMemRegion("Data & Bss", ((uint32_t) __bss_end__), ((uint32_t) __data_start__), 0);

  // Print SRAM Used by SoftDevice
  printMemRegion("S132", (uint32_t) __data_start__, 0x20000000, 0);

  Serial.println();

  // Print Task list
  uint32_t tasknum = uxTaskGetNumberOfTasks();
  char* buf = (char*) rtos_malloc(tasknum*40); // 40 bytes per task

  vTaskList(buf);

  Serial.println("Task    State   Prio    Stack   Num");
  Serial.println("-----------------------------------");
  Serial.println(buf);
  rtos_free(buf);
}
