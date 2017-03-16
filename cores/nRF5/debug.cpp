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
#include "utility/utilities.h"

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

static void printMemRegion(const char* name, uint32_t top, uint32_t bottom, uint32_t used)
{
  char buffer[30];
  if ( used )
  {
    sprintf(buffer, "%lu / %lu (%02lu%%)", used, top-bottom, (used*100)/ (top-bottom));
  }else
  {
    sprintf(buffer, "%lu", top-bottom);
  }

  Serial.printf("| %-10s: %-19s |\n", name, buffer);
}

void dbgMemInfo(void)
{
  Serial.printf (" _________________________________\n");
  Serial.println("|                                 |");

  // Pritn SRAM used for Stack executed by S132 and ISR
  printMemRegion("Stack", ((uint32_t) __StackTop), ((uint32_t) __StackLimit), 0);

  // Print Heap usage overall (including memory malloced to tasks)
  printMemRegion("Heap", ((uint32_t) __HeapLimit), ((uint32_t) __HeapBase), dbgHeapUsed());

  // DATA + BSS
  printMemRegion("Data & Bss", ((uint32_t) __bss_end__), ((uint32_t) __data_start__), 0);

  // Print SRAM Used by SoftDevice
  printMemRegion("S132", (uint32_t) __data_start__, 0x20000000, 0);

  Serial.printf("|_________________________________|\n");
  Serial.println();

  // Print Task list
  uint32_t tasknum = uxTaskGetNumberOfTasks();
  char* buf = (char*) rtos_malloc(tasknum*40); // 40 bytes per task

  vTaskList(buf);

  Serial.println("Task    State   Prio  StackLeft Num");
  Serial.println("-----------------------------------");
  Serial.println(buf);
  rtos_free(buf);
}

void dbgPrintVersion(void)
{
  Serial.println();
  Serial.println("BSP Library: " ARDUINO_BSP_VERSION);
  Serial.printf ("Firmware   : %s\n", getFirmwareVersion());
  Serial.printf ("Serial No  : %s\n", getMcuUniqueID());
  Serial.println();
}

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/

#if CFG_DEBUG

#include "ble.h"
#include "ble_hci.h"

// Common BLE Event base
static const char* _base_evt_str[] =
{
    "BLE_EVT_TX_COMPLETE"                     ,
    "BLE_EVT_USER_MEM_REQUEST"                ,
    "BLE_EVT_USER_MEM_RELEASE"                ,
};

static const char* _gap_evt_str[] =
{
    "BLE_GAP_EVT_CONNECTED"                   ,
    "BLE_GAP_EVT_DISCONNECTED"                ,
    "BLE_GAP_EVT_CONN_PARAM_UPDATE"           ,
    "BLE_GAP_EVT_SEC_PARAMS_REQUEST"          ,
    "BLE_GAP_EVT_SEC_INFO_REQUEST"            ,
    "BLE_GAP_EVT_PASSKEY_DISPLAY"             ,
    "BLE_GAP_EVT_KEY_PRESSED"                 ,
    "BLE_GAP_EVT_AUTH_KEY_REQUEST"            ,
    "BLE_GAP_EVT_LESC_DHKEY_REQUEST"          ,
    "BLE_GAP_EVT_AUTH_STATUS"                 ,
    "BLE_GAP_EVT_CONN_SEC_UPDATE"             ,
    "BLE_GAP_EVT_TIMEOUT"                     ,
    "BLE_GAP_EVT_RSSI_CHANGED"                ,
    "BLE_GAP_EVT_ADV_REPORT"                  ,
    "BLE_GAP_EVT_SEC_REQUEST"                 ,
    "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST"   ,
    "BLE_GAP_EVT_SCAN_REQ_REPORT"             ,
};

// GATTC Event
static const char* _gattc_evt_str[] =
{
    "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP"        ,
    "BLE_GATTC_EVT_REL_DISC_RSP"              ,
    "BLE_GATTC_EVT_CHAR_DISC_RSP"             ,
    "BLE_GATTC_EVT_DESC_DISC_RSP"             ,
    "BLE_GATTC_EVT_ATTR_INFO_DISC_RSP"        ,
    "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP" ,
    "BLE_GATTC_EVT_READ_RSP"                  ,
    "BLE_GATTC_EVT_CHAR_VALS_READ_RSP"        ,
    "BLE_GATTC_EVT_WRITE_RSP"                 ,
    "BLE_GATTC_EVT_HVX"                       ,
    "BLE_GATTC_EVT_TIMEOUT"
};

// GATTS Event
static const char* _gatts_evt_str[] =
{
    "BLE_GATTS_EVT_WRITE"                     ,
    "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST"      ,
    "BLE_GATTS_EVT_SYS_ATTR_MISSING"          ,
    "BLE_GATTS_EVT_HVC"                       ,
    "BLE_GATTS_EVT_SC_CONFIRM"                ,
    "BLE_GATTS_EVT_TIMEOUT"                   ,
};

const char* dbg_ble_event_str(uint16_t evt_id)
{
  static char unknown_evt[7] = {0};

  if      ( is_within(BLE_EVT_BASE, evt_id, BLE_EVT_LAST) )
    return _base_evt_str[evt_id-BLE_EVT_BASE];
  else if ( is_within(BLE_GAP_EVT_BASE, evt_id, BLE_GAP_EVT_LAST) )
    return _gap_evt_str[evt_id-BLE_GAP_EVT_BASE];
  else if ( is_within(BLE_GATTC_EVT_BASE, evt_id, BLE_GATTC_EVT_LAST) )
    return _gattc_evt_str[evt_id-BLE_GATTC_EVT_BASE];
  else if ( is_within(BLE_GATTS_EVT_BASE, evt_id, BLE_GATTS_EVT_LAST) )
    return _gatts_evt_str[evt_id-BLE_GATTS_EVT_BASE];
  else
  {
    sprintf(unknown_evt, "0x04X", evt_id);
    return unknown_evt;
  }
}

#endif

