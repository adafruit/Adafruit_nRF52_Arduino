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
#include <Arduino.h>

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

int dbgStackTotal(void)
{
  return ((uint32_t) __StackTop) - ((uint32_t) __StackLimit);
}

int dbgStackUsed(void)
{
  enum { STACK_PATTERN = 0xADADADAD };

  uint32_t * p_start = (uint32_t*) &__StackLimit;
  uint32_t * p_end   = (uint32_t*) &__StackTop;

  uint32_t * p_buf = p_start;
  while( *p_buf == STACK_PATTERN && p_buf != p_end)
  {
    p_buf++;
  }

  if (p_buf == p_end) return (-1);

  return ((uint32_t) p_end) - ((uint32_t) p_buf);
}

static void printMemRegion(const char* name, uint32_t top, uint32_t bottom, uint32_t used)
{
  char buffer[30];
  if ( used )
  {
    sprintf(buffer, "%5lu / %5lu (%02lu%%)", used, top-bottom, (used*100)/ (top-bottom));
  }else
  {
    sprintf(buffer, "%lu", top-bottom);
  }

  Serial.printf("| %-5s| 0x%04X - 0x%04X | %-19s |\n", name, (uint16_t) bottom, (uint16_t) (top-1), buffer);
}

void dbgMemInfo(void)
{
  Serial.println(" ______________________________________________");
  Serial.println("| Name | Addr 0x2000xxxx | Usage               |");
  Serial.println("| ---------------------------------------------|");

  // Pritn SRAM used for Stack executed by S132 and ISR
  printMemRegion("Stack", ((uint32_t) __StackTop), ((uint32_t) __StackLimit), dbgStackUsed() );

  // Print Heap usage overall (including memory malloced to tasks)
  printMemRegion("Heap", ((uint32_t) __HeapLimit), ((uint32_t) __HeapBase), dbgHeapUsed() );

  // DATA + BSS
  printMemRegion("Bss", ((uint32_t) __bss_end__), ((uint32_t) __data_start__), 0);

  // Print SRAM Used by SoftDevice
  printMemRegion("S132", (uint32_t) __data_start__, 0x20000000, 0);

  Serial.printf("|______________________________________________|\n");
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
  Serial.println("BSP Library     : " ARDUINO_BSP_VERSION);
  Serial.printf ("Firmware        : %s\n", getFirmwareVersion());
  Serial.printf ("Serial No       : %s\n", getMcuUniqueID());
  Serial.println();
}

/******************************************************************************/
/*!
    @brief  Helper function to display memory contents in a friendly format
*/
/******************************************************************************/
void dbgDumpMemory(void const *buf, uint8_t size, uint16_t count, bool printOffset)
{
  uint8_t const *buf8 = (uint8_t const *) buf;

  char format[] = "%00lX";
  format[2] += 2*size;

  char offset_fmt[] = "%02lX: ";

  if ( count*size > UINT8_MAX  ) format[2] *= 2;
  if ( count*size > UINT16_MAX ) format[2] *= 2;

  const uint8_t item_per_line = 16 / size;

  for(int i=0; i<count; i++)
  {
    uint32_t value=0;

    // Print address
    if ( i%item_per_line == 0 )
    {
      if ( i != 0 ) Serial.println();

      // print offset or absolute address
      if (printOffset)
      {
        Serial.printf(offset_fmt, 16*i/item_per_line);
      }else
      {
        Serial.printf("%08lX:", (uint32_t) buf8);
      }
    }

    memcpy(&value, buf8, size);
    buf8 += size;

    Serial.print(' ');
    Serial.printf(format, value);
  }

  Serial.println();
}

#if CFG_DEBUG

#include "ble.h"
#include "ble_hci.h"

// TODO require update when upgrading SoftDevice

/*------------------------------------------------------------------*/
/* Event String
 *------------------------------------------------------------------*/

// Common BLE Event base
static const char* _evt_base_str[] =
{
#if SD_VER < 500
    "BLE_EVT_TX_COMPLETE"                     ,
#endif
    "BLE_EVT_USER_MEM_REQUEST"                ,
    "BLE_EVT_USER_MEM_RELEASE"                ,
};

static const char* _evt_gap_str[] =
{
    "BLE_GAP_EVT_CONNECTED"                  ,
    "BLE_GAP_EVT_DISCONNECTED"               ,
    "BLE_GAP_EVT_CONN_PARAM_UPDATE"          ,
    "BLE_GAP_EVT_SEC_PARAMS_REQUEST"         ,
    "BLE_GAP_EVT_SEC_INFO_REQUEST"           ,
    "BLE_GAP_EVT_PASSKEY_DISPLAY"            ,
    "BLE_GAP_EVT_KEY_PRESSED"                ,
    "BLE_GAP_EVT_AUTH_KEY_REQUEST"           ,
    "BLE_GAP_EVT_LESC_DHKEY_REQUEST"         ,
    "BLE_GAP_EVT_AUTH_STATUS"                ,
    "BLE_GAP_EVT_CONN_SEC_UPDATE"            ,
    "BLE_GAP_EVT_TIMEOUT"                    ,
    "BLE_GAP_EVT_RSSI_CHANGED"               ,
    "BLE_GAP_EVT_ADV_REPORT"                 ,
    "BLE_GAP_EVT_SEC_REQUEST"                ,
    "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST"  ,
    "BLE_GAP_EVT_SCAN_REQ_REPORT"            ,
    "BLE_GAP_EVT_PHY_UPDATE_REQUEST"         ,
    "BLE_GAP_EVT_PHY_UPDATE"                 ,
    "BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST" ,
    "BLE_GAP_EVT_DATA_LENGTH_UPDATE"         ,
};

// GATTC Event
static const char* _evt_gattc_str[] =
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
#if SD_VER < 500
    "BLE_GATTC_EVT_TIMEOUT"                   ,
#else
    "BLE_GATTC_EVT_EXCHANGE_MTU_RSP"          ,
    "BLE_GATTC_EVT_TIMEOUT"                   ,
    "BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE"     ,
#endif
};

// GATTS Event
static const char* _evt_gatts_str[] =
{
    "BLE_GATTS_EVT_WRITE"                     ,
    "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST"      ,
    "BLE_GATTS_EVT_SYS_ATTR_MISSING"          ,
    "BLE_GATTS_EVT_HVC"                       ,
    "BLE_GATTS_EVT_SC_CONFIRM"                ,
#if SD_VER < 500
    "BLE_GATTC_EVT_TIMEOUT"                   ,
#else
    "BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST"      ,
    "BLE_GATTS_EVT_TIMEOUT"                   ,
    "BLE_GATTS_EVT_HVN_TX_COMPLETE"           ,
#endif
};

const char* dbg_ble_event_str(uint16_t evt_id)
{
  static char unknown_evt[7] = {0};

  if      ( is_within(BLE_EVT_BASE, evt_id, BLE_EVT_BASE+arrcount(_evt_base_str)) )
    return _evt_base_str[evt_id-BLE_EVT_BASE];
  else if ( is_within(BLE_GAP_EVT_BASE, evt_id, BLE_GAP_EVT_BASE+arrcount(_evt_gap_str)) )
    return _evt_gap_str[evt_id-BLE_GAP_EVT_BASE];
  else if ( is_within(BLE_GATTC_EVT_BASE, evt_id, BLE_GATTC_EVT_BASE+arrcount(_evt_gattc_str) ) )
    return _evt_gattc_str[evt_id-BLE_GATTC_EVT_BASE];
  else if ( is_within(BLE_GATTS_EVT_BASE, evt_id, BLE_GATTS_EVT_BASE+arrcount(_evt_gatts_str)) )
    return _evt_gatts_str[evt_id-BLE_GATTS_EVT_BASE];
  else
  {
    sprintf(unknown_evt, "0x%04X", evt_id);
    return unknown_evt;
  }
}

/*------------------------------------------------------------------*/
/* Error String
 *------------------------------------------------------------------*/
static lookup_entry_t const _err_lookup_items[] =
{
    // General
    { .key = NRF_ERROR_SVC_HANDLER_MISSING                     , .data = "NRF_ERROR_SVC_HANDLER_MISSING" },
    { .key = NRF_ERROR_SOFTDEVICE_NOT_ENABLED                  , .data = "NRF_ERROR_SOFTDEVICE_NOT_ENABLED" },
    { .key = NRF_ERROR_INTERNAL                                , .data = "NRF_ERROR_INTERNAL" },
    { .key = NRF_ERROR_NO_MEM                                  , .data = "NRF_ERROR_NO_MEM" },
    { .key = NRF_ERROR_NOT_FOUND                               , .data = "NRF_ERROR_NOT_FOUND" },
    { .key = NRF_ERROR_NOT_SUPPORTED                           , .data = "NRF_ERROR_NOT_SUPPORTED" },
    { .key = NRF_ERROR_INVALID_PARAM                           , .data = "NRF_ERROR_INVALID_PARAM" },
    { .key = NRF_ERROR_INVALID_STATE                           , .data = "NRF_ERROR_INVALID_STATE" },
    { .key = NRF_ERROR_INVALID_LENGTH                          , .data = "NRF_ERROR_INVALID_LENGTH" },
    { .key = NRF_ERROR_INVALID_FLAGS                           , .data = "NRF_ERROR_INVALID_FLAGS" },
    { .key = NRF_ERROR_INVALID_DATA                            , .data = "NRF_ERROR_INVALID_DATA" },
    { .key = NRF_ERROR_DATA_SIZE                               , .data = "NRF_ERROR_DATA_SIZE" },
    { .key = NRF_ERROR_TIMEOUT                                 , .data = "NRF_ERROR_TIMEOUT" },
    { .key = NRF_ERROR_NULL                                    , .data = "NRF_ERROR_NULL" },
    { .key = NRF_ERROR_FORBIDDEN                               , .data = "NRF_ERROR_FORBIDDEN" },
    { .key = NRF_ERROR_INVALID_ADDR                            , .data = "NRF_ERROR_INVALID_ADDR" },
    { .key = NRF_ERROR_BUSY                                    , .data = "NRF_ERROR_BUSY" },
    { .key = NRF_ERROR_CONN_COUNT                              , .data = "NRF_ERROR_CONN_COUNT" },
    { .key = NRF_ERROR_RESOURCES                               , .data = "NRF_ERROR_RESOURCES" },

    // SDM
    { .key = NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN                , .data = "NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN" },
    { .key = NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION   , .data = "NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION" },
    { .key = NRF_ERROR_SDM_INCORRECT_CLENR0                    , .data = "NRF_ERROR_SDM_INCORRECT_CLENR0" },

    // SOC
    { .key = NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN                 , .data = "NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN" },
    { .key = NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE        , .data = "NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE" },
    { .key = NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED , .data = "NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED" },
    { .key = NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN              , .data = "NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN" },
    { .key = NRF_ERROR_SOC_POWER_MODE_UNKNOWN                  , .data = "NRF_ERROR_SOC_POWER_MODE_UNKNOWN" },
    { .key = NRF_ERROR_SOC_POWER_POF_THRESHOLD_UNKNOWN         , .data = "NRF_ERROR_SOC_POWER_POF_THRESHOLD_UNKNOWN" },
    { .key = NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN         , .data = "NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN" },
    { .key = NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES              , .data = "NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES" },
    { .key = NRF_ERROR_SOC_PPI_INVALID_CHANNEL                 , .data = "NRF_ERROR_SOC_PPI_INVALID_CHANNEL" },
    { .key = NRF_ERROR_SOC_PPI_INVALID_GROUP                   , .data = "NRF_ERROR_SOC_PPI_INVALID_GROUP" },

    // BLE Generic
    { .key = BLE_ERROR_NOT_ENABLED                             , .data = "BLE_ERROR_NOT_ENABLED" },
    { .key = BLE_ERROR_INVALID_CONN_HANDLE                     , .data = "BLE_ERROR_INVALID_CONN_HANDLE" },
    { .key = BLE_ERROR_INVALID_ATTR_HANDLE                     , .data = "BLE_ERROR_INVALID_ATTR_HANDLE" },
    { .key = BLE_ERROR_INVALID_ROLE                            , .data = "BLE_ERROR_INVALID_ROLE" },

    // BLE GAP
    { .key = BLE_ERROR_GAP_UUID_LIST_MISMATCH                  , .data = "BLE_ERROR_GAP_UUID_LIST_MISMATCH" },
    { .key = BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST         , .data = "BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST" },
    { .key = BLE_ERROR_GAP_INVALID_BLE_ADDR                    , .data = "BLE_ERROR_GAP_INVALID_BLE_ADDR" },
    { .key = BLE_ERROR_GAP_WHITELIST_IN_USE                    , .data = "BLE_ERROR_GAP_WHITELIST_IN_USE" },
#if SD_VER >= 500
    { .key = BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE            , .data = "BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE" },
    { .key = BLE_ERROR_GAP_DEVICE_IDENTITIES_DUPLICATE         , .data = "BLE_ERROR_GAP_DEVICE_IDENTITIES_DUPLICATE" },
#endif

    // BLE GATTC
    { .key = BLE_ERROR_GATTC_PROC_NOT_PERMITTED                , .data = "BLE_ERROR_GATTC_PROC_NOT_PERMITTED" },

    // BLE GATTS
    { .key = BLE_ERROR_GATTS_INVALID_ATTR_TYPE                 , .data = "BLE_ERROR_GATTS_INVALID_ATTR_TYPE" },
    { .key = BLE_ERROR_GATTS_SYS_ATTR_MISSING                  , .data = "BLE_ERROR_GATTS_SYS_ATTR_MISSING" },
};

lookup_table_t const _err_table =
{
  .count = arrcount(_err_lookup_items),
  .items = _err_lookup_items
};

const char* dbg_err_str(uint32_t err_id)
{
  const char * str = (const char *) lookup_find(&_err_table, err_id);
  static char unknown_err[7] = {0};

  if ( str == NULL )
  {
    sprintf(unknown_err, "0x%04X", err_id);
    str = unknown_err;
  }

  return str;
}

#endif

