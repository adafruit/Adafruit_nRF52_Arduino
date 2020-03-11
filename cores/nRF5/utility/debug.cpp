/**************************************************************************/
/*!
    @file     debug.cpp
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

#include <stdint.h>
#include <stdarg.h>
#include <malloc.h>
#include <Arduino.h>
#include <ctype.h>
#include <common_func.h>

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

void HardFault_Handler(void)
{
  // reset on hardfault
  NVIC_SystemReset();
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

  PRINTF("| %-5s| 0x%04X - 0x%04X | %-19s |\n", name, (uint16_t) bottom, (uint16_t) (top-1), buffer);
}

void dbgMemInfo(void)
{
  PRINTF(" ______________________________________________\n");
  PRINTF("| Name | Addr 0x2000xxxx | Usage               |\n");
  PRINTF("| ---------------------------------------------|\n");

  // Pritn SRAM used for Stack executed by Softdevice and ISR
  printMemRegion("Stack", ((uint32_t) __StackTop), ((uint32_t) __StackLimit), dbgStackUsed() );

  // Print Heap usage overall (including memory malloced to tasks)
  printMemRegion("Heap", ((uint32_t) __HeapLimit), ((uint32_t) __HeapBase), dbgHeapUsed() );

  // DATA + BSS
  printMemRegion("Bss", ((uint32_t) __bss_end__), ((uint32_t) __data_start__), 0);

  // Print SRAM Used by SoftDevice
  printMemRegion("SD", (uint32_t) __data_start__, 0x20000000, 0);

  PRINTF("|______________________________________________|\n");
  PRINTF("\n");

  // Print Task list
  uint32_t tasknum = uxTaskGetNumberOfTasks();
  char* buf = (char*) rtos_malloc(tasknum*40); // 40 bytes per task

  vTaskList(buf);

  PRINTF("Task    State   Prio  StackLeft Num\n");
  PRINTF("-----------------------------------\n");
  PRINTF(buf);
  PRINTF("\n");
  rtos_free(buf);
}

void dbgPrintVersion(void)
{
  PRINTF("\n");
  PRINTF("BSP Library : " ARDUINO_BSP_VERSION "\n");
  PRINTF("Bootloader  : %s\n", getBootloaderVersion());
  PRINTF("Serial No   : %s\n", getMcuUniqueID());
  PRINTF("\n");
}

/******************************************************************************/
/*!
    @brief  Helper function to display memory contents in a friendly format
*/
/******************************************************************************/
static void dump_str_line(uint8_t const* buf, uint16_t count)
{
  // each line is 16 bytes
  for(int i=0; i<count; i++)
  {
    const char ch = buf[i];
    PRINTF("%c", isprint(ch) ? ch : '.');
  }
}

void dbgDumpMemory(void const *buf, uint8_t size, uint16_t count, bool printOffset)
{
  if ( !buf || !count )
  {
    PRINTF("NULL\n");
    return;
  }

  uint8_t const *buf8 = (uint8_t const *) buf;

  char format[] = "%00lX";
  format[2] += 2*size;

  const uint8_t  item_per_line  = 16 / size;

  for(uint32_t i=0; i<count; i++)
  {
    uint32_t value=0;

    if ( i%item_per_line == 0 )
    {
      // Print Ascii
      if ( i != 0 )
      {
        PRINTF(" | ");
        dump_str_line(buf8-16, 16);
        PRINTF("\n");
      }

      // print offset or absolute address
      if (printOffset)
      {
        PRINTF("%03lX: ", 16*i/item_per_line);
      }else
      {
        PRINTF("%08lX:", (uint32_t) buf8);
      }
    }

    memcpy(&value, buf8, size);
    buf8 += size;

    PRINTF(" ");
    PRINTF(format, value);
  }

  // fill up last row to 16 for printing ascii
  const uint16_t remain = count%16;
  uint8_t nback = (remain ? remain : 16);

  if ( remain )
  {
    for(int i=0; i< 16-remain; i++)
    {
      PRINTF(" ");
      for(int j=0; j<2*size; j++) PRINTF(" ");
    }
  }

  PRINTF(" | ");
  dump_str_line(buf8-nback, nback);
  PRINTF("\n");

  PRINTF("\n");
}


void dbgDumpMemoryCFormat(const char* str, void const *buf, uint16_t count)
{
  if ( !buf )
  {
    PRINTF("NULL\n");
    return;
  }

  PRINTF("%s = \n{\n  ", str);

  uint8_t const *buf8 = (uint8_t const *) buf;

  for(uint32_t i=0; i<count; i++)
  {
    if ( i%16 == 0 )
    {
      if ( i != 0 ) PRINTF(",\n  ");
    }else
    {
      if ( i != 0 ) PRINTF(", ");
    }

    PRINTF("0x%02X", *buf8);
    buf8++;
  }

  PRINTF("\n};\n");
}



#if CFG_DEBUG

#include "ble.h"
#include "ble_hci.h"

/*------------------------------------------------------------------*/
/* BLE Event String
 *------------------------------------------------------------------*/
static lookup_entry_t const _strevt_lookup[] =
{
    // BLE common: 0x01
    { .key = BLE_EVT_USER_MEM_REQUEST                , .data= "BLE_EVT_USER_MEM_REQUEST"                },
    { .key = BLE_EVT_USER_MEM_RELEASE                , .data= "BLE_EVT_USER_MEM_RELEASE"                },

    // BLE Gap: 0x10
    { .key = BLE_GAP_EVT_CONNECTED                   , .data= "BLE_GAP_EVT_CONNECTED"                   },
    { .key = BLE_GAP_EVT_DISCONNECTED                , .data= "BLE_GAP_EVT_DISCONNECTED"                },
    { .key = BLE_GAP_EVT_CONN_PARAM_UPDATE           , .data= "BLE_GAP_EVT_CONN_PARAM_UPDATE"           },
    { .key = BLE_GAP_EVT_SEC_PARAMS_REQUEST          , .data= "BLE_GAP_EVT_SEC_PARAMS_REQUEST"          },
    { .key = BLE_GAP_EVT_SEC_INFO_REQUEST            , .data= "BLE_GAP_EVT_SEC_INFO_REQUEST"            },
    { .key = BLE_GAP_EVT_PASSKEY_DISPLAY             , .data= "BLE_GAP_EVT_PASSKEY_DISPLAY"             },
    { .key = BLE_GAP_EVT_KEY_PRESSED                 , .data= "BLE_GAP_EVT_KEY_PRESSED"                 },
    { .key = BLE_GAP_EVT_AUTH_KEY_REQUEST            , .data= "BLE_GAP_EVT_AUTH_KEY_REQUEST"            },
    { .key = BLE_GAP_EVT_LESC_DHKEY_REQUEST          , .data= "BLE_GAP_EVT_LESC_DHKEY_REQUEST"          },
    { .key = BLE_GAP_EVT_AUTH_STATUS                 , .data= "BLE_GAP_EVT_AUTH_STATUS"                 },
    { .key = BLE_GAP_EVT_CONN_SEC_UPDATE             , .data= "BLE_GAP_EVT_CONN_SEC_UPDATE"             },
    { .key = BLE_GAP_EVT_TIMEOUT                     , .data= "BLE_GAP_EVT_TIMEOUT"                     },
    { .key = BLE_GAP_EVT_RSSI_CHANGED                , .data= "BLE_GAP_EVT_RSSI_CHANGED"                },
    { .key = BLE_GAP_EVT_ADV_REPORT                  , .data= "BLE_GAP_EVT_ADV_REPORT"                  },
    { .key = BLE_GAP_EVT_SEC_REQUEST                 , .data= "BLE_GAP_EVT_SEC_REQUEST"                 },
    { .key = BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST   , .data= "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST"   },
    { .key = BLE_GAP_EVT_SCAN_REQ_REPORT             , .data= "BLE_GAP_EVT_SCAN_REQ_REPORT"             },
    { .key = BLE_GAP_EVT_PHY_UPDATE_REQUEST          , .data= "BLE_GAP_EVT_PHY_UPDATE_REQUEST"          },
    { .key = BLE_GAP_EVT_PHY_UPDATE                  , .data= "BLE_GAP_EVT_PHY_UPDATE"                  },
    { .key = BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST  , .data= "BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST"  },
    { .key = BLE_GAP_EVT_DATA_LENGTH_UPDATE          , .data= "BLE_GAP_EVT_DATA_LENGTH_UPDATE"          },
    { .key = BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT   , .data= "BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT"   },
    { .key = BLE_GAP_EVT_ADV_SET_TERMINATED          , .data= "BLE_GAP_EVT_ADV_SET_TERMINATED"          },

    // BLE Gattc: 0x30
    { .key = BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP        , .data= "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP"        },
    { .key = BLE_GATTC_EVT_REL_DISC_RSP              , .data= "BLE_GATTC_EVT_REL_DISC_RSP"              },
    { .key = BLE_GATTC_EVT_CHAR_DISC_RSP             , .data= "BLE_GATTC_EVT_CHAR_DISC_RSP"             },
    { .key = BLE_GATTC_EVT_DESC_DISC_RSP             , .data= "BLE_GATTC_EVT_DESC_DISC_RSP"             },
    { .key = BLE_GATTC_EVT_ATTR_INFO_DISC_RSP        , .data= "BLE_GATTC_EVT_ATTR_INFO_DISC_RSP"        },
    { .key = BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP , .data= "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP" },
    { .key = BLE_GATTC_EVT_READ_RSP                  , .data= "BLE_GATTC_EVT_READ_RSP"                  },
    { .key = BLE_GATTC_EVT_CHAR_VALS_READ_RSP        , .data= "BLE_GATTC_EVT_CHAR_VALS_READ_RSP"        },
    { .key = BLE_GATTC_EVT_WRITE_RSP                 , .data= "BLE_GATTC_EVT_WRITE_RSP"                 },
    { .key = BLE_GATTC_EVT_HVX                       , .data= "BLE_GATTC_EVT_HVX"                       },
    { .key = BLE_GATTC_EVT_EXCHANGE_MTU_RSP          , .data= "BLE_GATTC_EVT_EXCHANGE_MTU_RSP"          },
    { .key = BLE_GATTC_EVT_TIMEOUT                   , .data= "BLE_GATTC_EVT_TIMEOUT"                   },
    { .key = BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE     , .data= "BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE"     },

    // BLE Gatts: 0x50
    { .key = BLE_GATTS_EVT_WRITE                     , .data= "BLE_GATTS_EVT_WRITE"                     },
    { .key = BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST      , .data= "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST"      },
    { .key = BLE_GATTS_EVT_SYS_ATTR_MISSING          , .data= "BLE_GATTS_EVT_SYS_ATTR_MISSING"          },
    { .key = BLE_GATTS_EVT_HVC                       , .data= "BLE_GATTS_EVT_HVC"                       },
    { .key = BLE_GATTS_EVT_SC_CONFIRM                , .data= "BLE_GATTS_EVT_SC_CONFIRM"                },
    { .key = BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST      , .data= "BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST"      },
    { .key = BLE_GATTS_EVT_TIMEOUT                   , .data= "BLE_GATTS_EVT_TIMEOUT"                   },
    { .key = BLE_GATTS_EVT_HVN_TX_COMPLETE           , .data= "BLE_GATTS_EVT_HVN_TX_COMPLETE"           },
};

lookup_table_t const _strevt_table =
{
  .count = arrcount(_strevt_lookup),
  .items = _strevt_lookup
};

const char* dbg_ble_event_str(uint16_t evt_id)
{
  const char * str = (const char *) lookup_find(&_strevt_table, evt_id);
  static char unknown_str[7] = {0};

  if ( str == NULL )
  {
    sprintf(unknown_str, "0x%04X", evt_id);
    str = unknown_str;
  }

  return str;
}

/*------------------------------------------------------------------*/
/* Error String
 *------------------------------------------------------------------*/
static lookup_entry_t const _strerr_lookup[] =
{
    // General: 0x0000
    { .key = NRF_ERROR_SVC_HANDLER_MISSING                     , .data = "NRF_ERROR_SVC_HANDLER_MISSING"                     },
    { .key = NRF_ERROR_SOFTDEVICE_NOT_ENABLED                  , .data = "NRF_ERROR_SOFTDEVICE_NOT_ENABLED"                  },
    { .key = NRF_ERROR_INTERNAL                                , .data = "NRF_ERROR_INTERNAL"                                },
    { .key = NRF_ERROR_NO_MEM                                  , .data = "NRF_ERROR_NO_MEM"                                  },
    { .key = NRF_ERROR_NOT_FOUND                               , .data = "NRF_ERROR_NOT_FOUND"                               },
    { .key = NRF_ERROR_NOT_SUPPORTED                           , .data = "NRF_ERROR_NOT_SUPPORTED"                           },
    { .key = NRF_ERROR_INVALID_PARAM                           , .data = "NRF_ERROR_INVALID_PARAM"                           },
    { .key = NRF_ERROR_INVALID_STATE                           , .data = "NRF_ERROR_INVALID_STATE"                           },
    { .key = NRF_ERROR_INVALID_LENGTH                          , .data = "NRF_ERROR_INVALID_LENGTH"                          },
    { .key = NRF_ERROR_INVALID_FLAGS                           , .data = "NRF_ERROR_INVALID_FLAGS"                           },
    { .key = NRF_ERROR_INVALID_DATA                            , .data = "NRF_ERROR_INVALID_DATA"                            },
    { .key = NRF_ERROR_DATA_SIZE                               , .data = "NRF_ERROR_DATA_SIZE"                               },
    { .key = NRF_ERROR_TIMEOUT                                 , .data = "NRF_ERROR_TIMEOUT"                                 },
    { .key = NRF_ERROR_NULL                                    , .data = "NRF_ERROR_NULL"                                    },
    { .key = NRF_ERROR_FORBIDDEN                               , .data = "NRF_ERROR_FORBIDDEN"                               },
    { .key = NRF_ERROR_INVALID_ADDR                            , .data = "NRF_ERROR_INVALID_ADDR"                            },
    { .key = NRF_ERROR_BUSY                                    , .data = "NRF_ERROR_BUSY"                                    },
    { .key = NRF_ERROR_CONN_COUNT                              , .data = "NRF_ERROR_CONN_COUNT"                              },
    { .key = NRF_ERROR_RESOURCES                               , .data = "NRF_ERROR_RESOURCES"                               },

    // SDM: 0x1000
    { .key = NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN                , .data = "NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN"                },
    { .key = NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION   , .data = "NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION"   },
    { .key = NRF_ERROR_SDM_INCORRECT_CLENR0                    , .data = "NRF_ERROR_SDM_INCORRECT_CLENR0"                    },

    // SOC: 0x2000
    { .key = NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN                 , .data = "NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN"                 },
    { .key = NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE        , .data = "NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE"        },
    { .key = NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED , .data = "NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED" },
    { .key = NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN              , .data = "NRF_ERROR_SOC_NVIC_SHOULD_NOT_RETURN"              },
    { .key = NRF_ERROR_SOC_POWER_MODE_UNKNOWN                  , .data = "NRF_ERROR_SOC_POWER_MODE_UNKNOWN"                  },
    { .key = NRF_ERROR_SOC_POWER_POF_THRESHOLD_UNKNOWN         , .data = "NRF_ERROR_SOC_POWER_POF_THRESHOLD_UNKNOWN"         },
    { .key = NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN         , .data = "NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN"         },
    { .key = NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES              , .data = "NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES"              },
    { .key = NRF_ERROR_SOC_PPI_INVALID_CHANNEL                 , .data = "NRF_ERROR_SOC_PPI_INVALID_CHANNEL"                 },
    { .key = NRF_ERROR_SOC_PPI_INVALID_GROUP                   , .data = "NRF_ERROR_SOC_PPI_INVALID_GROUP"                   },

    // BLE Generic: 0x2000
    { .key = BLE_ERROR_NOT_ENABLED                             , .data = "BLE_ERROR_NOT_ENABLED"                             },
    { .key = BLE_ERROR_INVALID_CONN_HANDLE                     , .data = "BLE_ERROR_INVALID_CONN_HANDLE"                     },
    { .key = BLE_ERROR_INVALID_ATTR_HANDLE                     , .data = "BLE_ERROR_INVALID_ATTR_HANDLE"                     },
    { .key = BLE_ERROR_INVALID_ADV_HANDLE                      , .data = "BLE_ERROR_INVALID_ADV_HANDLE"                      },
    { .key = BLE_ERROR_INVALID_ROLE                            , .data = "BLE_ERROR_INVALID_ROLE"                            },
    { .key = BLE_ERROR_BLOCKED_BY_OTHER_LINKS                  , .data = "BLE_ERROR_BLOCKED_BY_OTHER_LINKS"                  },

    // BLE GAP: 0x2200
    { .key = BLE_ERROR_GAP_UUID_LIST_MISMATCH                  , .data = "BLE_ERROR_GAP_UUID_LIST_MISMATCH"                  },
    { .key = BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST         , .data = "BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST"         },
    { .key = BLE_ERROR_GAP_INVALID_BLE_ADDR                    , .data = "BLE_ERROR_GAP_INVALID_BLE_ADDR"                    },
    { .key = BLE_ERROR_GAP_WHITELIST_IN_USE                    , .data = "BLE_ERROR_GAP_WHITELIST_IN_USE"                    },
    { .key = BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE            , .data = "BLE_ERROR_GAP_DEVICE_IDENTITIES_IN_USE"            },
    { .key = BLE_ERROR_GAP_DEVICE_IDENTITIES_DUPLICATE         , .data = "BLE_ERROR_GAP_DEVICE_IDENTITIES_DUPLICATE"         },

    // BLE GATTC: 0x2300
    { .key = BLE_ERROR_GATTC_PROC_NOT_PERMITTED                , .data = "BLE_ERROR_GATTC_PROC_NOT_PERMITTED"                },

    // BLE GATTS: 0x2400
    { .key = BLE_ERROR_GATTS_INVALID_ATTR_TYPE                 , .data = "BLE_ERROR_GATTS_INVALID_ATTR_TYPE"                 },
    { .key = BLE_ERROR_GATTS_SYS_ATTR_MISSING                  , .data = "BLE_ERROR_GATTS_SYS_ATTR_MISSING"                  },
};

lookup_table_t const _strerr_table =
{
  .count = arrcount(_strerr_lookup),
  .items = _strerr_lookup
};

const char* dbg_err_str(int32_t err_id)
{
  // TODO: verify it would be safe to change to int16_t
  const char * str = (const char *) lookup_find(&_strerr_table, err_id);
  static char unknown_str[7] = {0};

  if ( str == NULL )
  {
    sprintf(unknown_str, "0x%04lX", (uint32_t)err_id);
    str = unknown_str;
  }

  return str;
}

//--------------------------------------------------------------------+
// HCI STATUS
//--------------------------------------------------------------------+
static lookup_entry_t const _strhci_lookup[] =
{
  { .key = BLE_HCI_STATUS_CODE_SUCCESS                         , .data = "STATUS_CODE_SUCCESS"                         },
  { .key = BLE_HCI_STATUS_CODE_UNKNOWN_BTLE_COMMAND            , .data = "STATUS_CODE_UNKNOWN_BTLE_COMMAND "           },
  { .key = BLE_HCI_STATUS_CODE_UNKNOWN_CONNECTION_IDENTIFIER   , .data = "STATUS_CODE_UNKNOWN_CONNECTION_IDENTIFIER"   },
  { .key = BLE_HCI_AUTHENTICATION_FAILURE                      , .data = "AUTHENTICATION_FAILURE "                     },
  { .key = BLE_HCI_STATUS_CODE_PIN_OR_KEY_MISSING              , .data = "STATUS_CODE_PIN_OR_KEY_MISSING "             },
  { .key = BLE_HCI_MEMORY_CAPACITY_EXCEEDED                    , .data = "MEMORY_CAPACITY_EXCEEDED "                   },
  { .key = BLE_HCI_CONNECTION_TIMEOUT                          , .data = "CONNECTION_TIMEOUT "                         },
  { .key = BLE_HCI_STATUS_CODE_COMMAND_DISALLOWED              , .data = "STATUS_CODE_COMMAND_DISALLOWED "             },
  { .key = BLE_HCI_STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS , .data = "STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS" },
  { .key = BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION           , .data = "REMOTE_USER_TERMINATED_CONNECTION"           },
  { .key = BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES , .data = "REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES" },
  { .key = BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF     , .data = "REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF"     },
  { .key = BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION            , .data = "LOCAL_HOST_TERMINATED_CONNECTION "           },
  { .key = BLE_HCI_UNSUPPORTED_REMOTE_FEATURE                  , .data = "UNSUPPORTED_REMOTE_FEATURE"                  },
  { .key = BLE_HCI_STATUS_CODE_INVALID_LMP_PARAMETERS          , .data = "STATUS_CODE_INVALID_LMP_PARAMETERS "         },
  { .key = BLE_HCI_STATUS_CODE_UNSPECIFIED_ERROR               , .data = "STATUS_CODE_UNSPECIFIED_ERROR"               },
  { .key = BLE_HCI_STATUS_CODE_LMP_RESPONSE_TIMEOUT            , .data = "STATUS_CODE_LMP_RESPONSE_TIMEOUT "           },
  { .key = BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION , .data = "STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION" },
  { .key = BLE_HCI_STATUS_CODE_LMP_PDU_NOT_ALLOWED             , .data = "STATUS_CODE_LMP_PDU_NOT_ALLOWED"             },
  { .key = BLE_HCI_INSTANT_PASSED                              , .data = "INSTANT_PASSED "                             },
  { .key = BLE_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED           , .data = "PAIRING_WITH_UNIT_KEY_UNSUPPORTED"           },
  { .key = BLE_HCI_DIFFERENT_TRANSACTION_COLLISION             , .data = "DIFFERENT_TRANSACTION_COLLISION"             },
  { .key = BLE_HCI_PARAMETER_OUT_OF_MANDATORY_RANGE            , .data = "PARAMETER_OUT_OF_MANDATORY_RANGE "           },
  { .key = BLE_HCI_CONTROLLER_BUSY                             , .data = "CONTROLLER_BUSY"                             },
  { .key = BLE_HCI_CONN_INTERVAL_UNACCEPTABLE                  , .data = "CONN_INTERVAL_UNACCEPTABLE "                 },
  { .key = BLE_HCI_DIRECTED_ADVERTISER_TIMEOUT                 , .data = "DIRECTED_ADVERTISER_TIMEOUT"                 },
  { .key = BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE          , .data = "CONN_TERMINATED_DUE_TO_MIC_FAILURE "         },
  { .key = BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED               , .data = "CONN_FAILED_TO_BE_ESTABLISHED"               }
};

lookup_table_t const _strhci_table =
{
  .count = arrcount(_strhci_lookup),
  .items = _strhci_lookup
};


const char* dbg_hci_str(uint8_t id)
{
  return (const char *) lookup_find(&_strhci_table, id);
}

#endif

}
