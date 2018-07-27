/**************************************************************************/
/*!
    @file     bfutil.c (Bluefruit Util)
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <assert.h>
#include <string.h>

#include "libusb-1.0/libusb.h"

#if defined(_WIN32)

#include <windows.h>
static inline void sleep_ms(int ms)
{
  Sleep(ms);
}


#else

#include <unistd.h>
static inline void sleep_ms(int ms)
{
  usleep(ms*1000);
}

#endif

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
#define ASSERT(x)   assert( (x) == LIBUSB_SUCCESS )

enum
{
  RESET_DELAY     = 1000, // 1 second delay
  RESET_MSC_DELAY = 2000, // MSC available require more time for USB to enumerate
};

enum
{
  ARDUINO_ADDR    = 0x080C0000,
  FEATHERLIB_ADDR = 0x08010000
};

enum
{
  USB_VID     = 0xcafe,
  USB_PID     = 0x4011,

  USB_PID_MSC = 0x8010,
  USB_DFU_PID = 0x0008,
};

// SDEP Command Type
enum
{
  SDEP_MSGTYPE_COMMAND     = 0x10,
  SDEP_MSGTYPE_RESPONSE    = 0x20,
  SDEP_MSGTYPE_ALERT       = 0x40,
  SDEP_MSGTYPE_ERROR       = 0x80,
};

// SDEP Command Constant
enum
{
  // General Purpose
  SDEP_CMD_RESET               = 0x0001,    // HW reset
  SDEP_CMD_FACTORYRESET        = 0x0002,    // Factory reset
  SDEP_CMD_DFU                 = 0x0003,    // Enter DFU mode
  SDEP_CMD_INFO                = 0x0004,    // System information
  SDEP_CMD_NVM_RESET           = 0x0005,    // Reset DCT
  SDEP_CMD_ERROR_STRING        = 0x0006,    // Get descriptive error string
  SDEP_CMD_COMMAND_STRING      = 0x0007,    // Get descriptive SDEP command string
  SDEP_CMD_SFLASH_ERASEALL     = 0x0008,    // Erase SPI flash memory

  // DEBUG Commands
  SDEP_CMD_STACKDUMP           = 0x0300,    // Dump the stack
  SDEP_CMD_STACKSIZE           = 0x0301,    // Get stack size
  SDEP_CMD_HEAPDUMP            = 0x0302,    // Dump the heap
  SDEP_CMD_HEAPSIZE            = 0x0303,    // Get heap size
  SDEP_CMD_THREADLIST_PRINT    = 0x0304,    // Get thread information
  SDEP_CMD_THREAD_INFO         = 0x0305,    // Get thread info
  SDEP_CMD_ARDUINO_HEAPSIZE    = 0x0306,    // Get Arduino heap size
};


/*------------------------------------------------------------------*/
/* VARIABLE DECLARATION
 *------------------------------------------------------------------*/
libusb_device_handle* udev = NULL;

bool _dfu_mode = false;
bool _msc_enbled = false;

/*------------------------------------------------------------------*/
/* FUNCTION DECLARATION
 *------------------------------------------------------------------*/
void print_help(void);
void sdep_syscmd(uint16_t cmd);

void arduino_upgrade(char* dfu_util, char* binfile);
void featherlib_upgrade(char* dfu_util, char* binfile);

struct {
  const char* command;
  uint16_t sdep_cmdid;
  void (*func) (char*, char*);
}cmd_table[] =
{
  { "arduino_upgrade"    , 0                        , arduino_upgrade    },
  { "enter_dfu"          , SDEP_CMD_DFU             , NULL               },
  { "erase_spiflash"     , SDEP_CMD_SFLASH_ERASEALL , NULL               },
  { "factory_reset"      , SDEP_CMD_FACTORYRESET    , NULL               },
  { "featherlib_upgrade" , 0                        , featherlib_upgrade },
  { "info"               , SDEP_CMD_INFO            , NULL               },
  { "nvm_reset"          , SDEP_CMD_NVM_RESET       , NULL               },
  { "reboot"             , SDEP_CMD_RESET           , NULL               },
};

/*------------------------------------------------------------------*/
/* IMPLEMENTATION
 *------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
  char* command = argv[1];

  if ( argc <= 1 || (!strcmp("--help", command)) )
  {
    print_help();
    return 0;
  }

  ASSERT( libusb_init(NULL) );

  // Open device, try with all PIDs
  udev = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID);

  if (!udev)
  {
    udev = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID_MSC);
    if (udev) _msc_enbled = true;
  }

  if (!udev)
  {
    udev = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_DFU_PID);
    if(udev) _dfu_mode = true;
  }

  if (!udev)
  {
    printf("Bluefruit Feather Board is not detected !!!\n");
    return 0;
  }

  printf("Bluefruit Feather Board opened !!!\n");
  return 0;

  for(int i=0; i<sizeof(cmd_table)/sizeof(cmd_table[0]); i++)
  {
    if (!strcmp(cmd_table[i].command, command))
    {
      // Execute SDEP commnad by ID
      if ( cmd_table[i].sdep_cmdid )
      {
        // skip if command is ENTER_DFU and we are already in dfu mode
        if ( !((cmd_table[i].sdep_cmdid == SDEP_CMD_DFU) && _dfu_mode) )
        {
          sdep_syscmd(cmd_table[i].sdep_cmdid);
        }
      }
      // Execute function by pointer
      else if ( cmd_table[i].func )
      {
//        cmd_table[i].func();
      }else
      {
        assert(0);
      }

      break;
    }
  }

  // close usb if not closed already
  if (udev)
  {
    libusb_close(udev);
    libusb_exit(NULL);
  }

  return 0;
}

/*------------------------------------------------------------------*/
/*
 *------------------------------------------------------------------*/

void sdep_syscmd(uint16_t cmd)
{
  assert( cmd <= SDEP_CMD_SFLASH_ERASEALL);

  ASSERT( libusb_control_transfer(udev, 0x40, SDEP_MSGTYPE_COMMAND, cmd, 0, NULL, 0, 0) );

  // For system commands, only INFO has response data
  if ( cmd == SDEP_CMD_INFO )
  {
    char info[512] = {  0 };
    int len;

    sleep_ms(100);
    len = libusb_control_transfer(udev, 0xC0, SDEP_MSGTYPE_RESPONSE, cmd, 0, (unsigned char*) info, sizeof(info), 0);

    if ( len <= 0 )
    {
      printf("error = %d\n", len);
    }else
    {
      printf("%s\n", info+4);
    }
  }
}

void download_bin(char* dfu_util, char* binfile, uint32_t addr)
{
  // Enter DFU mode first
  if (!_dfu_mode)
  {
    sdep_syscmd(SDEP_CMD_DFU);

    // Close libusb since dfu-util need to access to upload
    libusb_close(udev);
    libusb_exit(NULL);
    udev = NULL;

    sleep_ms(RESET_DELAY);
  }

  char cmd[1024];
  sprintf(cmd, "%s -d 0x239A:0x0008 -a 0 -s 0x%08X:leave -D \"%s\"", dfu_util, addr, binfile);

  system(cmd);

  // Also print out the dfu-util command for debug purpose
  puts("Flash command");
  puts(cmd);

  // Delay a bit before return, otherwise enumeration time will cause
  // Arduino IDE fails to re-open Serial
  sleep_ms ( _msc_enbled ? RESET_MSC_DELAY : RESET_DELAY );
}

void arduino_upgrade(char* dfu_util, char* binfile)
{
  download_bin(dfu_util, binfile, ARDUINO_ADDR);
}

void featherlib_upgrade(char* dfu_util, char* binfile)
{
  download_bin(dfu_util, binfile, FEATHERLIB_ADDR);
}


void print_help(void)
{
  printf("Usage: feather_dfu.py [OPTIONS] COMMAND [ARGS]...\n\n");
  printf("Options:\n");
  printf("  --help               Show this message and exit.\n\n");

  printf("Commands:\n");
//  printf("  enter_dfu           Force board into DFU mode.\n");
  printf("  factory_reset       Perform a factory reset of the board.\n");
  printf("  info                Print information about the connected board.\n");
  printf("  reboot              Perform a reboot/reset of the board.\n");
}
