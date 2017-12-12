/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

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

#include "Arduino.h"
#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOOTLOADER_DFU_SERIAL_MAGIC         0x4e
#define BOOTLOADER_DFU_OTA_FULLRESET_MAGIC  0xA8

// Must match temp register in bootloader
#define BOOTLOADER_VERSION_REGISTER     NRF_TIMER2->CC[0]
uint32_t bootloaderVersion = 0;

void init( void )
{
  // Retrieve bootloader version
  bootloaderVersion = BOOTLOADER_VERSION_REGISTER;

#if defined( USE_LFXO )
  // 32Khz XTAL
  NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
#elif defined(USE_LFRC)
  // Internal OSC
  NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
#else
  #error Clock Source is not configured, define USE_LFXO or USE_LFRC according to your board
#endif

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  // RTC1 could be enabled by bootloader. Disable it
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;
}

void enterSerialDfu(void)
{
  NRF_POWER->GPREGRET = BOOTLOADER_DFU_SERIAL_MAGIC;
  NVIC_SystemReset();
}

void enterOTADfu(void)
{
  NRF_POWER->GPREGRET = BOOTLOADER_DFU_OTA_FULLRESET_MAGIC;
  NVIC_SystemReset();
}

void waitForEvent(void)
{
#if 0
  // Set bit 7 and bits 4..0 in the mask to one (0x ...00 1001 1111)
  enum { FPU_EXCEPTION_MASK = 0x0000009F };

  /* Clear exceptions and PendingIRQ from the FPU unit */
  __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
#endif

  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

  if ( sd_en )
  {
    (void) sd_app_evt_wait();
  }else
  {
    // SoftDevice is not enabled.
    __WFE();
    __SEV(); // Clear Event Register.
    __WFE();
  }
}


void systemOff(uint32_t pin, uint8_t wake_logic)
{
//  for(int i=0; i<8; i++)
//  {
//    NRF_POWER->RAM[i].POWERCLR = 0x03UL;
//  }

#if 0
  // pin 0 & 1 is for XTAL
  for(int i=2; i<PINS_COUNT; i++)
  {
    pinMode(i, INPUT);
  }
#endif

  pinMode(pin, wake_logic ? INPUT_PULLDOWN : INPUT_PULLUP);
  NRF_GPIO->PIN_CNF[pin] |= ((uint32_t) (wake_logic ? GPIO_PIN_CNF_SENSE_High : GPIO_PIN_CNF_SENSE_Low) << GPIO_PIN_CNF_SENSE_Pos);

  uint8_t sd_en;
  (void) sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if ( sd_en )
  {
    sd_power_system_off();
  }else
  {
    NRF_POWER->SYSTEMOFF = 1;
  }
}

#ifdef __cplusplus
}
#endif
