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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Bootloader version. Assigned by init().
 * - value of 0x000500 is version 0.5.0
 */
extern uint32_t bootloaderVersion;

extern void init(void);

void enterSerialDfu(void);
void enterOTADfu(void);
void enterUf2Dfu(void);

void waitForEvent(void);
void systemOff(uint32_t pin, uint8_t wake_logic);

// Test if in interrupt mode
static inline bool isInISR(void)
{
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}

#ifdef __cplusplus
}
#endif
