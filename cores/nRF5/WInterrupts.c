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

#include <nrf.h>

#include "Arduino.h"
#include "wiring_private.h"

#include <string.h>

#if defined(NRF52) || defined(NRF52_SERIES)
#define NUMBER_OF_GPIO_TE 8
#else
#define NUMBER_OF_GPIO_TE 4
#endif

#ifdef GPIOTE_CONFIG_PORT_Msk
#define GPIOTE_CONFIG_PORT_PIN_Msk (GPIOTE_CONFIG_PORT_Msk | GPIOTE_CONFIG_PSEL_Msk)
#else
#define GPIOTE_CONFIG_PORT_PIN_Msk GPIOTE_CONFIG_PSEL_Msk
#endif

static voidFuncPtr callbacksInt[NUMBER_OF_GPIO_TE];
static bool callbackDeferred[NUMBER_OF_GPIO_TE];
static int8_t channelMap[NUMBER_OF_GPIO_TE];
static int enabled = 0;

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(callbacksInt, 0, sizeof(callbacksInt));
  memset(channelMap, -1, sizeof(channelMap));
  memset(callbackDeferred, 0, sizeof(callbackDeferred));

  NVIC_DisableIRQ(GPIOTE_IRQn);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 3);
  NVIC_EnableIRQ(GPIOTE_IRQn);
}

/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 *
 * \return Interrupt Mask
 */
int attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  if (pin >= PINS_COUNT) {
    return 0;
  }

  pin = g_ADigitalPinMap[pin];

  bool deferred = (mode & ISR_DEFERRED) ? true : false;
  mode &= ~ISR_DEFERRED;

  uint32_t polarity;

  switch (mode) {
    case CHANGE:
      polarity = GPIOTE_CONFIG_POLARITY_Toggle;
      break;

    case FALLING:
      polarity = GPIOTE_CONFIG_POLARITY_HiToLo;
      break;

    case RISING:
      polarity = GPIOTE_CONFIG_POLARITY_LoToHi;
      break;

    default:
      return 0;
  }

  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    if (channelMap[ch] == -1 || (uint32_t)channelMap[ch] == pin) {
      channelMap[ch] = pin;
      callbacksInt[ch] = callback;
      callbackDeferred[ch] = deferred;

      NRF_GPIOTE->CONFIG[ch] &= ~(GPIOTE_CONFIG_PORT_PIN_Msk | GPIOTE_CONFIG_POLARITY_Msk);
      NRF_GPIOTE->CONFIG[ch] |= ((pin << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk) |
                              ((polarity << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk);

      NRF_GPIOTE->CONFIG[ch] |= GPIOTE_CONFIG_MODE_Event;

      NRF_GPIOTE->INTENSET = (1 << ch);

      return (1 << ch);
    }
  }

  return 0;
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
  if (pin >= PINS_COUNT) {
    return;
  }

  pin = g_ADigitalPinMap[pin];

  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    if ((uint32_t)channelMap[ch] == pin) {
      channelMap[ch] = -1;
      callbacksInt[ch] = NULL;
      callbackDeferred[ch] = false;

      NRF_GPIOTE->CONFIG[ch] &= ~GPIOTE_CONFIG_MODE_Event;

      NRF_GPIOTE->INTENCLR = (1 << ch);

      break;
    }
  }
}

void GPIOTE_IRQHandler()
{
#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  uint32_t event = offsetof(NRF_GPIOTE_Type, EVENTS_IN[0]);

  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    if ((*(uint32_t *)((uint32_t)NRF_GPIOTE + event) == 0x1UL) && (NRF_GPIOTE->INTENSET & (1 << ch))) {
      if (channelMap[ch] != -1 && callbacksInt[ch]) {
        if ( callbackDeferred[ch] )  {
          // Adafruit defer callback to non-isr if configured so
          ada_callback(NULL, 0, callbacksInt[ch]);
        }else{
         callbacksInt[ch]();
        }
      }

    *(uint32_t *)((uint32_t)NRF_GPIOTE + event) = 0;
#if __CORTEX_M == 0x04
    volatile uint32_t dummy = *((volatile uint32_t *)((uint32_t)NRF_GPIOTE + event));
    (void)dummy;
#endif
    }

    event = (uint32_t)((uint32_t)event + 4);
  }

#if CFG_SYSVIEW
    SEGGER_SYSVIEW_RecordExitISR();
#endif
}
