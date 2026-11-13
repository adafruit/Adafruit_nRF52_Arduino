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
#include "nrf_gpiote.h"

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

static voidFuncPtrArg callbacksInt[NUMBER_OF_GPIO_TE];
static void* callbackArgs[NUMBER_OF_GPIO_TE];
static bool callbackDeferred[NUMBER_OF_GPIO_TE];
static int8_t channelMap[NUMBER_OF_GPIO_TE];
static int enabled = 0;

/* Configure I/O interrupt sources */
static void __initialize()
{
  memset(callbacksInt, 0, sizeof(callbacksInt));
  memset(callbackArgs, 0, sizeof(callbackArgs));
  memset(channelMap, -1, sizeof(channelMap));
  memset(callbackDeferred, 0, sizeof(callbackDeferred));

  NVIC_DisableIRQ(GPIOTE_IRQn);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_SetPriority(GPIOTE_IRQn, 3);
  NVIC_EnableIRQ(GPIOTE_IRQn);
}

/* Trampoline storing a no-argument callback as a one-argument one, so a single
 * code path (and a single callbacksInt[]) can serve attachInterrupt() and
 * attachInterruptArg() alike. */
static void _callNoArgCallback(void *callback)
{
  ((voidFuncPtr) callback)();
}

/* taskENTER_CRITICAL()/taskEXIT_CRITICAL() must not be called from interrupt context: unlike
 * the _FROM_ISR() pair, exiting them unconditionally zeroes a nesting counter and clears
 * BASEPRI, which can turn interrupts back on from underneath an outer ISR-side critical
 * section before that caller is done with it. Pick the ISR-safe pair when called from one,
 * so attachInterruptArg()/detachInterrupt() are safe whether called from a task or an ISR
 * (e.g. a non-deferred callback detaching itself). */
static UBaseType_t _criticalEnter(bool *fromISR)
{
  *fromISR = isInISR();
  if (*fromISR) {
    return taskENTER_CRITICAL_FROM_ISR();
  }
  taskENTER_CRITICAL();
  return 0;
}

static void _criticalExit(bool fromISR, UBaseType_t savedMask)
{
  if (fromISR) {
    taskEXIT_CRITICAL_FROM_ISR(savedMask);
  } else {
    taskEXIT_CRITICAL();
  }
}

/*
 * \brief Specifies a named Interrupt Service Routine (ISR), along with an argument to pass
 *        it, to call when an interrupt occurs. Replaces any previous function that was
 *        attached to the interrupt.
 *
 * \return Interrupt Mask
 */
int attachInterruptArg(uint32_t pin, voidFuncPtrArg handler, void *arg, uint32_t mode)
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

  // All information for the configuration is known, except the prior values
  // of the config register.  Pre-compute the mask and new bits for later use.
  //     CONFIG[n] = (CONFIG[n] & oldRegMask) | newRegBits;
  //
  // Three fields are configured here: PORT/PIN, POLARITY, MODE
  const uint32_t oldRegMask = ~(GPIOTE_CONFIG_PORT_PIN_Msk | GPIOTE_CONFIG_POLARITY_Msk | GPIOTE_CONFIG_MODE_Msk);
  const uint32_t newRegBits =
    ((pin                      << GPIOTE_CONFIG_PSEL_Pos    ) & GPIOTE_CONFIG_PORT_PIN_Msk) |
    ((polarity                 << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
    ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos    ) & GPIOTE_CONFIG_MODE_Msk    ) ;

  // Hold the channel scan, selection, and configuration all under one critical section, so
  // a concurrent attach/detach (from another task or an ISR) can't race this call to the
  // same channel, and GPIOTE_IRQHandler never observes a channel half-applied. This nests
  // correctly with any critical section (FreeRTOS's or the SoftDevice's) the caller may
  // already be in, unlike disabling/re-enabling GPIOTE_IRQn directly would.
  bool criticalFromISR;
  UBaseType_t const criticalMask = _criticalEnter(&criticalFromISR);

  int ch = -1;
  int newChannel = 0;

  // Find channel where pin is already assigned, if any
  for (int i = 0; i < NUMBER_OF_GPIO_TE; i++) {
    if ((uint32_t)channelMap[i] != pin) continue;
    ch = i;
    break;
  }
  // else, find one not already mapped and also not in use by others
  if (ch == -1) {
    for (int i = 0; i < NUMBER_OF_GPIO_TE; i++) {
      if (channelMap[i] != -1) continue;
      if (nrf_gpiote_te_is_enabled(NRF_GPIOTE, i)) continue;
      
      ch = i;
      newChannel = 1;
      break;
    }
  }
  // if no channel found, exit
  if (ch == -1) {
    _criticalExit(criticalFromISR, criticalMask);
    return 0; // no channel available
  }

  channelMap[ch]       = pin;      // harmless for existing channel
  callbacksInt[ch]     = handler;  // caller might be updating this for existing channel
  callbackArgs[ch]     = arg;      // caller might be updating this for existing channel
  callbackDeferred[ch] = deferred; // caller might be updating this for existing channel

  uint32_t tmp = NRF_GPIOTE->CONFIG[ch];
  tmp &= oldRegMask;
  tmp |= newRegBits;                 // for existing channel, effectively updates only the polarity
  NRF_GPIOTE->CONFIG[ch] = tmp;

  // For a new channel, additionally ensure no old events existed, and enable the interrupt
  if (newChannel) {
    NRF_GPIOTE->EVENTS_IN[ch] = 0;
    NRF_GPIOTE->INTENSET = (1 << ch);
  }

  _criticalExit(criticalFromISR, criticalMask);

  // Finally, indicate to caller the allocated / updated channel
  return (1 << ch);
}

/*
 * \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 *        Replaces any previous function that was attached to the interrupt.
 *
 * \return Interrupt Mask
 */
int attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
  return attachInterruptArg(pin, callback ? _callNoArgCallback : NULL, (void *) callback, mode);
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

  // Same protection as attachInterruptArg(): hold the channel scan and teardown under one
  // critical section, so neither GPIOTE_IRQHandler nor another task/ISR can race it.
  bool criticalFromISR;
  UBaseType_t const criticalMask = _criticalEnter(&criticalFromISR);

  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    if ((uint32_t)channelMap[ch] == pin) {
      NRF_GPIOTE->INTENCLR = (1 << ch);
      NRF_GPIOTE->CONFIG[ch] = 0;
      NRF_GPIOTE->EVENTS_IN[ch] = 0; // clear any final events

      // now cleanup the rest of the use of the channel
      channelMap[ch] = -1;
      callbacksInt[ch] = NULL;
      callbackArgs[ch] = NULL;
      callbackDeferred[ch] = false;
      break;
    }
  }

  _criticalExit(criticalFromISR, criticalMask);
}

void GPIOTE_IRQHandler()
{
#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  // Read this once (not 8x), as it's a volatile read
  // across the AHB, which adds up to 3 cycles.
  uint32_t const enabledInterruptMask = NRF_GPIOTE->INTENSET;
  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    // only process where the interrupt is enabled and the event register is set
    // check interrupt enabled mask first, as already read that IOM value, to
    // reduce delays from AHB (16MHz) reads.
    if ( 0 == (enabledInterruptMask & (1 << ch))) continue;
    if ( 0 == NRF_GPIOTE->EVENTS_IN[ch]) continue;

    // If the event was set and interrupts are enabled,
    // call the callback function only if it exists,
    // but ALWAYS clear the event to prevent an interrupt storm.
    if (channelMap[ch] != -1 && callbacksInt[ch]) {
      if ( callbackDeferred[ch] ) {
        // Adafruit defer callback to non-isr if configured so
        ada_callback(NULL, 0, callbacksInt[ch], (uint32_t) callbackArgs[ch]);
      } else {
        callbacksInt[ch](callbackArgs[ch]);
      }
    }

    // clear the event
    NRF_GPIOTE->EVENTS_IN[ch] = 0;
  }
#if __CORTEX_M == 0x04
  // See note at nRF52840_PS_v1.1.pdf section 6.1.8 ("interrupt clearing")
  // See also https://gcc.gnu.org/onlinedocs/gcc/Volatiles.html for why
  // using memory barrier instead of read of an unrelated volatile
  __DSB(); __NOP();__NOP();__NOP();__NOP();
#endif

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif
}
