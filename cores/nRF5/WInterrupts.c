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

  // To avoid unexpected consequences, first loop through the channelMap
  // to preferably re-use any channel that was already in use (even if
  // earlier channel is no longer in use).
  for (int ch = 0; ch < NUMBER_OF_GPIO_TE; ch++) {
    // skip if the channel was not already assigned to this pin
    if ((uint32_t)channelMap[ch] != pin) continue;

    // The pin is already allocated in the channelMap
    // update the polarity (when events fire) and callbacks
    // However, do NOT clear any GPIOTE events
    uint32_t tmp = NRF_GPIOTE->CONFIG[ch];
    channelMap[ch] = pin;
    callbacksInt[ch] = callback;
    callbackDeferred[ch] = deferred;
    tmp &= oldRegMask;
    tmp |= newRegBits;
    NRF_GPIOTE->CONFIG[ch] = tmp;
    asm volatile ("" : : : "memory");
    __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n");
    NRF_GPIOTE->INTENSET = (1 << ch); // old code did this ... no harm in ensuring this is set
    return (1 << ch);
  }

  // When the pin isn't already configured for interrupts, then attempt to
  // find an unused GPIOTE channel.  This code is identical to the above,
  // except for also validating that the MODE bits show the channel is disabled.
  for (int ch=0; ch < NUMBER_OF_GPIO_TE; ch++) {
    // skip if already used in AttachInterrupt for another pin
    if (channelMap[ch] != -1) continue;
    // skip if channel is not disabled (e.g., in use by some other component or library)
    if (nrf_gpiote_te_is_enabled(NRF_GPIOTE, ch)) continue;

    // clear any old events on this GPIOTE channel
    NRF_GPIOTE->EVENTS_IN[ch] = 0;
    uint32_t tmp = NRF_GPIOTE->CONFIG[ch];
    channelMap[ch] = pin;
    callbacksInt[ch] = callback;
    callbackDeferred[ch] = deferred;
    tmp &= oldRegMask;
    tmp |= newRegBits;
    // TODO: make check/set for new channel an atomic operation
    NRF_GPIOTE->CONFIG[ch] = tmp;
    asm volatile ("" : : : "memory");
    __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n");
    NRF_GPIOTE->INTENSET = (1 << ch);
    return (1 << ch);
  }
  // Else, pin was neither already setup, nor could a GPIOTE be allocated
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
      // Unfortunately, simply marking a variable as volatile is not enough to
      // prevent GCC from reordering or combining memory accesses, without also
      // having an explicit sequence point.
      //   See https://gcc.gnu.org/onlinedocs/gcc/Volatiles.html
      //
      // Here, ensure a sequence point by adding a memory barrier
      // followed with four nops after having disabled the interrupt,
      // to ensure the peripheral registers have been updated.
      NRF_GPIOTE->INTENCLR = (1 << ch);
      asm volatile ("" : : : "memory");
      __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n");

      // now cleanup the rest of the use of the channel
      channelMap[ch] = -1;
      callbacksInt[ch] = NULL;
      callbackDeferred[ch] = false;
      NRF_GPIOTE->EVENTS_IN[ch] = 0; // clear the event
      // Finally, clear the CONFIG register only after ensure
      // all the other state has been written to the peripheral registers
      asm volatile ("" : : : "memory");
      __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n");
      NRF_GPIOTE->CONFIG[ch] = 0;
      break;
    }
  }
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
        ada_callback(NULL, 0, callbacksInt[ch]);
      } else {
        callbacksInt[ch]();
      }
    }

    // clear the event
    NRF_GPIOTE->EVENTS_IN[ch] = 0;
  }
#if __CORTEX_M == 0x04
  // To quote from nRF52840 product specification:
  // --------------------------------------------------------------------------
  //   Note: To avoid an interrupt reoccurring before a new event has come,
  //         the program should perform a read from one of the peripheral
  //         registers. For example, the event register that has been cleared,
  //         or the INTENCLR register that has been used to disable the
  //         interrupt. This will cause a one to three cycle delay and ensure
  //         the interrupt is cleared before exiting the interrupt handler.
  //   Care should be taken to ensure the compiler does not remove the read
  //   operation as an optimization. If the program can guarantee a four-cycle
  //   delay after event being cleared or interrupt disabled in any other way,
  //   then a read of a register is not required.
  // --------------------------------------------------------------------------
  //
  // Unfortunately, simply marking a variable as volatile is not enough to
  // prevent GCC from reordering or combining memory accesses, without also
  // having an explicit sequence point.
  //   See https://gcc.gnu.org/onlinedocs/gcc/Volatiles.html
  //
  // Here, ensure a sequence point by adding a memory barrier
  // followed with four nops before exiting the ISR
  asm volatile ("" : : : "memory");
  __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n");
#endif

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif
}
