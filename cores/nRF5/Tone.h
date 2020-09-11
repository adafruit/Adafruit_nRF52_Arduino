/*
  Copyright (c) 2016 Arduino.  All right reserved.

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
/** \file Tone.h */

#ifndef _WIRING_TONE_
#define _WIRING_TONE_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "wiring_digital.h"
#include "nrf_pwm.h"

/**
 * \brief Generate a tone (PWM) output on the specified pin.
 * 
 * \param[in] pin        The Arduino pin for output
 * 
 * \param[in] frequency  The frequency, in hertz, of the requested tone.
 *
 * \param[in] duration   An optional duration, in milliseconds (default: 0 == infinite)
 *
 * Generates a square wave of the specified frequency (and 50% duty cycle)
 * on a pin. A duration can be specified, otherwise the wave continues until a call
 * to `noTone()`. The pin can be connected to a piezo buzzer or other speaker to
 * play tones.
 * 
 * The `tone()` API has the following contracts, which were defined by
 * the original Arduino code:
 * 
 * 1. at most one `tone()` can be generated at a time
 * 2. so long as the `pin` stays the same, `tone()` can be called repeatedly
 * 3. a call to `noTone()` is required prior to calling `tone()` with a different pin.
 * 
 * The third requirement is not enforced in the nRF52 BSP.  Instead, if a call
 * to `tone()` is made with a different pin, a call to `noTone()` occurs automatically,
 * simplifying use somewhat.
 * 
 * For the nRF52, the allowed range for parameter `frequency` is `[20..25000]`.
 * 
 * \see noTone
 */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration = 0);
/**
 * \brief Stops generation of a tone (PWM) output.
 * 
 * \param pin        For the nRF52, this argument is ignored
 * 
 * Stops the generation of a square wave triggered by `tone()`.
 * This function has no effect if no tone is being generated.
 */
void noTone(uint8_t pin);



#endif /* _WIRING_TONE_ */
