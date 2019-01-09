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

#ifndef _DELAY_
#define _DELAY_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "nrfx.h"
#include "variant.h"

/**
 * \brief Returns the number of milliseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 50 days.
 *
 * \return Number of milliseconds since the program started (uint32_t)
 */
extern uint32_t millis( void ) ;

/**
 * \brief Returns the number of microseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards
 * (e.g. Duemilanove and Nano), this function has a resolution of four microseconds (i.e. the value returned is
 * always a multiple of four). On 8 MHz Arduino boards (e.g. the LilyPad), this function has a resolution
 * of eight microseconds.
 *
 * \note There are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 */
extern uint32_t micros( void ) ;

/**
 * \brief Pauses the program for the amount of time (in miliseconds) specified as parameter.
 * (There are 1000 milliseconds in a second.)
 *
 * \param dwMs the number of milliseconds to pause (uint32_t)
 */
extern void delay( uint32_t dwMs );

/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param dwUs the number of microseconds to pause (uint32_t)
 */
static __inline__ void delayMicroseconds( uint32_t ) __attribute__((always_inline, unused)) ;
static __inline__ void delayMicroseconds( uint32_t usec )
{
  nrfx_coredep_delay_us(usec);
}

/**
 * Enable DWT, required for delay_ns()
 */
void dwt_enable(void);
void dwt_disable(void);


/**
 * Delay nano seconds, delay_ns() make use of DWT of debug core.
 * dwt_enable() must be called previously.
 * Note: nrf52 run at 64MHz
 *    - 1000 ns ~ 64 cycles
 *    -  500 ns ~ 32 cycles
 *    -  250 ns ~ 16 cycles
 *    -  125 ns ~  8 cycles
 */
// Enable pragma when this bug is fixed  https://bugs.launchpad.net/gcc-arm-embedded/+bug/1534360
// #pragma GCC push_options
// #pragma GCC optimize ("Ofast")

#define DELAY_CYCLE_CORRECTION   8 // ~125 ns overhead for delay_ns()
#define delay_ns(ns) \
  do {\
    register uint32_t _endtime = DWT->CYCCNT;\
    _endtime += ((F_CPU/1000000)*ns)/1000 - DELAY_CYCLE_CORRECTION;\
    while (DWT->CYCCNT < _endtime) {}\
  } while(0)

// #pragma GCC pop_options


#ifdef __cplusplus
}
#endif

#endif /* _DELAY_ */
