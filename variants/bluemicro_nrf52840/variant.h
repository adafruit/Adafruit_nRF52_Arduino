 /*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

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

#ifndef _VARIANT_BLUEMICRO_NRF52840_
#define _VARIANT_BLUEMICRO_NRF52840_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define _PINNUM(port, pin)     ((port)*32 + (pin))

// Number of pins defined in PinDescription array
#define PINS_COUNT           (48u)
#define NUM_DIGITAL_PINS     (48u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED1             _PINNUM(1, 4)
#define PIN_LED2             _PINNUM(1, 10)

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED2

#define LED_STATE_ON         1         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          _PINNUM(0, 18)
#define PIN_BUTTON2          _PINNUM(1, 1)

/*
 * Analog pins
 */
#define PIN_A0               (2)
#define PIN_A1               (3)
#define PIN_A2               (4)
#define PIN_A3               (5)
#define PIN_A4               (28)
#define PIN_A5               (29)
#define PIN_A6               (30)
#define PIN_A7               (31)

#define PIN_VBAT           PIN_A7

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
#define ADC_RESOLUTION    14

// Other pins
#define PIN_AREF           (2)
#define PIN_NFC1           _PINNUM(0, 9)
#define PIN_NFC2           _PINNUM(0, 10)

static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */

// Arduino Header D0, D1
#define PIN_SERIAL1_RX      _PINNUM(1, 13)
#define PIN_SERIAL1_TX      _PINNUM(0, 28)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO         _PINNUM(0, 9)
#define PIN_SPI_MOSI         _PINNUM(1, 11)
#define PIN_SPI_SCK          _PINNUM(0, 10)

static const uint8_t SS   = _PINNUM(0, 3) ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA         _PINNUM(0, 30)
#define PIN_WIRE_SCL         _PINNUM(0, 2)


#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
