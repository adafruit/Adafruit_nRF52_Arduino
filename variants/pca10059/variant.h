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

#ifndef _VARIANT_PCA10059_
#define _VARIANT_PCA10059_

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

// Number of pins defined in PinDescription array
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (6)
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
// LD1 - Green LED
#define PIN_LED1             (6)   // P0.06
// LD2 - RGB LED
#define PIN_LED2_R           (8)   // P0.08 Red
#define PIN_LED2_G           (41)  // P1.09 Green
#define PIN_LED2_B           (12)  // P0.12 Blue

#define PIN_LED2             PIN_LED2_R

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2

#define LED_RED              PIN_LED2_R
#define LED_GREEN            PIN_LED1
#define LED_BLUE             PIN_LED2_B

#define LED_STATE_ON         0         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          (38)  // P1.06 (SW1)

/*
 * Analog pins
 */
#define PIN_A0               (2)   // P0.02 AIN0
#define PIN_A1               (4)   // P0.04 AIN2
#define PIN_A2               (5)   // P0.05 AIN3
#define PIN_A3               (28)  // P0.28 AIN4
#define PIN_A4               (29)  // P0.29 AIN5
#define PIN_A5               (31)  // P0.31 AIN7
#define PIN_A6               (0xff)
#define PIN_A7               (0xff)

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
#define PIN_NFC1           (9)   // P0.09
#define PIN_NFC2           (10)  // P0.10

static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */

// Edge GPIOs
#define PIN_SERIAL1_RX      (13)  // P0.13
#define PIN_SERIAL1_TX      (15)  // P0.15

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (24)  // P0.24
#define PIN_SPI_MOSI         (32)  // P1.00
#define PIN_SPI_SCK          (22)  // P0.22

static const uint8_t SS   = 17 ;   // P0.17
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (26)  // P0.26
#define PIN_WIRE_SCL         (29)  // P0.29

// No QSPI Flash on PCA10059

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
