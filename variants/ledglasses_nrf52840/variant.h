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

#ifndef _VARIANT_FEATHER52840_
#define _VARIANT_FEATHER52840_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

//#define USE_LFXO      // Board uses 32khz crystal for LF
#define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (28)
#define NUM_DIGITAL_PINS     (28)
#define NUM_ANALOG_INPUTS    (6) // only A6 is routed and used for battery
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
#define PIN_LED1             (2)
#define PIN_NEOPIXEL         (3)
#define NEOPIXEL_NUM         1

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED1

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED1

#define LED_STATE_ON         1         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          (4)

// Microphone
#define PIN_PDM_DIN           5
#define PIN_PDM_CLK           6
#define PIN_PDM_PWR           -1  // not used

/*
 * Analog pins
 */
#define PIN_A0               (14) // not connected
#define PIN_A1               (15) // not connected
#define PIN_A2               (16) // not connected
#define PIN_A3               (17) // not connected
#define PIN_A4               (18) // not connected
#define PIN_A5               (19) // not connected
#define PIN_A6               (20) // VBAT
#define PIN_A7               (21) // not connected

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
#define PIN_VBAT           PIN_A6

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX       (1) // not connected
#define PIN_SERIAL1_TX       (0) // not connected

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (10) // not connected
#define PIN_SPI_MOSI         (11) // not connected
#define PIN_SPI_SCK          (12) // not connected

static const uint8_t SS   = (13); // not connected
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SCL         (8)
#define PIN_WIRE_SDA         (9)

// QSPI Pins
#define PIN_QSPI_SCK         22
#define PIN_QSPI_CS          23
#define PIN_QSPI_IO0         24
#define PIN_QSPI_IO1         25
#define PIN_QSPI_IO2         26
#define PIN_QSPI_IO3         27

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
