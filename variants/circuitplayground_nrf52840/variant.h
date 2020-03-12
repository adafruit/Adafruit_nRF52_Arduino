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

#ifndef _VARIANT_CIRCUITPLAY52840_
#define _VARIANT_CIRCUITPLAY52840_

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
#define PINS_COUNT           (36)
#define NUM_DIGITAL_PINS     (36)
#define NUM_ANALOG_INPUTS    (8)
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
#define PIN_LED1             (13)
#define PIN_NEOPIXEL         (8)

#define LED_BUILTIN          PIN_LED1

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED1

#define LED_STATE_ON         1         // State when LED is litted

// Buttons
#define PIN_BUTTON1          (4)    // Button A (Left)
#define PIN_BUTTON2          (5)    // Button B (Right)

// Microphone
#define PIN_PDM_DIN           24
#define PIN_PDM_CLK           25
#define PIN_PDM_PWR           -1  // not used

// Buzzer
#define PIN_BUZZER            12

/*
 * Analog pins
 */
#define PIN_A0               (14)  // not really analog!
#define PIN_A1               (15)
#define PIN_A2               (16)
#define PIN_A3               (17)
#define PIN_A4               (18)
#define PIN_A5               (19)
#define PIN_A6               (20)
#define PIN_A7               (21) // not really analog!
#define PIN_A8               (22)
#define PIN_A9               (23)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;
static const uint8_t A8  = PIN_A8 ;
static const uint8_t A9  = PIN_A9 ;
#define ADC_RESOLUTION    14

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX       (0)
#define PIN_SERIAL1_TX       (1)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (10)
#define PIN_SPI_MOSI         (2)
#define PIN_SPI_SCK          (3)

static const uint8_t SS   = (0);
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA        (2)
#define PIN_WIRE_SCL        (3)

#define PIN_WIRE1_SDA       (28)
#define PIN_WIRE1_SCL       (26)

/*
 * QSPI Interfaces
 */
#define PIN_QSPI_SCK         29
#define PIN_QSPI_CS          30
#define PIN_QSPI_IO0         31
#define PIN_QSPI_IO1         32
#define PIN_QSPI_IO2         33
#define PIN_QSPI_IO3         34

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

/*
 * PDM Interfaces
 */
#define PIN_PDM_DIN         24
#define PIN_PDM_CLK         25
#define PIN_PDM_PWR         -1 // not used

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
