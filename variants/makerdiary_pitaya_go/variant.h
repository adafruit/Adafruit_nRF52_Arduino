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

#ifndef _VARIANT_PITAYAGO
#define _VARIANT_PITAYAGO

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
#define PINS_COUNT           (41)
#define NUM_DIGITAL_PINS     (41)
#define NUM_ANALOG_INPUTS    (7)
#define NUM_ANALOG_OUTPUTS   (0)

/*
 * LEDs
 */
#define PIN_LED1             (28)
#define PIN_LED2             (29)
#define PIN_LED3             (30)

#define LED_RED              PIN_LED1
#define LED_GREEN            PIN_LED2
#define LED_BLUE             PIN_LED3

#define LED_STATE_ON         0         // State when LED is litted

/*
 * Buttons
 */
#define PIN_BUTTON1          (21)

/*
 * Digital Pins
 */
static const uint8_t P13   = 4;
static const uint8_t P14   = 5;
static const uint8_t P15   = 6;
static const uint8_t P16   = 7;
static const uint8_t P17   = 8;
static const uint8_t P20   = 9;
static const uint8_t P21   = 10;
static const uint8_t P22   = 11;
static const uint8_t P23   = 12;
static const uint8_t P24   = 13;
static const uint8_t P25   = 14;

/*
 * Analog pins
 */
#define PIN_A0               (0)
#define PIN_A1               (1)
#define PIN_A2               (2)
#define PIN_A3               (3)
#define PIN_A4               (17)
#define PIN_A5               (18)
#define PIN_A6               (19)
#define PIN_A7               (20)

static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
#define ADC_RESOLUTION    14

/*
 * Other Pins
 */
#define PIN_VBAT          PIN_A0
#define PIN_NFC1          (31)
#define PIN_NFC2          (32)

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX    (15)
#define PIN_SERIAL1_TX    (16)

static const uint8_t RXD  = PIN_SERIAL1_RX;
static const uint8_t TXD  = PIN_SERIAL1_TX;

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO       (34)
#define PIN_SPI_MOSI       (38)
#define PIN_SPI_SCK        (35)

/*
 * Onboard ATWINC1500B WiFi Module
 */
static const uint8_t WIFI_SS    = (36);
static const uint8_t WIFI_MISO  = PIN_SPI_MISO;
static const uint8_t WIFI_MOSI  = PIN_SPI_MOSI;
static const uint8_t WIFI_SCK   = PIN_SPI_SCK;
static const uint8_t WIFI_IRQ   = (33);
static const uint8_t WIFI_EN    = (39);
static const uint8_t WIFI_RST   = (37);
static const uint8_t WIFI_WAKE  = (40);


/*
 * Wire Interfaces
 */
// #define WIRE_INTERFACES_COUNT 1

// #define PIN_WIRE_SDA       ()
// #define PIN_WIRE_SCL       ()

// QSPI Pins
#define PIN_QSPI_SCK       25
#define PIN_QSPI_CS        24
#define PIN_QSPI_IO0       27
#define PIN_QSPI_IO1       22
#define PIN_QSPI_IO2       26
#define PIN_QSPI_IO3       23

// On-board QSPI Flash
// If EXTERNAL_FLASH_DEVICES is not defined, all supported devices will be used
#define EXTERNAL_FLASH_DEVICES   MX25R6435F
#define EXTERNAL_FLASH_USE_QSPI

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
