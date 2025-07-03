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

#ifndef _VARIANT_OMNIMO_NRF52840_
#define _VARIANT_OMNIMO_NRF52840_

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
#define PINS_COUNT           (44)
#define NUM_DIGITAL_PINS     (44)
#define NUM_ANALOG_INPUTS    (8) // A6 is used for battery voltage monitoring
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs /////////////////////
#define PIN_LED1             (3)
#define PIN_LED2             (4)
#define PIN_NEOPIXEL         (8)
#define NEOPIXEL_NUM         1

static const uint8_t LED1  = PIN_LED1 ;
static const uint8_t LED2  = PIN_LED2 ;

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2

#define LED_RED              PIN_LED1
#define LED_BLUE             PIN_LED2

#define LED_STATE_ON         1         // State when LED is litted

// Buttons //////////////////
#define PIN_BUTTON1          (7)
#define PIN_BTN1             (7)
#define PIN_BUTTON2          (37)
#define PIN_BTN2             (37)

static const uint8_t BTN1  = PIN_BTN1 ;
static const uint8_t BTN2  = PIN_BTN2 ;

// Analog Pins //////////////
#define PIN_A0               (14)
#define PIN_A1               (15)
#define PIN_A2               (16)
#define PIN_A3               (17)
#define PIN_A4               (18)
#define PIN_A5               (19)
#define PIN_A6               (20)
#define PIN_A7               (21)  // mikroBUS AN

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;

#define PIN_VBAT           PIN_A6

#define ADC_RESOLUTION    14

// NFC Pins /////////////////
#define PIN_NFC1           (33)
#define PIN_NFC2           (34)

static const uint8_t NFC1 = PIN_NFC1 ;
static const uint8_t NFC2 = PIN_NFC2 ;

// Feather UART Pins ////////
#define PIN_SERIAL1_RX       (1)
#define PIN_SERIAL1_TX       (0)
// Redefine
#define PIN_FEATHER_RX       PIN_SERIAL1_RX
#define PIN_FEATHER_TX       PIN_SERIAL1_TX

// Feather SPI Interface ////
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (24)
#define PIN_SPI_MOSI         (25)
#define PIN_SPI_SCK          (26)
// Redefine
#define PIN_FEATHER_MISO     PIN_SPI_MISO
#define PIN_FEATHER_MOSI     PIN_SPI_MOSI
#define PIN_FEATHER_SCK      PIN_SPI_SCK

static const uint8_t SS   = (5);
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

// Feather I2C Interface ////
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (22)
#define PIN_WIRE_SCL         (23)
// Redefine
#define PIN_FEATHER_SDA      PIN_WIRE_SDA
#define PIN_FEATHER_SCL      PIN_WIRE_SCL

// mikroBUS Pins ////////////
#define PIN_mikroBUS_PWM      (10)
#define PIN_mikroBUS_INT      (9)
#define PIN_mikroBUS_RX       (6)
#define PIN_mikroBUS_TX       (5)
#define PIN_mikroBUS_SCL      (23)
#define PIN_mikroBUS_SDA      (22)

#define PIN_mikroBUS_AN       (21)
#define PIN_mikroBUS_RST      (25)
#define PIN_mikroBUS_CS       (24)
#define PIN_mikroBUS_SCK      (1)
#define PIN_mikroBUS_MISO     (0)
#define PIN_mikroBUS_MOSI     (2)

static const uint8_t mikroBUS_PWM  = PIN_mikroBUS_PWM ;
static const uint8_t mikroBUS_INT  = PIN_mikroBUS_INT ;
static const uint8_t mikroBUS_RX   = PIN_mikroBUS_RX ;
static const uint8_t mikroBUS_TX   = PIN_mikroBUS_TX ;
static const uint8_t mikroBUS_SCL  = PIN_mikroBUS_SCL ;
static const uint8_t mikroBUS_SDA  = PIN_mikroBUS_SDA ;

static const uint8_t mikroBUS_AN   = PIN_mikroBUS_AN ;
static const uint8_t mikroBUS_RST  = PIN_mikroBUS_RST ;
static const uint8_t mikroBUS_CS   = PIN_mikroBUS_CS ;
static const uint8_t mikroBUS_SCK  = PIN_mikroBUS_SCK ;
static const uint8_t mikroBUS_MISO = PIN_mikroBUS_MISO ;
static const uint8_t mikroBUS_MOSI = PIN_mikroBUS_MOSI ;

// PMOD Pins ////////////////
#define PIN_PMOD1             (17)
#define PIN_PMOD2             (16)
#define PIN_PMOD3             (15)
#define PIN_PMOD4             (14)
#define PIN_PMOD5             (39)
#define PIN_PMOD6             (40)
#define PIN_PMOD7             (41)
#define PIN_PMOD8             (42)

static const uint8_t PMOD1  = PIN_PMOD1 ;
static const uint8_t PMOD2  = PIN_PMOD2 ;
static const uint8_t PMOD3  = PIN_PMOD3 ;
static const uint8_t PMOD4  = PIN_PMOD4 ;
static const uint8_t PMOD5  = PIN_PMOD5 ;
static const uint8_t PMOD6  = PIN_PMOD6 ;
static const uint8_t PMOD7  = PIN_PMOD7 ;
static const uint8_t PMOD8  = PIN_PMOD8 ;

// Qwiic Pins ///////////////
#define PIN_QWIIC_SCL         (35)
#define PIN_QWIIC_SDA         (36)

static const uint8_t QWIIC_SCL = PIN_QWIIC_SCL ;
static const uint8_t QWIIC_SDA = PIN_QWIIC_SDA ;

// QSPI Pins ////////////////
#define PIN_QSPI_SCK         27
#define PIN_QSPI_CS          28
#define PIN_QSPI_IO0         29
#define PIN_QSPI_IO1         30
#define PIN_QSPI_IO2         31
#define PIN_QSPI_IO3         32

// On-board QSPI Flash //////
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
