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
#define PINS_COUNT           (64)
#define NUM_DIGITAL_PINS     (64)
#define NUM_ANALOG_INPUTS    (6)
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
#define PIN_LED1             (6)
#define PIN_LED2_R           (8)
#define PIN_LED2_G           (41)     //P1.09
#define PIN_LED2_B           (12)     //P0.12

#define LED_BUILTIN          PIN_LED1
#define LED_CONN             PIN_LED2_B

#define LED_RED              PIN_LED2_R
#define LED_BLUE             PIN_LED2_B
#define LED_GREEN            PIN_LED2_G

#define LED_STATE_ON         0         // State when LED is litted

/*
 * Analog pins
 */
#define PIN_A0               (2)       //P0.02 
#define PIN_A1               (0xff)    //P0.03 not connected
#define PIN_A2               (4)       //P0.04
#define PIN_A3               (0xff)    //P0.05 not connected
#define PIN_A4               (0xff)    //P0.28 not connected
#define PIN_A5               (29)      //P0.29
#define PIN_A6               (0xff)    //P0.30 not connected
#define PIN_A7               (31)      //P0.31

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
#define PIN_AREF           (2)       //P0.02           
#define PIN_DFU            (43)      //P1.11
#define PIN_NFC1           (9)       //P0.09 
#define PIN_NFC2           (10)      //P0.10 

static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
#define PIN_SERIAL_RX       (11)     //P0.11 located on bottom
#define PIN_SERIAL_TX       (14)     //P0.14 located on bottom

//#define PIN_SERIAL2_RX      (8)
//#define PIN_SERIAL2_TX      (6)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (46)            //P1.10
#define PIN_SPI_MOSI         (45)            //P1.13
#define PIN_SPI_SCK          (47)            //P1.15

static const uint8_t SS   = 32;              //P1.00  
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (13)             //P0.13
#define PIN_WIRE_SCL         (15)             //P0.15

/*
 * QSPI interface for external flash
 */
#define PIN_QSPI_SCK         16               //not connected
#define PIN_QSPI_CS          17               //not connected
#define PIN_QSPI_DATA0       27               //not connected
#define PIN_QSPI_DATA1       7                //not connected
#define PIN_QSPI_DATA2       34               //not connected
#define PIN_QSPI_DATA3       27               //not connected

// On-board QSPI Flash
// If EXTERNAL_FLASH_DEVICES is not defined, all supported devices will be used
#define EXTERNAL_FLASH_DEVICES   GD25Q16C

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
