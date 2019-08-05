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

#ifndef _VARIANT_PARTICLE_XENON_
#define _VARIANT_PARTICLE_XENON_

#define _PINNUM(port, pin)    ((port)*32 + (pin))

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
#define PINS_COUNT           (34)
#define NUM_DIGITAL_PINS     (20)
#define NUM_ANALOG_INPUTS    (6) // A6 is used for battery, A7 is analog reference
#define NUM_ANALOG_OUTPUTS   (6)

/*
 * Digital pins
 */
#define PIN_D0               _PINNUM(0, 26)
#define PIN_D1               _PINNUM(0, 27)
#define PIN_D2               _PINNUM(1, 1)
#define PIN_D3               _PINNUM(1, 2)
#define PIN_D4               _PINNUM(1, 8)
#define PIN_D5               _PINNUM(1, 10)
#define PIN_D6               _PINNUM(1, 11)
#define PIN_D7               _PINNUM(1, 12)
#define PIN_D8               _PINNUM(1, 3)
#define PIN_D9               _PINNUM(0, 6)
#define PIN_D10              _PINNUM(0, 8)
#define PIN_D11              _PINNUM(1, 14)
#define PIN_D12              _PINNUM(1, 13)
#define PIN_D13              _PINNUM(1, 15)
#define PIN_D14              _PINNUM(0, 31)
#define PIN_D15              _PINNUM(0, 30)
#define PIN_D16              _PINNUM(0, 29)
#define PIN_D17              _PINNUM(0, 28)
#define PIN_D18              _PINNUM(0, 24)
#define PIN_D19              _PINNUM(0, 3)
#define PIN_D20              _PINNUM(0, 11)

static const uint8_t D0  = PIN_D0;
static const uint8_t D1  = PIN_D1;
static const uint8_t D2  = PIN_D2;
static const uint8_t D3  = PIN_D4;
static const uint8_t D5  = PIN_D5;
static const uint8_t D6  = PIN_D0;
static const uint8_t D7  = PIN_D1;
static const uint8_t D8  = PIN_D2;
static const uint8_t D9  = PIN_D4;
static const uint8_t D10 = PIN_D5;
static const uint8_t D11 = PIN_D0;
static const uint8_t D12 = PIN_D1;
static const uint8_t D13 = PIN_D2;
static const uint8_t D14 = PIN_D4;
static const uint8_t D15 = PIN_D5;
static const uint8_t D16 = PIN_D0;
static const uint8_t D17 = PIN_D1;
static const uint8_t D18 = PIN_D2;
static const uint8_t D19 = PIN_D4;
static const uint8_t D20 = PIN_D5;

// LEDs
 #define LEDS_NUMBER    1
 #define LED_PRIMARY_PIN            _PINNUM(1, 12)
 #define LED_STATE_ON   1
 #define LED_PRIMARY_IDX 0
 #define LED_RGB_RED_IDX 1
 #define LED_RGB_GREEN_IDX 2
 #define LED_RGB_BLUE_IDX 3

 #define LED_RGB_RED_PIN            _PINNUM(0, 13)
 #define LED_RGB_GREEN_PIN          _PINNUM(0, 14)
 #define LED_RGB_BLUE_PIN           _PINNUM(0, 15)
 #define BOARD_RGB_BRIGHTNESS 0x202020

 static const uint8_t LED_BUILTIN   = LED_PRIMARY_PIN;
 static const uint8_t LED_RGB_RED   = LED_RGB_RED_PIN;
 static const uint8_t LED_RGB_GREEN = LED_RGB_GREEN_PIN;
 static const uint8_t LED_RGB_BLUE  = LED_RGB_BLUE_PIN;

// Buttons
#define BUTTONS_NUMBER 2
#define BUTTON_DFU                  _PINNUM(0, 11)
#define BUTTON_FRESET               _PINNUM(0, 03) // A0
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

static const uint8_t BUTTON_MODE = PIN_D20;

// Antenna
#define ANTENNA_SWITCH_1            _PINNUM(0, 24)
#define ANTENNA_SWITCH_2            _PINNUM(0, 25)

// NFC
#define NFC1                        _PINNUM(0, 9)
#define NFC2                        _PINNUM(0, 10)

/*
 * Analog pins
 */
#define PIN_A0                      _PINNUM(0, 3)
#define PIN_A1                      _PINNUM(0, 4)
#define PIN_A2                      _PINNUM(0, 28)
#define PIN_A3                      _PINNUM(0, 29)
#define PIN_A4                      _PINNUM(0, 30)
#define PIN_A5                      _PINNUM(0, 31)

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
#define ADC_RESOLUTION    14

/*
 * Serial interfaces
 */
// Serial
#define PIN_SERIAL_RX         PIN_D9
#define PIN_SERIAL_TX         PIN_D10
#define PIN_SERIAL2_RX        PIN_D5
#define PIN_SERIAL2_TX        PIN_D6
//#define HAVE_HWSERIAL2  true  // TODO: implement UARTE in Uart.cpp

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_SS           PIN_A0
#define PIN_SPI_MISO         _PINNUM(1, 14)
#define PIN_SPI_MOSI         _PINNUM(1, 13)
#define PIN_SPI_SCK          _PINNUM(1, 15)
#define PIN_SPI1_MISO        _PINNUM(1, 8)
#define PIN_SPI1_MOSI        _PINNUM(1, 2)
#define PIN_SPI1_SCK         _PINNUM(1, 1)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         _PINNUM(0, 26)
#define PIN_WIRE_SCL         _PINNUM(0, 27)
#define PIN_WIRE1_SDA        _PINNUM(1, 1)
#define PIN_WIRE1_SCL        _PINNUM(1, 2)

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C

#define USB_MSC_BLOCK_SIZE    512
#define USB_MSC_BLOCK_COUNT   ((2*1024*1024) / USB_MSC_BLOCK_SIZE)

void switch_antenna(bool useExternal);

// led pwm
void led_pwm_init(uint32_t led_index, uint32_t led_pin);
void led_pwm_teardown(void);
void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle);
void pwm_teardown(NRF_PWM_Type* pwm);
static uint16_t led_duty_cycles[PWM0_CH_NUM] = { 0 };

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
