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

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"
#include "nrf_pwm.h"

const uint32_t g_ADigitalPinMap[] =
{
  // D0 .. D8
  _PINNUM(0, 26),   // P0.26 (SDA)
  _PINNUM(0, 27),   // P0.27 (SCL)
  _PINNUM(1, 1),    // P1.01 (SDA1) (UART1_RTS) (SPI1_SCK) (PWM3)
  _PINNUM(1, 2),    // P1.02 (SCL1) (UART1_CTS) (SPI1_MOSI) (PWM3)
  _PINNUM(1, 8),    // P1.08 (UART2_TX) (SPI1_MISO) (PWM1)
  _PINNUM(1, 10),   // P1.10 (UART2_RX) (PWM1)
  _PINNUM(1, 11),   // P1.11 (UART2_CTS) (PWM1)
  _PINNUM(1, 12),   // P1.12 (PWM0)
  _PINNUM(1, 03),   // P1.03 (UART2_RTS)

  // D9 .. D21
  _PINNUM(0, 6),    // P0.06 (UART1_TX)
  _PINNUM(0, 8),    // P0.08 (UART1_RX)
  _PINNUM(1, 14),   // P1.14 (SPI_MISO)
  _PINNUM(1, 13),   // P1.13 (SPI_MOSI)
  _PINNUM(1, 15),   // P1.15 (SPI_SCK)
  _PINNUM(0, 31),   // P0.31 (PWM3) (SPI_SS) (ADC5)
  _PINNUM(0, 30),   // P0.30 (PWM3) (ADC4)
  _PINNUM(0, 29),   // P0.29 (PWM2) (ADC3)
  _PINNUM(0, 28),   // P0.28 (PWM2) (ADC2)
  _PINNUM(0, 4),    // P0.04 (PWM2) (ADC1)
  _PINNUM(0, 3),    // P0.03 (PWM2) (ADC0)
  _PINNUM(0, 11),   // P0.11 (MODE)
  _PINNUM(0, 18),   // P0.18 (RESET)

  // D22 .. D24
  // LEDS
  _PINNUM(0, 13),   // P0.13 (RGB_RED)
  _PINNUM(0, 14),   // P0.14 (RGB_GREEN)
  _PINNUM(0, 15),   // P0.15 (RGB_BLUE) 

  // D25 .. D26
  // Antenna
  _PINNUM(0, 24),   // P0.24 (ANTENNA_SWITCH_1 - PCB ANTENNA)
  _PINNUM(0, 25),   // P0.25 (ANTENNA_SWITCH_2 - EXTERNAL u.FL)

  // D27 .. D28
  // NFC
  _PINNUM(0, 9),    // P0.09 (u.FL FOR NFC ANTENNA)
  _PINNUM(0, 10),   // P0.10 (u.FL FOR NFC ANTENNA)

  // D29 .. 36
  // Analog Pins A0 .. A7
  _PINNUM(0, 3),    // P0.03 (A0)
  _PINNUM(0, 4),    // P0.04 (A1)
  _PINNUM(0, 28),   // P0.28 (A2)
  _PINNUM(0, 29),   // P0.29 (A3)
  _PINNUM(0, 30),   // P0.30 (A4)
  _PINNUM(0, 31),   // P0.31 (A5)
  _PINNUM(0, 5),   // P0.05 (BAT_DET/VBAT)
  _PINNUM(0, 2),   // P0.02 (AREF)

  // D37 .. D38
  // Power status
  _PINNUM(0, 12),  // P0.12 (PWR)
  _PINNUM(1, 9),   // P1.09 (CHG)
};

void initVariant()
{
  // LED1
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);
}