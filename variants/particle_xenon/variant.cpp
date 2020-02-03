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
  26,   // P0.26 (SDA)
  27,   // P0.27 (SCL)
  33,   // P1.01 (SDA1) (UART1_RTS) (SPI1_SCK) (PWM3)
  34,   // P1.02 (SCL1) (UART1_CTS) (SPI1_MOSI) (PWM3)
  40,   // P1.08 (UART2_TX) (SPI1_MISO) (PWM1)
  42,   // P1.10 (UART2_RX) (PWM1)
  43,   // P1.11 (UART2_CTS) (PWM1)
  44,   // P1.12 (PWM0)
  35,   // P1.03 (UART2_RTS)

  // D9 .. D20
  6,    // P0.06 (UART1_TX)
  8,    // P0.27 (UART1_RX)
  46,   // P1.14 (SPI_MISO)
  45,   // P1.13 (SPI_MOSI)
  47,   // P1.15 (SPI_SCK)
  31,   // P0.31 (PWM3) (SPI_SS) (ADC5)
  30,   // P0.30 (PWM3) (ADC4)
  29,   // P0.29 (PWM2) (ADC3)
  28,   // P0.28 (PWM2) (ADC2)
  4,    // P0.04 (PWM2) (ADC1)
  3,    // P0.03 (PWM2) (ADC0)
  11,   // P0.11 (MODE)
  18,   // P0.18 (RESET)
};

static void switch_antenna(bool useExternal) {
  if (useExternal) {
    digitalWrite(ANTENNA_SWITCH_1, LOW);
    digitalWrite(ANTENNA_SWITCH_2, HIGH);
  }
  else {
    digitalWrite(ANTENNA_SWITCH_1, HIGH);
    digitalWrite(ANTENNA_SWITCH_2, LOW);
  }
}

void initVariant()
{
  switch_antenna(false);
}
