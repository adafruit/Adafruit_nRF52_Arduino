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

const uint32_t g_ADigitalPinMap[] =
{
  // D0 .. D13
  29,  // D0  is P0.29 (GPIO D0 / A6 / UART RX)
  14,  // D1  is P0.14 (GPIO D1 / UART TX)
   4,  // D2  is P0.04 (GPIO D2 / A5 / SDA)
   5,  // D3  is P0.05 (GPIO D3 / A4 / SCL)
  33,  // D4  is P1.01 (GPIO D4 / Left Button)
  47,  // D5  is P1.15 (GPIO D5 / Right button)
   3,  // D6  is P0.03 (GPIO D6 / A1)
  34,  // D7  is P1.02 (GPIO D7 / Slide Switch)
  13,  // D8  is P0.13 (GPIO D8 / NeoPixels)
   2,  // D9  is P0.02 (GPIO D9 / A2)
  28,  // D10 is P0.28 (GPIO D10 / A3)
  35,  // D11 is P1.03 (GPIO D11 / Speaker Shutdown)
  26,  // D12 is P0.26 (GPIO D12 / Audio Out)
  46,  // D13 is P1.14 (GPIO D13 / Red LED)

  // D14 .. D23
  26,  // NOT REALLY ANALOG, A PLACEHODER SINCE D12 is not analog
   3,  // D6  is P0.03 (GPIO D6 / A1)
   2,  // D9  is P0.02 (GPIO D9 / A2)
  28,  // D10 is P0.28 (GPIO D10 / A3)
   5,  // D3  is P0.05 (GPIO D3 / A4 / SCL)
   4,  // D2  is P0.04 (GPIO D2 / A5 / SDA)
  29,  // D0  is P0.29 (GPIO D0 / A6 / UART RX)
  14,  // NOT REALLY ANALOG, A PLACEHODER SINCE D1 is not analog
  31,  // A8 - Light sensor
  30,  // A9 - Thermistor sensor

  45,  // D24 is P1.13  (LIS IRQ)
  44,  // D25 is P1.12 (LIS SCL)
  43,  // D26 is P1.11 (LIS SDA)

  // QSPI pins (not exposed via any header / test point)
  19,  // D27 is P0.19 (QSPI CLK)
  15,  // D28 is P0.15 (QSPI CS)
  21,  // D29 is P0.21 (QSPI Data 0)
  23,  // D30 is P0.23 (QSPI Data 1)
  32,  // D31 is P1.00 (QSPI Data 2)
  22,  // D32 is P0.22 (QSPI Data 3)

  // D33 & D34 PDM pins
  24,  // D33 is P0.24 (PDM CLK)
  25,  // D34 is P0.25 (PDM DAT)

};

void initVariant()
{
  // LED1
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);
}

