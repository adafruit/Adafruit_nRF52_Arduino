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
  30,  // D0  is P0.30 (GPIO D0 / A6 / UART RX)
  14,  // D1  is P0.14 (GPIO D1 / UART TX)
   5,  // D2  is P0.05 (GPIO D2 / A5 / SDA)
   4,  // D3  is P0.04 (GPIO D3 / A4 / SCL)
  34,  // D4  is P1.02 (GPIO D4 / Left Button)
  47,  // D5  is P1.15 (GPIO D5 / Right button)
   2,  // D6  is P0.02 (GPIO D6 / A1)
  38,  // D7  is P1.06 (GPIO D7 / Slide Switch)
  13,  // D8  is P0.13 (GPIO D8 / NeoPixels)
  29,  // D9  is P0.29 (GPIO D9 / A2)
   3,  // D10 is P0.03 (GPIO D10 / A3)
  36,  // D11 is P1.04 (GPIO D11 / Speaker Shutdown)
  26,  // D12 is P0.26 (GPIO D12 / Audio Out)
  46,  // D13 is P1.14 (GPIO D13 / Red LED)

  // D14 .. D23
  26,  // NOT REALLY ANALOG, A PLACEHODER SINCE D12 is not analog
   2,  // D6  is P0.02 (GPIO D6 / A1)
  29,  // D9  is P0.29 (GPIO D9 / A2)
   3,  // D10 is P0.03 (GPIO D10 / A3)
   4,  // D3  is P0.04 (GPIO D3 / A4 / SCL)
   5,  // D2  is P0.05 (GPIO D2 / A5 / SDA)
  30,  // D0  is P0.30 (GPIO D0 / A6 / UART RX)
  14,  // NOT REALLY ANALOG, A PLACEHODER SINCE D1 is not analog
  28,  // A8 - Light sensor
  31,  // A9 - Thermistor sensor

  // D24 & D25 PDM pins
  16,  // D24 is P0.16 (PDM DAT)
  17,  // D25 is P0.17 (PDM CLK)

  44,  // D26 is P1.12 (LIS SCL)
  45,  // D27 is P1.13 (LIS IRQ)
  42,  // D28 is P1.10 (LIS SDA)

  // QSPI pins (not exposed via any header / test point)
  19,  // D29 is P0.19 (QSPI CLK)
  15,  // D30 is P0.15 (QSPI CS)
  21,  // D31 is P0.21 (QSPI Data 0)
  23,  // D32 is P0.23 (QSPI Data 1)
  32,  // D33 is P1.00 (QSPI Data 2)
  22,  // D34 is P0.22 (QSPI Data 3)

  // D35 power switch
   6,  // D36  is P0.06 (NeoPixel / Sensor switch)
};

void initVariant()
{
  // LED1
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);
}

