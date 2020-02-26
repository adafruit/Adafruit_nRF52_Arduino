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
  // D0 .. D20
   4,  // D0  is P0.04 (GPIO D0 / AIN2 / UART RX)
   5,  // D1  is P0.05 (GPIO D1 / AIN3 / UART TX)
   3,  // D2  is P0.03 (GPIO D2 / AIN4)
  28,  // D3  is P0.28 (GPIO D3 / AIN5)
   2,  // D4  is P0.02 (GPIO D4 / AIN6)
  34,  // D5  is P1.02 (GPIO D5 / Left button)
  41,  // D6  is P1.09 (GPIO D6)
   7,  // D7  is P0.07 (GPIO D7)
  39,  // D8  is P1.07 (GPIO D8)
  27,  // D9  is P0.27 (GPIO D9)
  30,  // D10 is P0.30 (GPIO D10 / AIN7)
  42,  // D11 is P1.10 (GPIO D11 / Right Button)
  31,  // D12 is P0.31 (GPIO D12 / AIN0)
   8,  // D13 is P0.08 (GPIO D13 / SCK)
   6,  // D14 is P0.06 (GPIO D14 / MISO)
  26,  // D15 is P0.26 (GPIO D15 / MOSI)
  29,  // D16 is P0.29 (GPIO D16 / AIN1)
  33,  // D17 is P1.01 (Red LED)
  16,  // D18 is P0.16 (NeoPixel)
  25,  // D19 is P0.25 (GPIO D19 / SCL)
  24,  // D20 is P0.24 (GPIO D20 / SDA)

  // D21..D28 (A0 to A7) 
  31,  // D12 is P0.31 (GPIO D12 / AIN0)
  29,  // D16 is P0.29 (GPIO D16 / AIN1)
   4,  // D0  is P0.04 (GPIO D0 / AIN2 / UART RX)
   5,  // D1  is P0.05 (GPIO D1 / AIN3 / UART TX)
   3,  // D2  is P0.03 (GPIO D2 / AIN4)
  28,  // D3  is P0.28 (GPIO D3 / AIN5)
   2,  // D4  is P0.02 (GPIO D4 / AIN6)
  30,  // D10 is P0.30 (GPIO D10 / AIN7)

  // D29 .. D34 - TFT control
  14, // P0.14 (TFT SCK)
  15, // P0.15 (TFT MOSI)
  12, // P0.12 (TFT CS)
  13, // P0.13 (TFT DC)
  35, // P1.03 (TFT RESET)
  37, // P1.05 (TFT LITE)

  // 35 & 36 - PDM mic
  0,   // D35 is P0.00 (PDM DAT)
  1,   // D36 is P0.01 (PDM CLK)

  // QSPI pins (not exposed via any header / test point)
  19,  // D37 is P0.19 (QSPI CLK)
  20,  // D38 is P0.20 (QSPI CS)
  17,  // D39 is P0.17 (QSPI Data 0)
  22,  // D40 is P0.22 (QSPI Data 1)
  23,  // D41 is P0.23 (QSPI Data 2)
  21,  // D42 is P0.21 (QSPI Data 3)

  10,  // D43 is P0.10 white LED control
   9,  // D44 is P0.09 APDS IRQ
  38,  // D45 is P1.06 LSM6DS33 IRQ
  32,  // D46 is P1.00 Speaker/buzzer
};

void initVariant()
{
  // LED1
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  // Disable TFT LITE powering up
  pinMode(PIN_TFT_LITE, OUTPUT);
  digitalWrite(PIN_TFT_LITE, LOW);
}

