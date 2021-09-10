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
  // UART: place holder only
  25   , // D0  is UART TX (not connected)
  24   , // D1  is UART RX (not connected)

  31   , // D2  is LED1
  32+15, // D3  is NeoPixel
  30   , // D4  is Button
  27   , // D5  is MIC Data
  0    , // D6  is MIC Clk
  1    , // D7  is LIS3DH IRQ

  // I2C: connected to on-board LIS3DH and SteammaQT
  8    , // D8  is SCL
  6    , // D9  is SDA

  // SPI: place holder only
  13   , // D10 is MISO (not connected)
  14   , // D11 is MOSI (not connected)
  15   , // D12 is SCK  (not connected)
  16   , // D13 is SS   (not connected)

  // A0 .. A7: only A6 is connected to Battery
  32+3 , // D14 is A0 (not connected)
  32+4 , // D15 is A1 (not connected)
  32+5 , // D16 is A2 (not connected)
  32+6 , // D17 is A3 (not connected)
  32+7 , // D18 is A4 (not connected)
  32+1 , // D19 is A5 (not connected)
  4    , // D20 is A6 Battery
  5    , // D21 is A7 (not connected)

  // QSPI pins (not exposed via any header / test point)
  19   , // D22 is QSPI CLK
  20   , // D23 is QSPI CS
  32+0 , // D24 is QSPI Data 0
  21   , // D25 is QSPI Data 1
  22   , // D26 is QSPI Data 2
  23   , // D27 is QSPI Data 3

  // Thus, there are 28 defined pins
};

void initVariant()
{
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);
}
