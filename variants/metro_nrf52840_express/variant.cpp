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

#define _PINNUM(port, pin)     ((port)*32 + (pin))

const uint32_t g_ADigitalPinMap[] =
{
  _PINNUM(0, 25), // D0 is UART TX on P0.25
  _PINNUM(0, 24), // D1 is UART RX on P0.24
  _PINNUM(1, 10), // D2 is on P1.10
  _PINNUM(1, 4 ), // D3 on P1.04
  _PINNUM(1, 11), // D4 on P1.11
  _PINNUM(1, 12), // D5 on P1.12
  _PINNUM(1, 14), // D6 on P1.14
  _PINNUM(0, 26), // D7 on P0.26
  _PINNUM(0, 27), // D8 on P0.27
  _PINNUM(0, 12), // D9 on P0.12
  _PINNUM(0, 6 ), // D10 on P0.06
  _PINNUM(0, 8 ), // D11 on P0.08
  _PINNUM(1, 9 ), // D12 on P1.09
  _PINNUM(0, 14), // D13 on P0.14

  _PINNUM(0, 4 ), // D14 is A0 on P0.04
  _PINNUM(0, 5 ), // D15 is A1 on P0.05
  _PINNUM(0, 28), // D16 is A2 on P0.28
  _PINNUM(0, 30), // D17 is A3 on P0.30
  _PINNUM(0, 2 ), // D18 is A4 on P0.02
  _PINNUM(0, 3 ), // D19 is A5 on P0.03
  _PINNUM(0, 29), // D20 is A6 (Battery) on P0.29
  _PINNUM(0, 31), // D21 is A7 (ARef) on P0.31

  _PINNUM(0, 15), // D22 is SDA on P0.15
  _PINNUM(0, 16), // D23 is SCL on P0.16

  _PINNUM(0, 11), // D24 is SPI MISO on P0.11
  _PINNUM(1, 8 ), // D25 is SPI MOSI on P1.08
  _PINNUM(0, 7 ), // D26 is SPI SCK on P0.07

  _PINNUM(0, 19), // D27 is QSPI CLK on P0.19
  _PINNUM(0, 20), // D28 is QSPI CS on P0.20
  _PINNUM(0, 17), // D29 is QSPI Data 0 on P0.17
  _PINNUM(0, 23), // D30 is QSPI Data 1 on P0.23
  _PINNUM(0, 22), // D31 is QSPI Data 2 on P0.22
  _PINNUM(0, 21), // D32 is QSPI Data 3 on P0.21

  _PINNUM(1, 13), // D33 is LED1 on P1.13
  _PINNUM(1, 15), // D34 is LED2 on P1.15
  _PINNUM(0, 13), // D35 is NeoPixel on P0.13
  _PINNUM(1, 2 ), // D36 is Switch on P1.02
  _PINNUM(1, 0 ), // D37 is SWO/DFU on P1.00
  _PINNUM(0, 9 ), // D38 is NFC1 on P0.09
  _PINNUM(0, 10), // D39 is NFC2 on P0.10

  // Unused pin
  40, 41, 42, 43, 44, 45, 46, 47
};



void initVariant()
{
  // LED1 & LED2
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);
}

