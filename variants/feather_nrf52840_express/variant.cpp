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

#include "usb.h"
#include "flash/flash_qspi.h"

const uint32_t g_ADigitalPinMap[] =
{
  25,  // D0 is UART TX on P0.25
  24,  // D1 is UART RX on P0.24 
  10,  // D2 is on P0.10
  47,  // D3 is LED1 on P1.15
  42,  // D4 is LED2 on P1.10
  40,  // D5 on P1.08
  7,   // D6 on P0.07 
  34,  // D7 is Button on P1.02
  16,  // D8 is NeoPixel on P0.16
  26,  // D9 on P0.26
  27,  // D10 on P0.27 
  6,   // D11 on P.06 
  8,   // D12 on P0.08
  41,  // D13 on P1.09 

  4,   // D14 is A0 on P0.04
  5,   // D15 is A1 on P0.05
  30,  // D16 is A2 on P0.30
  28,  // D17 is A3 on P0.28
  2,   // D18 is A4 on P0.02
  3,   // D19 is A5 on P0.03
  29,  // D20 is A6 (Battery) on P0.29
  31,  // D21 is A7 (ARef) on P0.31

  12,  // D22 is SDA on P0.12
  11,  // D23 is SCL on P0.11

  15,  // D24 is SPI MISO on P0.15 
  13,  // D25 is SPI MOSI on P0.13 
  14,  // D26 is SPI SCK on P0.14 

  19,  // D27 is QSPI CLK on P0.19
  20,  // D28 is QSPI CS on P0.20
  17,  // D29 is QSPI Data 0 on P0.17
  22,  // D30 is QSPI Data 1 on P0.22
  23,  // D31 is QSPI Data 2 on P0.23
  21,  // D32 is QSPI Data 3 on P0.21

  // P1
  33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47
};

void initVariant()
{
  // LED1 & LED2
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);

#if 0 // disable QSPI flash for now
  // Init External Flash
  flash_qspi_init();
#endif


  // USB init
  usb_init();
}

