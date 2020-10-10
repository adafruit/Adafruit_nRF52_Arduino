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
  // D0-D10 on the Left Side
  _PINNUM(0, 13), // D0 is on P0.13
  _PINNUM(0, 14), // D1 is on P0.14
  _PINNUM(0, 15), // D2 is SDA on P0.15
  _PINNUM(0, 16), // D3 is SCL on P0.16
  _PINNUM(0, 17), // D4 is on P0.17
  _PINNUM(0, 20), // D5 is on P0.20
  _PINNUM(0, 21), // D6 is on P0.21
  _PINNUM(0, 22), // D7 is on P0.22
  _PINNUM(0, 23), // D8 is on P0.23
  _PINNUM(0, 24), // D9 is on P0.24
  _PINNUM(0, 25), // D10 is on P0.25

  // UART
  _PINNUM(0, 27), // D11 is UART TXD on P0.27
  _PINNUM(0, 26), // D12 is UART RXD on P0.26

  // A1-A7
  _PINNUM(0,  3), // D13 is A1 on P0.3
  _PINNUM(0,  4), // D14 is A2 on P0.4
  _PINNUM(0,  5), // D15 is A3 on P0.5
  _PINNUM(0, 28), // D16 is A4 on P0.28
  _PINNUM(0, 29), // D17 is A5 on P0.29
  _PINNUM(0, 30), // D18 is A6 on P0.30
  _PINNUM(0, 31), // D19 is A7 on P0.31

  // QSPI Flash
  _PINNUM(1,  4), // D20 is QSPI CLK on P1.4
  _PINNUM(1,  3), // D21 is QSPI CS on P1.3
  _PINNUM(1,  6), // D22 is QSPI Data 0 on P1.6
  _PINNUM(1,  1), // D23 is QSPI Data 1 on P1.1
  _PINNUM(1,  5), // D24 is QSPI Data 2 on P1.5
  _PINNUM(1,  2), // D25 is QSPI Data 3 on P1.2

  // ATWINC1500B Wi-Fi Module
  _PINNUM(0,  7), // D26 is WINC_SPI_MISO on P0.7
  _PINNUM(1,  8), // D27 is WINC_SPI_MOSI on P1.8
  _PINNUM(0,  8), // D28 is WINC_SPI_SCK on P0.8
  _PINNUM(0,  6), // D29 is WINC_IRQN_PIN on P0.6
  _PINNUM(0, 11), // D30 is WINC_SPI_SSN on P0.11
  _PINNUM(0, 12), // D31 is WINC_RSTN_PIN on P0.12
  _PINNUM(1, 14), // D32 is WINC_WAKE_PIN on P1.14
  _PINNUM(1,  9), // D33 is WINC_CHIP_EN_PIN on P1.9

  // Misc
  _PINNUM(1, 10), // D34 is Red LED1 on P1.10
  _PINNUM(1, 11), // D35 is Green LED2 on P1.11
  _PINNUM(1, 12), // D36 is Blue LED3 on P1.12
  _PINNUM(1,  0), // D37 is User Button on P1.0
  _PINNUM(0,  9), // D38 is NFC1 on P0.09
  _PINNUM(0, 10), // D39 is NFC2 on P0.10

    // Unused pin
  40, 41, 42, 43, 44, 45, 46, 47
};


void initVariant()
{
  // RGB LEDs
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);

  pinMode(PIN_LED3, OUTPUT);
  ledOff(PIN_LED3);
}

