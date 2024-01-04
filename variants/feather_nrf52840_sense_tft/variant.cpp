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

#define PINNUM(port, pin)    ((port)*32 + (pin))

const uint32_t g_ADigitalPinMap[] = {
    // D0 .. D13
    PINNUM(1, 11),  // D0  is P1.11 (UART TX)
    PINNUM(1, 13),  // D1  is P1.13 (UART RX)
    PINNUM(1, 10),  // D2  is P1.10 (DFU)
    PINNUM(1,  6),  // D3  is P1.06 (Button)
    PINNUM(0,  6),  // D4  is P0.06 (LSM6DS3 IRQ)
    PINNUM(0, 20),  // D5  is P0.20
    PINNUM(0, 16),  // D6  is P0.16
    PINNUM(1,  2),  // D7  is P1.02 (Neopixel Power)
    PINNUM(1,  8),  // D8  is P1.08 (NeoPixel)
    PINNUM(0, 15),  // D9  is P0.15
    PINNUM(0, 13),  // D10 is P0.13
    PINNUM(0, 14),  // D11 is P0.14
    PINNUM(0, 12),  // D12 is P0.12
    PINNUM(0, 11),  // D13 is P0.11 (LED1)

    // D14 .. D20 (aka A0 .. A6)
    PINNUM(0,  3),  // D14 is P0.03 (A0)
    PINNUM(0,  2),  // D15 is P0.02 (A1)
    PINNUM(0, 28),  // D16 is P0.28 (A2)
    PINNUM(0, 30),  // D17 is P0.30 (A3)
    PINNUM(0, 31),  // D18 is P0.31 (A4)
    PINNUM(0, 29),  // D19 is P0.29 (A5)
    PINNUM(0,  4),  // D20 is P0.04 (A6, Battery)

    PINNUM(1,  7),  // D21 is P1.07 (Sensor Power)

    // D22 .. D23 (aka I2C pins)
    PINNUM(0, 25),  // D22 is P0.25 (SDA)
    PINNUM(0, 24),  // D23 is P0.24 (SCL)

    // D24 .. D26 (aka SPI pins)
    PINNUM(0,  7),  // D24 is P0.07 (SPI MISO)
    PINNUM(0,  5),  // D25 is P0.05 (SPI MOSI)
    PINNUM(0, 26),  // D26 is P0.26 (SPI SCK )

    // D27 .. D32 QSPI pins (not exposed via any header / test point)
    PINNUM(0, 19),  // D27 is P0.19 (QSPI CLK)
    PINNUM(0, 23),  // D28 is P0.23 (QSPI CS)
    PINNUM(0, 21),  // D29 is P0.21 (QSPI Data 0)
    PINNUM(0, 22),  // D30 is P0.22 (QSPI Data 1)
    PINNUM(1,  0),  // D31 is P1.00 (QSPI Data 2)
    PINNUM(0, 17),  // D32 is P0.17 (QSPI Data 3)

    // D33 .. D34 PDM pins
    PINNUM(0,  8),   // D33 P0.08 is PDM Data
    PINNUM(1,  9),   // D34 P1.09 is PDM Clock

    // D35 .. D38 TFT
    PINNUM(1,  5),   // D35 P1.05 is TFT CS
    PINNUM(1,  1),   // D36 P1.01 is TFT DC
    PINNUM(1,  3),   // D37 P1.03 is TFT Reset
    PINNUM(0, 27),   // D38 P0.27 is TFT Backlight

    // D39 .. D40 NFC
    PINNUM(0,  9),   // D39 is P0.09 (NFC1)
    PINNUM(0, 10),   // D40 is P0.10 (NFC2)
};

void initVariant()
{
  // power off LED, neopixel, sensor, tft backlight
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
  pinMode(PIN_SENSOR_POWER, OUTPUT);
  pinMode(PIN_TFT_LITE, OUTPUT);

  ledOff(PIN_LED1);
  digitalWrite(PIN_NEOPIXEL_POWER, LOW);
  digitalWrite(PIN_SENSOR_POWER, LOW);
  digitalWrite(PIN_TFT_LITE, LOW);
}

