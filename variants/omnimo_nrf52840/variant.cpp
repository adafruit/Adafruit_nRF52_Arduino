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
  25,  // D0  is P0.25 (Feather UART TX, mikroBUS MISO)
  24,  // D1  is P0.24 (Feather UART RX, mikroBUS SCK) 
  41,  // D2  is P1.09 (Feather D2, mikroBUS MOSI)    /// 10,  // D2  is P0.10 (NFC2)
  42,  // D3  is P1.10 (LED1)
  47,  // D4  is P1.15 (LED2)
  40,  // D5  is P1.08 (Feather D5, mikroBUS TX)
   7,  // D6  is P0.07 (Feather D6, mikroBUS RX)
  34,  // D7  is P1.02 (BTN1)      /// 34,  // D7  is P1.02 (Button)
  16,  // D8  is P0.16 (NeoPixel)
  26,  // D9  is P0.26 (Feather D9,  mikroBUS INT)
  27,  // D10 is P0.27 (Feather D10, mikroBUS PWM)
  46,  // D11 is P1.14 (Feather D11)        ///  6,  // D11 is P0.06
  45,  // D12 is P1.13 (Feather D12)        ///  8,  // D12 is P0.08
  44,  // D13 is P1.12 (Feather D13)        /// 41,  // D13 is P1.09

  // D14 .. D21 (aka A0 .. A7)
   4,  // D14 is P0.04 (Feather A0, PMOD4)
   5,  // D15 is P0.05 (Feather A1, PMOD3)
  30,  // D16 is P0.30 (Feather A2, PMOD2)
  28,  // D17 is P0.28 (Feather A3, PMOD1)
   2,  // D18 is P0.02 (Feather A4)
   3,  // D19 is P0.03 (Feather A5)
  29,  // D20 is P0.29 (A6, Battery Voltage Monitoring)
  31,  // D21 is P0.31 (A7, mikroBUS AN)    /// ARef 

  // D22 .. D23 (aka I2C pins)
  12,  // D22 is P0.12 (Feather SDA, mikroBUS SDA)
  11,  // D23 is P0.11 (Feather SCL, mikroBUS SCL)

  // D24 .. D26 (aka SPI pins)
  15,  // D24 is P0.15 (Feather MISO, mikroBUS CS)
  13,  // D25 is P0.13 (Feather MOSI, mikroBUS RST)
  14,  // D26 is P0.14 (Feather SCK)

  // QSPI pins (not exposed via any header / test point)
  19,  // D27 is P0.19 (QSPI CLK)
  20,  // D28 is P0.20 (QSPI CS)
  17,  // D29 is P0.17 (QSPI Data 0)
  22,  // D30 is P0.22 (QSPI Data 1)
  23,  // D31 is P0.23 (QSPI Data 2)
  21,  // D32 is P0.21 (QSPI Data 3)

  // NFC pins
   9,  // D33 is P0.09 (NFC1)
  10,  // D34 is P0.10 (NFC2)

  // Qwiic/Stemma QT
   6,  // D35 is P0.06 (Qwiic SCL)
   8,  // D36 is P0.08 (Qwiic SDA)

  39,  // D37 is P1.07 (BTN2)

  36,  // D38 is P1.04 (VOUTEN)

  // PMOD pin5 to 8
  43,  // D39 is P1.11 (PMOD5)
  33,  // D40 is P1.01 (PMOD6)
  35,  // D41 is P1.03 (PMOD7)
  37,  // D42 is P1.05 (PMOD8)

  38,  // D43 is P1.06 is not connected


  // Thus, there are 44 defined pins


  // The following pins were never listed as they were considered unusable
  //  0, // P0.00 is XL1   (attached to 32.768kHz crystal)
  //  1, // is P0.01 is XL2   (attached to 32.768kHz crystal)
  // 18, // is P0.18 is RESET (attached to switch)
  // 32, // is P1.00 is SWO   (attached to debug header)
  // 

};

void initVariant()
{
  // LED1 & LED2
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);
}

