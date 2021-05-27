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
//  _PINNUM(0, 0),  // P0.00 is XL1 (attached to 32.768kHz crystal)
//  _PINNUM(0, 1),  // P0.01 is XL2 (attached to 32.768kHz crystal)  
  _PINNUM(0, 2),    // P0.02 is BAT_VSENSE_PIN (AIN0, VBAT)          #0
  _PINNUM(0, 3),    // P0.03 is AIN1 (A1)                            #1
  _PINNUM(0, 4),    // P0.04 is AIN2 (A2)                            #2
  _PINNUM(0, 5),    // P0.05 is AIN3 (A3)                            #3
  _PINNUM(0, 13),   // P0.13 is P13                                  #4
  _PINNUM(0, 14),   // P0.14 is P14                                  #5
  _PINNUM(0, 15),   // P0.15 is P15                                  #6
  _PINNUM(0, 16),   // P0.16 is P16                                  #7
  _PINNUM(0, 17),   // P0.17 is P17                                  #8                              
  _PINNUM(0, 20),   // P0.20 is P20                                  #9
  _PINNUM(0, 21),   // P0.21 is P21                                  #10
  _PINNUM(0, 22),   // P0.22 is P22                                  #11
  _PINNUM(0, 23),   // P0.23 is P23                                  #12
  _PINNUM(0, 24),   // P0.24 is P24                                  #13
  _PINNUM(0, 25),   // P0.25 is P25                                  #14
  _PINNUM(0, 26),   // P0.26 is P26 (RXD)                            #15
  _PINNUM(0, 27),   // P0.27 is P27 (TXD)                            #16
  _PINNUM(0, 28),   // P0.28 is AIN4 (A4)                            #17
  _PINNUM(0, 29),   // P0.29 is AIN5 (A5)                            #18
  _PINNUM(0, 30),   // P0.30 is AIN6 (A6)                            #19
  _PINNUM(0, 31),   // P0.31 is AIN7 (A7)                            #20
  _PINNUM(1, 0),    // P1.00 is MODE_BTN_PIN (USER BUTTON)           #21
  _PINNUM(1, 1),    // P1.01 is NRF_QSPI_SIO1                        #22
  _PINNUM(1, 2),    // P1.02 is NRF_QSPI_SIO3                        #23
  _PINNUM(1, 3),    // P1.03 is NRF_QSPI_CSN                         #24
  _PINNUM(1, 4),    // P1.04 is NRF_QSPI_CLK                         #25
  _PINNUM(1, 5),    // P1.05 is NRF_QSPI_SIO2                        #26
  _PINNUM(1, 6),    // P1.06 is NRF_QSPI_SIO0                        #27
  _PINNUM(1, 10),   // P1.10 is LED_R_PIN                            #28
  _PINNUM(1, 11),   // P1.11 is LED_G_PIN                            #29
  _PINNUM(1, 12),   // P1.12 is LED_B_PIN                            #30
  _PINNUM(0, 9),    // P0.09 is NFC1                                 #31
  _PINNUM(0, 10),   // P0.10 is NFC2                                 #32
  _PINNUM(0, 6),    // P0.06 is WINC_IRQN_PIN                        #33
  _PINNUM(0, 7),    // P0.07 is WINC_SPI_MISO                        #34
  _PINNUM(0, 8),    // P0.08 is WINC_SPI_SCK                         #35                     
  _PINNUM(0, 11),   // P0.11 is WINC_SPI_SSN                         #36
  _PINNUM(0, 12),   // P0.12 is WINC_RSTN_PIN                        #37
  _PINNUM(1, 8),    // P1.08 is WINC_SPI_MOSI                        #38
  _PINNUM(1, 9),    // P1.09 is WINC_CHIP_EN_PIN                     #39
  _PINNUM(1, 14),   // P1.14 is WINC_WAKE_PIN                        #40
//  _PINNUM(1, 13), // P1.13 is BAT_CHRG_PIN
//  _PINNUM(1, 15), // P1.15 is BAT_PGOOD_PIN (POWER GOOD LED)
//  _PINNUM(0, 18), // P0.18 is RESET (RESET BUTTON)
};



void initVariant()
{
  // RGB LED
  pinMode(PIN_LED1, OUTPUT);
  ledOff(PIN_LED1);

  pinMode(PIN_LED2, OUTPUT);
  ledOff(PIN_LED2);

  pinMode(PIN_LED3, OUTPUT);
  ledOff(PIN_LED3);
}

