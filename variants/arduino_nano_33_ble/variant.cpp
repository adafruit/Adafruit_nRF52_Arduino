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

#define _P(X, Y) (X * 32 + Y)

const uint32_t g_ADigitalPinMap[] = {
  // D0 - D7
  _P(1, 3),     // D0/TX
  _P(1, 10),    // D1/RX
  _P(1, 11),    // D2
  _P(1, 12),    // D3
  _P(1, 15),    // D4
  _P(1, 13),    // D5
  _P(1, 14),    // D6
  _P(0, 23),     // D7

  // D8 - D13
  _P(0, 21),    // D8
  _P(0, 27),    // D9
  _P(1, 2),     // D10
  _P(1, 1),     // D11/MOSI
  _P(1, 8),     // D12/MISO
  _P(0, 13),    // D13/SCK/LED

  // A0 - A7
  _P(0, 4),     // A0
  _P(0, 5),     // A1
  _P(0, 30),    // A2
  _P(0, 29),    // A3
  _P(0, 31),    // A4/SDA
  _P(0, 2),     // A5/SCL
  _P(0, 28),    // A6
  _P(0, 3),     // A7

  // LEDs
  _P(0, 24),    // LED R
  _P(0, 16),    // LED G
  _P(0, 6),     // LED B
  _P(1, 9),     // LED PWR

  _P(0, 19),    // INT APDS

  // PDM
  _P(0, 17),    // PDM PWR
  _P(0, 26),    // PDM CLK
  _P(0, 25),    // PDM DIN

  // Internal I2C
  _P(0, 14),    // SDA2
  _P(0, 15),    // SCL2

  // Internal I2C
  _P(1, 0),     // I2C_PULL
  _P(0, 22),    // VDD_ENV_ENABLE
};

extern "C" {
  unsigned int PINCOUNT_fn() {
    return (sizeof(g_ADigitalPinMap) / sizeof(g_ADigitalPinMap[0]));
  }
}

#include "nrf_rtc.h"

void initVariant() {
  // turn power LED on
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);

  // Errata Nano33BLE - I2C pullup is on SWO line, need to disable TRACE
  // was being enabled by nrfx_clock_anomaly_132
  CoreDebug->DEMCR = 0;
  NRF_CLOCK->TRACECONFIG = 0;

  // FIXME: bootloader enables interrupt on COMPARE[0], which we don't handle
  // Disable it here to avoid getting stuck when OVERFLOW irq is triggered
  nrf_rtc_event_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
  nrf_rtc_int_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);

  // FIXME: always enable I2C pullup and power @startup
  // Change for maximum powersave
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
 
  NRF_PWM_Type* PWM[] = {
    NRF_PWM0, NRF_PWM1, NRF_PWM2
#ifdef NRF_PWM3
    ,NRF_PWM3
#endif
  };

  for (unsigned int i = 0; i < (sizeof(PWM)/sizeof(PWM[0])); i++) {
    PWM[i]->ENABLE = 0;
    PWM[i]->PSEL.OUT[0] = 0xFFFFFFFFUL;
  } 
}

#ifdef SERIAL_CDC

static void utox8(uint32_t val, uint8_t* s) {
  for (int i = 0; i < 16; i=i+2) {
    int d = val & 0XF;
    val = (val >> 4);

    s[15 - i -1] = d > 9 ? 'A' + d - 10 : '0' + d;
    s[15 - i] = '\0';
  }
}

uint8_t getUniqueSerialNumber(uint8_t* name) {
  #define SERIAL_NUMBER_WORD_0  NRF_FICR->DEVICEADDR[1]
  #define SERIAL_NUMBER_WORD_1  NRF_FICR->DEVICEADDR[0]

  utox8(SERIAL_NUMBER_WORD_0, &name[0]);
  utox8(SERIAL_NUMBER_WORD_1, &name[16]);

  return 32;
}

void _ontouch1200bps_() {
  __disable_irq();
  NRF_POWER->GPREGRET = DFU_MAGIC_SERIAL_ONLY_RESET;
  NVIC_SystemReset();
}

#endif
