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
#include "nrf_pwm.h"

const uint32_t g_ADigitalPinMap[] =
{
  // D0 .. D8
  _PINNUM(0, 26),   // P0.26 (SDA)
  _PINNUM(0, 27),   // P0.27 (SCL)
  _PINNUM(1, 1),    // P1.01 (SDA1) (UART1_RTS) (SPI1_SCK) (PWM3)
  _PINNUM(1, 2),    // P1.02 (SCL1) (UART1_CTS) (SPI1_MOSI) (PWM3)
  _PINNUM(1, 8),    // P1.08 (UART2_TX) (SPI1_MISO) (PWM1)
  _PINNUM(1, 10),   // P1.10 (UART2_RX) (PWM1)
  _PINNUM(1, 26),   // P1.11 (UART2_CTS) (PWM1)
  _PINNUM(1, 26),   // P1.12 (PWM0)
  _PINNUM(1, 26),   // P1.03 (UART2_RTS)

  // D9 .. D20
  _PINNUM(0, 6),    // P0.06 (UART1_TX)
  _PINNUM(0, 8),    // P0.08 (UART1_RX)
  _PINNUM(1, 14),   // P1.14 (SPI_MISO)
  _PINNUM(1, 13),   // P1.13 (SPI_MOSI)
  _PINNUM(1, 15),   // P1.15 (SPI_SCK)
  _PINNUM(0, 31),   // P0.31 (PWM3) (SPI_SS) (ADC5)
  _PINNUM(0, 30),   // P0.30 (PWM3) (ADC4)
  _PINNUM(0, 29),   // P0.29 (PWM2) (ADC3)
  _PINNUM(0, 28),   // P0.28 (PWM2) (ADC2)
  _PINNUM(0, 4),    // P0.04 (PWM2) (ADC1)
  _PINNUM(0, 3),    // P0.03 (PWM2) (ADC0)
  _PINNUM(0, 11),   // P0.11 (MODE)
  _PINNUM(0, 18),   // P0.18 (RESET)
};

void initVariant()
{
  switch_antenna(false);

  led_pwm_init(LED_PRIMARY_IDX, LED_PRIMARY_PIN);
  led_pwm_init(LED_RGB_RED_IDX, LED_RGB_RED_PIN);
  led_pwm_init(LED_RGB_BLUE_IDX, LED_RGB_BLUE_PIN);
  led_pwm_init(LED_RGB_GREEN_IDX, LED_RGB_GREEN_PIN);
}

void switch_antenna(bool useExternal) {
  if (useExternal) {
    digitalWrite(ANTENNA_SWITCH_1, LOW);
    digitalWrite(ANTENNA_SWITCH_2, HIGH);
  }
  else {
    digitalWrite(ANTENNA_SWITCH_1, HIGH);
    digitalWrite(ANTENNA_SWITCH_2, LOW);
  }
}

void initVariant()
{
  switch_antenna(false);
}

void ledWrite(uint32_t led_pin, uint8_t value) {
  uint32_t index = -1;
  switch (led_pin) {
    case LED_PRIMARY_PIN:
      index = LED_PRIMARY_IDX;
      break;
    case LED_RGB_RED_PIN:
      index = LED_RGB_RED_IDX;
      break;
    case LED_RGB_GREEN_PIN:
      index = LED_RGB_GREEN_IDX;
      break;
    case LED_RGB_BLUE_PIN:
      index = LED_RGB_BLUE_IDX;
      break;
  }

  if (index != -1)
    led_pwm_duty_cycle(index, value);
}

void rgbLedWrite(uint8_t red, uint8_t green, uint8_t blue) {
  led_pwm_duty_cycle(LED_RGB_RED_IDX, red);
  led_pwm_duty_cycle(LED_RGB_GREEN_IDX, green);
  led_pwm_duty_cycle(LED_RGB_BLUE_IDX, blue);
}

void pwm_teardown(NRF_PWM_Type* pwm) {
  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE            = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;
  pwm->PSEL.OUT[1] = 0xFFFFFFFF;
  pwm->PSEL.OUT[2] = 0xFFFFFFFF;
  pwm->PSEL.OUT[3] = 0xFFFFFFFF;

  pwm->MODE        = 0;
  pwm->COUNTERTOP  = 0x3FF;
  pwm->PRESCALER   = 0;
  pwm->DECODER     = 0;
  pwm->LOOP        = 0;
  pwm->SEQ[0].PTR  = 0;
  pwm->SEQ[0].CNT  = 0;
}
