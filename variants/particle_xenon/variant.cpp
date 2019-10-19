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
  26,   // P0.26 (SDA)
  27,   // P0.27 (SCL)
  33,   // P1.01 (SDA1) (UART1_RTS) (SPI1_SCK) (PWM3)
  34,   // P1.02 (SCL1) (UART1_CTS) (SPI1_MOSI) (PWM3)
  40,   // P1.08 (UART2_TX) (SPI1_MISO) (PWM1)
  42,   // P1.10 (UART2_RX) (PWM1)
  43,   // P1.11 (UART2_CTS) (PWM1)
  44,   // P1.12 (PWM0)
  35,   // P1.03 (UART2_RTS)

  // D9 .. D20
  6,    // P0.06 (UART1_TX)
  8,    // P0.27 (UART1_RX)
  46,   // P1.14 (SPI_MISO)
  45,   // P1.13 (SPI_MOSI)
  47,   // P1.15 (SPI_SCK)
  31,   // P0.31 (PWM3) (SPI_SS) (ADC5)
  30,   // P0.30 (PWM3) (ADC4)
  29,   // P0.29 (PWM2) (ADC3)
  28,   // P0.28 (PWM2) (ADC2)
  4,    // P0.04 (PWM2) (ADC1)
  3,    // P0.03 (PWM2) (ADC0)
  11,   // P0.11 (MODE)
  18,   // P0.18 (RESET)
};

void initVariant()
{
  switch_antenna(false);
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

void led_pwm_init(uint32_t led_index, uint32_t led_pin) {
  NRF_PWM_Type* pwm    = NRF_PWM0;

  pwm->ENABLE = 0;

  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[led_index] = led_pin;

  pwm->MODE            = PWM_MODE_UPDOWN_Up;
  pwm->COUNTERTOP      = 0xff;
  pwm->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_16;
  pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  pwm->LOOP            = 0;

  pwm->SEQ[0].PTR      = (uint32_t) (led_duty_cycles);
  pwm->SEQ[0].CNT      = 4; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH  = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->ENABLE = 1;

  pwm->EVENTS_SEQEND[0] = 0;
//  pwm->TASKS_SEQSTART[0] = 1;
}

void led_pwm_teardown(void) {
  pwm_teardown(NRF_PWM0);
}

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle) {
  led_duty_cycles[led_index] = duty_cycle;
  nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
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