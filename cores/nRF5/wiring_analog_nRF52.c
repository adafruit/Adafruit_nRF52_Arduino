/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

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

#if defined(NRF52) || defined(NRF52_SERIES)

#include "nrf.h"

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
static uint32_t saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_6;

static bool saadcBurst = SAADC_CH_CONFIG_BURST_Disabled;

// Note: Adafruit use seperated HardwarePWM class

static int readResolution = 10;
void analogReadResolution( int res )
{
  readResolution = res;
}

static inline uint32_t mapResolution( uint32_t value, uint32_t from, uint32_t to )
{
  if ( from == to )
  {
    return value ;
  }

  if ( from > to )
  {
    return value >> (from-to) ;
  }
  else
  {
    return value << (to-from) ;
  }
}

/*
 * Internal Reference is +/-0.6V, with an adjustable gain of 1/6, 1/5, 1/4,
 * 1/3, 1/2 or 1, meaning 3.6, 3.0, 2.4, 1.8, 1.2 or 0.6V for the ADC levels.
 *
 * External Reference is VDD/4, with an adjustable gain of 1, 2 or 4, meaning
 * VDD/4, VDD/2 or VDD for the ADC levels.
 *
 * Default settings are internal reference with 1/6 gain (GND..3.6V ADC range)
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */
void analogReference( eAnalogReference ulMode )
{
  switch ( ulMode ) {
    case AR_VDD4:
      saadcReference = SAADC_CH_CONFIG_REFSEL_VDD1_4;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_4;
      break;
    case AR_INTERNAL_3_0:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_5;
      break;
    case AR_INTERNAL_2_4:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_4;
      break;
    case AR_INTERNAL_1_8:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_3;
      break;
    case AR_INTERNAL_1_2:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_2;
      break;
    case AR_DEFAULT:
    case AR_INTERNAL:
    default:
      saadcReference = SAADC_CH_CONFIG_REFSEL_Internal;
      saadcGain      = SAADC_CH_CONFIG_GAIN_Gain1_6;
      break;

  }
}

void analogOversampling( uint32_t ulOversampling )
{
	saadcBurst = SAADC_CH_CONFIG_BURST_Enabled;

	switch (ulOversampling) {
		case 0:
		case 1:
			saadcBurst = SAADC_CH_CONFIG_BURST_Disabled;
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
			return;
			break;
		case 2:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over2x;
			break;
		case 4:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over4x;
			break;
		case 8:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over8x;
			break;
		case 16:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over16x;
			break;
		case 32:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over32x;
			break;
		case 64:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over64x;
			break;
		case 128:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over128x;
			break;
		case 256:
			NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Over256x;
			break;
	}
}

static uint32_t analogRead_internal( uint32_t psel )
{
  uint32_t saadcResolution;
  uint32_t resolution;
  volatile int16_t value = 0;

  if (readResolution <= 8) {
    resolution = 8;
    saadcResolution = SAADC_RESOLUTION_VAL_8bit;
  } else if (readResolution <= 10) {
    resolution = 10;
    saadcResolution = SAADC_RESOLUTION_VAL_10bit;
  } else if (readResolution <= 12) {
    resolution = 12;
    saadcResolution = SAADC_RESOLUTION_VAL_12bit;
  } else {
    resolution = 14;
    saadcResolution = SAADC_RESOLUTION_VAL_14bit;
  }

  NRF_SAADC->RESOLUTION = saadcResolution;

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }
  NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                            | ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                            | ((saadcGain                     << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                            | ((saadcReference                << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                            | ((SAADC_CH_CONFIG_TACQ_3us      << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                            | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
                            | ((saadcBurst                    << SAADC_CH_CONFIG_BURST_Pos)   & SAADC_CH_CONFIG_BURST_Msk);
  NRF_SAADC->CH[0].PSELN = psel;
  NRF_SAADC->CH[0].PSELP = psel;


  NRF_SAADC->RESULT.PTR = (uint32_t)&value;
  NRF_SAADC->RESULT.MAXCNT = 1; // One sample

  NRF_SAADC->TASKS_START = 0x01UL;

  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;

  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0x00UL;

  NRF_SAADC->TASKS_STOP = 0x01UL;

  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0x00UL;

  if (value < 0) {
    value = 0;
  }

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);

  return mapResolution(value, resolution, readResolution);
}


uint32_t analogRead( uint32_t ulPin )
{
  uint32_t psel = SAADC_CH_PSELP_PSELP_NC;

  if (ulPin >= PINS_COUNT) {
    return 0;
  }

  ulPin = g_ADigitalPinMap[ulPin];

  switch ( ulPin ) {
    case 2:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput0;
      break;

    case 3:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput1;
      break;

    case 4:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput2;
      break;

    case 5:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput3;
      break;

    case 28:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput4;
      break;

    case 29:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput5;
      break;

    case 30:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput6;
      break;

    case 31:
      psel = SAADC_CH_PSELP_PSELP_AnalogInput7;
      break;

    default:
      return 0;
  }

  return analogRead_internal(psel);
}

uint32_t analogReadVDD( void )
{
  return analogRead_internal(SAADC_CH_PSELP_PSELP_VDD);
}

#ifdef __cplusplus
}
#endif

#endif
