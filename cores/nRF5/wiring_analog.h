/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*
 * \brief SAMD products have only one reference for ADC
 */
#if defined(NRF52) || defined(NRF52_SERIES)
typedef enum _eAnalogReference
{
  AR_DEFAULT,
  AR_INTERNAL,          // 0.6V Ref * 6 = 0..3.6V
  AR_INTERNAL_3_0,      // 0.6V Ref * 5 = 0..3.0V
  AR_INTERNAL_2_4,      // 0.6V Ref * 4 = 0..2.4V
  AR_INTERNAL_1_8,      // 0.6V Ref * 3 = 0..1.8V
  AR_INTERNAL_1_2,      // 0.6V Ref * 2 = 0..1.6V
  AR_VDD4               // VDD/4 REF * 4 = 0..VDD
} eAnalogReference ;
#else
typedef enum _eAnalogReference
{
  AR_DEFAULT,
  AR_VBG,
  AR_SUPPLY_ONE_HALF,
  AR_SUPPLY_ONE_THIRD,
  AR_EXT0,
  AR_EXT1
} eAnalogReference ;
#endif


/*
 * \brief Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * This function is kept only for compatibility with existing AVR based API.
 *
 * \param ulMmode Should be set to AR_DEFAULT.
 */
extern void analogReference( eAnalogReference ulMode ) ;

/*
* \brief Configures the oversampling amount used to sample analog input.
*
* \param ulOversampling Should be set to 1, 2, 4, 8, 16, 32, 64, 128 or 256.
*/
extern void analogOversampling( uint32_t ulOversampling );

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

/*
 * \brief Reads the value from the specified analog pin.
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead( uint32_t ulPin ) ;

/*
 * \brief Read the value from the vdd pin.
 *
 * \return Read value from vdd pin, if no error.
 */
extern uint32_t analogReadVDD( void ) ;

#ifdef SAADC_CH_PSELP_PSELP_VDDHDIV5
/*
 * \brief Read the value from the vddh pin with div5.
 *
 * \return Read value from vddh pin with div5, if no error.
 */
extern uint32_t analogReadVDDHDIV5( void ) ;
#endif

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(uint8_t res);

/*
 * \brief Set the ADC sample time. Default is 3us (appropriate for low source resistance).
 * See the chart of appropriate sample time vs. source resistance in the SAADC section
 * of the Nordic nrf52 product specification.
 *
 * \param time Should be set to 3, 5, 10, 15, 20 or 40.
 */
extern void analogSampleTime(uint8_t sTime);

/*
 * \brief Calibrate the ADC offset. This is recommended occasionally, or any time the
 * chip temperature changes by more than 10 C.  See the SAADC section of the Nordic
 * nrf52 product pecification.
 * 
 */
extern void analogCalibrateOffset( void );

extern void analogOutputInit( void ) ;

#ifdef __cplusplus
}
#endif
