/**************************************************************************/
/*!
    @file     IEEE11073float.h
*/
/**************************************************************************/

/**
 * \file bytelib.c
 * \brief Byte manipulation module implementation.
 * Copyright (C) 2010 Signove Tecnologia Corporation.
 * All rights reserved.
 * Contact: Signove Tecnologia Corporation (contact@signove.com)
 *
 * $LICENSE_TEXT:BEGIN$
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation and appearing
 * in the file LICENSE included in the packaging of this file; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 * $LICENSE_TEXT:END$
 *
 * \author Walter Guerra, Mateus Lima
 * \date Jun 14, 2010
 */

#include <Arduino.h>
#include "IEEE11073float.h"

uint32_t float2IEEE11073(double data, uint8_t output[4])
{
  uint32_t result = MDER_NaN;


  if (isnan(data)) {
    goto finally;
  }/* else if (data > MDER_FLOAT_MAX) {
    result = MDER_POSITIVE_INFINITY;
    goto finally;
  } else if (data < MDER_FLOAT_MIN) {
    result = MDER_NEGATIVE_INFINITY;
    goto finally;
  } else if (data >= -MDER_FLOAT_EPSILON &&
    data <= MDER_FLOAT_EPSILON) {
    result = 0;
    goto finally;
  }*/

  double sgn; sgn = data > 0 ? +1 : -1;
  double mantissa; mantissa = fabs(data);
  int32_t exponent; exponent = 0; // Note: 10**x exponent, not 2**x

  // scale up if number is too big
  while (mantissa > MDER_FLOAT_MANTISSA_MAX) {
    mantissa /= 10.0;
    ++exponent;
    if (exponent > MDER_FLOAT_EXPONENT_MAX) {
      // argh, should not happen
      if (sgn > 0) {
        result = MDER_POSITIVE_INFINITY;
      } else {
        result = MDER_NEGATIVE_INFINITY;
      }
      goto finally;
    }
  }

  // scale down if number is too small
  while (mantissa < 1) {
    mantissa *= 10;
    --exponent;
    if (exponent < MDER_FLOAT_EXPONENT_MIN) {
      // argh, should not happen
      result = 0;
      goto finally;
    }
  }

  // scale down if number needs more precision
  double smantissa; smantissa = round(mantissa * MDER_FLOAT_PRECISION);
  double rmantissa; rmantissa = round(mantissa) * MDER_FLOAT_PRECISION;
  double mdiff; mdiff = abs(smantissa - rmantissa);
  while (mdiff > 0.5 && exponent > MDER_FLOAT_EXPONENT_MIN &&
      (mantissa * 10) <= MDER_FLOAT_MANTISSA_MAX) {
    mantissa *= 10;
    --exponent;
    smantissa = round(mantissa * MDER_FLOAT_PRECISION);
    rmantissa = round(mantissa) * MDER_FLOAT_PRECISION;
    mdiff = abs(smantissa - rmantissa);
  }

  uint32_t int_mantissa; int_mantissa = (int) round(sgn * mantissa);
  result = (exponent << 24) | (int_mantissa & 0xFFFFFF);

finally:
  if ( output ) memcpy(output, &result, 4);
  return result;
}
