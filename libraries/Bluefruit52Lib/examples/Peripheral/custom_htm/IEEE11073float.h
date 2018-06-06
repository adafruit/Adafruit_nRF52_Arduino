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

#ifndef _IEEE11073FLOAT_H_
#define _IEEE11073FLOAT_H_

#include <stdint.h>

typedef enum {
  MDER_POSITIVE_INFINITY = 0x007FFFFE,
  MDER_NaN = 0x007FFFFF,
  MDER_NRes = 0x00800000,
  MDER_RESERVED_VALUE = 0x00800001,
  MDER_NEGATIVE_INFINITY = 0x00800002
} ReservedFloatValues;
static const int32_t FIRST_RESERVED_VALUE = MDER_POSITIVE_INFINITY;

// (2 ** 23 - 3)
#define MDER_FLOAT_MANTISSA_MAX 0x007FFFFD
// 2 ** 7 - 1
#define MDER_FLOAT_EXPONENT_MAX 127
#define MDER_FLOAT_EXPONENT_MIN -128
// (2 ** 23 - 3) * 10 ** 127
#define MDER_FLOAT_MAX 8.388604999999999e+133
// -(2 ** 23 - 3) * 10 ** 127
#define MDER_FLOAT_MIN (-MDER_FLOAT_MAX)
// 10 ** -128
#define MDER_FLOAT_EPSILON 1e-128
// 10 ** upper(23 * log(2) / log(10))
// precision for a number 1.0000xxx
#define MDER_FLOAT_PRECISION 10000000

typedef enum {
  MDER_S_POSITIVE_INFINITY = 0x07FE,
  MDER_S_NaN = 0x07FF,
  MDER_S_NRes = 0x0800,
  MDER_S_RESERVED_VALUE = 0x0801,
  MDER_S_NEGATIVE_INFINITY = 0x0802
} ReservedSFloatValues;
static const uint32_t FIRST_S_RESERVED_VALUE = MDER_S_POSITIVE_INFINITY;

// (2 ** 11 - 3)
#define MDER_SFLOAT_MANTISSA_MAX 0x07FD
// 2 ** 3 - 1
#define MDER_SFLOAT_EXPONENT_MAX 7
#define MDER_SFLOAT_EXPONENT_MIN -8
// (2 ** 11 - 3) * 10 ** 7
#define MDER_SFLOAT_MAX 20450000000.0
// -(2 ** 11 - 3) * 10 ** 7
#define MDER_SFLOAT_MIN (-MDER_SFLOAT_MAX)
// 10 ** -8
#define MDER_SFLOAT_EPSILON 1e-8
// 10 ** upper(11 * log(2) / log(10))
#define MDER_SFLOAT_PRECISION 10000

uint32_t float2IEEE11073(double data, uint8_t output[4]);

#endif /* _IEEE11073FLOAT_H_ */
