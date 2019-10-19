/*
 * SPI Master library for nRF5x.
 * Copyright (c) 2015 Arduino LLC
 * Copyright (c) 2016 Sandeep Mistry All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

#include "nrfx_spim.h"

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03


class SPISettings {
  public:
    SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) {
      this->clockFreq = clock;
      this->bitOrder = bitOrder;
      this->dataMode = dataMode;
    }

    // Default speed set to 4MHz, SPI mode set to MODE 0 and Bit order set to MSB first.
    SPISettings() {
      this->clockFreq = 4000000;
      this->bitOrder = MSBFIRST;
      this->dataMode = SPI_MODE0;
    }

  private:
    uint32_t clockFreq;
    uint8_t  dataMode;
    uint8_t  bitOrder;

    friend class SPIClass;
};

class SPIClass {
  public:
    SPIClass(NRF_SPIM_Type *p_spi, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI);

    byte transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void transfer(void *buf, size_t count);
    void transfer(const void *tx_buf, void *rx_buf, size_t count);

    // Transaction Functions
    void usingInterrupt(int interruptNumber);
    void beginTransaction(SPISettings settings);
    void endTransaction(void);

    // SPI Configuration methods
    void attachInterrupt();
    void detachInterrupt();

    void begin();
    void end();

    void setBitOrder(BitOrder order);
    void setDataMode(uint8_t uc_mode);
    void setClockDivider(uint32_t uc_div);

  private:
    nrfx_spim_t _spim;
    NRF_SPI_Type *_p_spi;

    uint8_t _uc_pinMiso;
    uint8_t _uc_pinMosi;
    uint8_t _uc_pinSCK;

    uint8_t _dataMode;
    uint8_t _bitOrder;

    bool initialized;
};

#if SPI_INTERFACES_COUNT > 0
extern SPIClass SPI;
#endif

#if SPI_INTERFACES_COUNT > 1
extern SPIClass SPI1;
#endif

// For compatibility with sketches designed for AVR @ 64 MHz
// New programs should use SPI.beginTransaction to set the SPI clock
#if F_CPU == 64000000 // feather52 run @ 64Mhz
  #define SPI_CLOCK_DIV2   2
  #define SPI_CLOCK_DIV4   4
  #define SPI_CLOCK_DIV8   8
  #define SPI_CLOCK_DIV16  16
  #define SPI_CLOCK_DIV32  32
  #define SPI_CLOCK_DIV64  64
  #define SPI_CLOCK_DIV128 128
  #define SPI_CLOCK_DIV256 256
  #define SPI_CLOCK_DIV512 512
#endif

#endif
