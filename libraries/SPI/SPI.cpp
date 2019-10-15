/*
 * SPI Master library for nRF5x.
 * Copyright (c) 2015 Arduino LLC
 * Copyright (c) 2016 Sandeep Mistry All right reserved.
 * Copyright (c) 2019 Ha Thach for Adafruit Industries. All right reserved.
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

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>
#include <assert.h>

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

SPIClass::SPIClass(NRF_SPI_Type *p_spi, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI)
{
  initialized = false;
  assert(p_spi != NULL);
  _p_spi = p_spi;

  _spim.p_reg = NRF_SPIM0;
  _spim.drv_inst_idx = NRFX_SPIM0_INST_IDX;

  // pins
  _uc_pinMiso = g_ADigitalPinMap[uc_pinMISO];
  _uc_pinSCK = g_ADigitalPinMap[uc_pinSCK];
  _uc_pinMosi = g_ADigitalPinMap[uc_pinMOSI];

  _dataMode = SPI_MODE0;
  _bitOrder = SPI_CONFIG_ORDER_MsbFirst;
}

void SPIClass::begin()
{
  if (initialized) return;
  initialized = true;

  nrfx_spim_config_t cfg =
  {
    .sck_pin        = _uc_pinSCK,
    .mosi_pin       = _uc_pinMosi,
    .miso_pin       = _uc_pinMiso,
    .ss_pin         = NRFX_SPIM_PIN_NOT_USED,
    .ss_active_high = false,
    .irq_priority   = 3,
    .orc            = 0xFF,
    .frequency      = NRF_SPIM_FREQ_8M, // NRF_SPIM_FREQ_4M,
    .mode           = NRF_SPIM_MODE_0,
    .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
  };

  // blocking
  nrfx_spim_init(&_spim, &cfg, NULL, NULL);

  _p_spi->PSELSCK  = _uc_pinSCK;
  _p_spi->PSELMOSI = _uc_pinMosi;
  _p_spi->PSELMISO = _uc_pinMiso;

  config(DEFAULT_SPI_SETTINGS);
}

void SPIClass::config(SPISettings settings)
{
#if 0
  _p_spi->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);

  uint32_t config = settings.bitOrder;

  switch (settings.dataMode) {
    default:
    case SPI_MODE0:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE1:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE2:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE3:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;
  }

  _p_spi->CONFIG = config;
  _p_spi->FREQUENCY = settings.clockFreq;

  _p_spi->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
#endif
}

void SPIClass::end()
{
//  _p_spi->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);

  nrfx_spim_uninit(&_spim);

  initialized = false;
}

void SPIClass::usingInterrupt(int /*interruptNumber*/)
{
}

void SPIClass::beginTransaction(SPISettings settings)
{
//  config(settings);
}

void SPIClass::endTransaction(void)
{
//  _p_spi->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
}

void SPIClass::setBitOrder(BitOrder order)
{
#if 0
  this->_bitOrder = (order == MSBFIRST ? SPI_CONFIG_ORDER_MsbFirst : SPI_CONFIG_ORDER_LsbFirst);

  uint32_t config = this->_bitOrder;

  switch (this->_dataMode) {
    default:
    case SPI_MODE0:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE1:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE2:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE3:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;
  }

  _p_spi->CONFIG = config;
#endif
}

void SPIClass::setDataMode(uint8_t mode)
{
#if 0
  this->_dataMode = mode;

  uint32_t config = this->_bitOrder;

  switch (this->_dataMode) {
    default:
    case SPI_MODE0:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE1:
      config |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE2:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Leading    << SPI_CONFIG_CPHA_Pos);
      break;

    case SPI_MODE3:
      config |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
      config |= (SPI_CONFIG_CPHA_Trailing   << SPI_CONFIG_CPHA_Pos);
      break;
  }

  _p_spi->CONFIG = config;
#endif
}

void SPIClass::setClockDivider(uint8_t div)
{
#if 0
  uint32_t clockFreq;

  // Adafruit Note: nrf52 run at 64MHz
  if (div >= SPI_CLOCK_DIV512) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_K125;
  } else if (div >= SPI_CLOCK_DIV256) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_K250;
  } else if (div >= SPI_CLOCK_DIV128) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_K500;
  } else if (div >= SPI_CLOCK_DIV64) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_M1;
  } else if (div >= SPI_CLOCK_DIV32) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_M2;
  } else if (div >= SPI_CLOCK_DIV16) {
    clockFreq = SPI_FREQUENCY_FREQUENCY_M4;
  } else {
    clockFreq = SPI_FREQUENCY_FREQUENCY_M8;
  }

  _p_spi->FREQUENCY = clockFreq;
#endif
}

void SPIClass::transfer(void *buf, size_t count)
{
#if 0
  // TODO: Optimize for faster block-transfer
  uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
  for (size_t i=0; i<count; i++)
    buffer[i] = transfer(buffer[i]);
#else
  nrfx_spim_xfer_desc_t xfer_desc =
  {
    .p_tx_buffer = (uint8_t*) buf,
    .tx_length   = count,
    .p_rx_buffer = (uint8_t*) buf,
    .rx_length   = count,
  };

  nrfx_spim_xfer(&_spim, &xfer_desc, 0);
#endif
}

byte SPIClass::transfer(uint8_t data)
{
#if 0
  _p_spi->TXD = data;

  while(!_p_spi->EVENTS_READY);

  data = _p_spi->RXD;

  _p_spi->EVENTS_READY = 0x0UL;

  return data;
#else
  transfer(&data, 1);
  return data;
#endif
}

uint16_t SPIClass::transfer16(uint16_t data) {

  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

  t.val = data;

  if (_bitOrder == SPI_CONFIG_ORDER_LsbFirst) {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }

  return t.val;
}

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

#if SPI_INTERFACES_COUNT > 0
SPIClass SPI (NRF_SPI0,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI);
#endif

#if SPI_INTERFACES_COUNT > 1
SPIClass SPI1(NRF_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);
#endif
