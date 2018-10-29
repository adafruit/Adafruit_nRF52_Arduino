/**************************************************************************/
/*!
    @file     flash_qspi.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifdef NRF52840_XXAA

#include "Arduino.h"
#include "flash_devices.h"
#include "flash_qspi.h"
#include "flash_cache.h"
#include "nrfx_qspi.h"

#define _VALID_PIN(n)       (defined(PIN_QSPI_DATA##n) && (PIN_QSPI_DATA##n != 0xff))

#define QSPI_FLASH_MODE     \
        (( _VALID_PIN(0) && _VALID_PIN(1) && _VALID_PIN(2) && _VALID_PIN(3) ) ? 4 : \
         ( _VALID_PIN(0) && _VALID_PIN(1) ) ? 2 : \
         ( _VALID_PIN(0) ) ? 1 : 0)

#if QSPI_FLASH_MODE

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
enum
{
  QSPI_CMD_RSTEN = 0x66,
  QSPI_CMD_RST = 0x99,
  QSPI_CMD_WRSR = 0x01,
  QSPI_CMD_READID = 0x90
};

// If Flash device is not specified, support all devices in flash_devices.h
#ifdef EXTERNAL_FLASH_DEVICES
const qspi_flash_device_t _flash_devices_arr[] = { EXTERNAL_FLASH_DEVICES };
#else
const qspi_flash_device_t _flash_devices_arr[] = { GD25Q16C, MX25R6435F };
#endif

enum
{
  FLASH_DEVICE_COUNT = arrcount(_flash_devices_arr)
};

const qspi_flash_device_t* _flash_dev = NULL;

// Flash Abstraction Layer
static bool fal_qspi_erase (uint32_t addr);
static uint32_t fal_qspi_program (uint32_t dst, void const * src, uint32_t len);
static uint32_t fal_qspi_read (void* dst, uint32_t src, uint32_t len);
static bool fal_qspi_verify (uint32_t addr, void const * buf, uint32_t len);

static uint8_t _cache_buffer[FLASH_CACHE_SIZE] __attribute__((aligned(4)));

static flash_cache_t _cache = {
  .erase = fal_qspi_erase,
  .program = fal_qspi_program,
  .read = fal_qspi_read,
  .verify = fal_qspi_verify,
  .cache_addr = FLASH_CACHE_INVALID_ADDR,
  .cache_buf = _cache_buffer
};

static SemaphoreHandle_t _qspi_mutex;

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+
uint32_t flash_qspi_size (void)
{
  VERIFY(_flash_dev, 0);
  return _flash_dev->total_size;
}

uint32_t flash_qspi_write (uint32_t dst, void const * src, uint32_t len)
{
  VERIFY(_flash_dev, 0);

  xSemaphoreTake(_qspi_mutex, portMAX_DELAY);
  uint32_t res = flash_cache_write(&_cache, dst, src, len);
  xSemaphoreGive(_qspi_mutex);

  return res;
}

uint32_t flash_qspi_read (void* dst, uint32_t src, uint32_t len)
{
  VERIFY(_flash_dev, 0);

//  xSemaphoreTake(_qspi_mutex, portMAX_DELAY);
  flash_cache_read(&_cache, dst, src, len);
//  xSemaphoreGive(_qspi_mutex);

  return len;
}

void flash_qspi_flush (void)
{
  VERIFY(_flash_dev,);

  xSemaphoreTake(_qspi_mutex, portMAX_DELAY);
  flash_cache_flush(&_cache);
  xSemaphoreGive(_qspi_mutex);
}

bool flash_qspi_chiperase (void)
{
  VERIFY(_flash_dev);

  xSemaphoreTake(_qspi_mutex, portMAX_DELAY);
  VERIFY(NRFX_SUCCESS == nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_ALL, 0));
  xSemaphoreGive(_qspi_mutex);

  return true;
}

void flash_qspi_init (void)
{
  // Init QSPI flash
  nrfx_qspi_config_t qspi_cfg = {
    .xip_offset = 0,
    .pins = {
      .sck_pin = PIN_QSPI_SCK,
      .csn_pin = PIN_QSPI_CS,
      .io0_pin = PIN_QSPI_DATA0,
      .io1_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io2_pin = NRF_QSPI_PIN_NOT_CONNECTED,
      .io3_pin = NRF_QSPI_PIN_NOT_CONNECTED,

    },
    .prot_if = {
      .readoc = NRF_QSPI_READOC_FASTREAD,
      .writeoc = NRF_QSPI_WRITEOC_PP,
      .addrmode = NRF_QSPI_ADDRMODE_24BIT,
      .dpmconfig = false
    },
    .phy_if = {
      .sck_freq = NRF_QSPI_FREQ_32MDIV1,
      .sck_delay = 1,    // min time CS must stay high before going low again. in unit of 62.5 ns
      .spi_mode = NRF_QSPI_MODE_0,
      .dpmen = false
    },
    .irq_priority = 7,
  };

#if QSPI_FLASH_MODE > 1
  qspi_cfg.pins.io1_pin = PIN_QSPI_DATA1;
  qspi_cfg.prot_if.readoc = NRF_QSPI_READOC_READ2IO;
  qspi_cfg.prot_if.writeoc = NRF_QSPI_WRITEOC_PP2O;
#if QSPI_FLASH_MODE > 2
  qspi_cfg.pins.io2_pin = PIN_QSPI_DATA2;
  qspi_cfg.pins.io3_pin = PIN_QSPI_DATA3;
  qspi_cfg.prot_if.readoc = NRF_QSPI_READOC_READ4IO;
  qspi_cfg.prot_if.writeoc = NRF_QSPI_WRITEOC_PP4IO;
#endif
#endif

  // No callback for blocking API
  nrfx_qspi_init(&qspi_cfg, NULL, NULL);

  nrf_qspi_cinstr_conf_t cinstr_cfg = {
    .opcode = 0,
    .length = 0,
    .io2_level = true,
    .io3_level = true,
    .wipwait = false,
    .wren = false
  };

  // Send reset enable
  cinstr_cfg.opcode = QSPI_CMD_RSTEN;
  cinstr_cfg.length = 1;
  nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);

  // Send reset command
  cinstr_cfg.opcode = QSPI_CMD_RST;
  cinstr_cfg.length = 1;
  nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);

  NRFX_DELAY_US(100);    // wait for flash device to reset

  // Send (Read ID + 3 dummy bytes) + Receive 2 bytes of (Manufacture + Device ID)
  uint8_t dummy[6] = { 0 };
  uint8_t id_resp[6] = { 0 };
  cinstr_cfg.opcode = QSPI_CMD_READID;
  cinstr_cfg.length = 6;

  // Bug with -nrf_qspi_cinstrdata_get() didn't combine data.
  // https://devzone.nordicsemi.com/f/nordic-q-a/38540/bug-nrf_qspi_cinstrdata_get-didn-t-collect-data-from-both-cinstrdat1-and-cinstrdat0
  nrfx_qspi_cinstr_xfer(&cinstr_cfg, dummy, id_resp);

  // Due to the bug, we collect data manually
  uint8_t dev_id = (uint8_t) NRF_QSPI->CINSTRDAT1;
  uint8_t mfgr_id = (uint8_t) ( NRF_QSPI->CINSTRDAT0 >> 24);

  // Look up the flash device in supported array
  for ( int i = 0; i < FLASH_DEVICE_COUNT; i++ )
  {
    // Match ID
    if ( _flash_devices_arr[i].manufacturer_id == mfgr_id && _flash_devices_arr[i].device_id == dev_id )
    {
      _flash_dev = &_flash_devices_arr[i];
      break;
    }
  }

  if ( _flash_dev )
  {
    // Enable quad mode if needed
#if QSPI_FLASH_MODE == 4
    cinstr_cfg.opcode = QSPI_CMD_WRSR;
    cinstr_cfg.length = 3;
    cinstr_cfg.wipwait = cinstr_cfg.wren = true;
    nrfx_qspi_cinstr_xfer(&cinstr_cfg, &_flash_dev->status_quad_enable, NULL);
#endif
  }

  // create mutex
  _qspi_mutex = xSemaphoreCreateMutex();
}

//--------------------------------------------------------------------+
// HAL for caching
//--------------------------------------------------------------------+
static bool fal_qspi_erase (uint32_t addr)
{
  VERIFY(_flash_dev);
  VERIFY(NRFX_SUCCESS == nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr));
  return true;
}

static uint32_t fal_qspi_program (uint32_t dst, void const * src, uint32_t len)
{
  VERIFY(_flash_dev, 0);
  VERIFY(NRFX_SUCCESS == nrfx_qspi_write(src, len, dst));
  return len;
}

static uint32_t fal_qspi_read (void* dst, uint32_t src, uint32_t len)
{
  VERIFY(_flash_dev, 0);
  VERIFY(NRFX_SUCCESS == nrfx_qspi_read(dst, len, src));
  return len;
}

static bool fal_qspi_verify (uint32_t addr, void const * buf, uint32_t len)
{
  return false;
}

#endif // valid pin
#endif // nrf52840

