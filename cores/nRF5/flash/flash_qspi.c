/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifdef NRF52840_XXAA

#include "Arduino.h"

#ifdef EXTERNAL_FLASH_DEVICES

#include "flash_devices.h"
#include "flash_qspi.h"
#include "flash_cache.h"
#include "nrfx_qspi.h"


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
enum
{
  QSPI_CMD_RSTEN = 0x66,
  QSPI_CMD_RST = 0x99,
  QSPI_CMD_WRSR = 0x01,
  QSPI_CMD_REMS = 0x90,
};

const qspi_flash_device_t _flash_devices_arr[] = { EXTERNAL_FLASH_DEVICES };
enum { FLASH_DEVICE_COUNT = arrcount(_flash_devices_arr) };

const qspi_flash_device_t* _flash_dev = NULL;

// Flash Abstraction Layer
static bool fal_qspi_erase (uint32_t addr);
static uint32_t fal_qspi_program (uint32_t dst, void const * src, uint32_t len);
static uint32_t fal_qspi_read (void* dst, uint32_t src, uint32_t len);
static bool fal_qspi_verify (uint32_t addr, void const * buf, uint32_t len);

static uint8_t _cache_buffer[FLASH_CACHE_SIZE] __attribute__((aligned(4)));

static flash_cache_t _cache = {
  .erase      = fal_qspi_erase,
  .program    = fal_qspi_program,
  .read       = fal_qspi_read,
  .verify     = fal_qspi_verify,
  .cache_addr = FLASH_CACHE_INVALID_ADDR,
  .cache_buf  = _cache_buffer
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
      .sck_pin = g_ADigitalPinMap[PIN_QSPI_SCK],
      .csn_pin = g_ADigitalPinMap[PIN_QSPI_CS],
      .io0_pin = g_ADigitalPinMap[PIN_QSPI_DATA0],
      .io1_pin = g_ADigitalPinMap[PIN_QSPI_DATA1],
      .io2_pin = g_ADigitalPinMap[PIN_QSPI_DATA2],
      .io3_pin = g_ADigitalPinMap[PIN_QSPI_DATA3],
    },
    .prot_if = {
      .readoc = NRF_QSPI_READOC_READ4IO,
      .writeoc = NRF_QSPI_WRITEOC_PP4O,
      .addrmode = NRF_QSPI_ADDRMODE_24BIT,
      .dpmconfig = false
    },
    .phy_if = {
      .sck_freq = NRF_QSPI_FREQ_32MDIV16, // start with low 2 Mhz speed
      .sck_delay = 10,    // min time CS must stay high before going low again. in unit of 62.5 ns
      .spi_mode = NRF_QSPI_MODE_0,
      .dpmen = false
    },
    .irq_priority = 7,
  };

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
  uint8_t dummy[5] = { 0 };
  uint8_t id_resp[5] = { 0 };
  cinstr_cfg.opcode = QSPI_CMD_REMS;
  cinstr_cfg.length = 6;

  nrfx_qspi_cinstr_xfer(&cinstr_cfg, dummy, id_resp);

  uint8_t const mfgr_id = id_resp[3];
  uint8_t const dev_id = id_resp[4];

  // Debug
//  printf("qspi mfgr id  : 0x%02X\n", mfgr_id);
//  printf("qspi device id: 0x%02X\n", dev_id);
//  PRINT_BUFFER(id_resp, sizeof(id_resp));

  // Look up the flash device in supported array
  for ( uint32_t i = 0; i < FLASH_DEVICE_COUNT; i++ )
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
    // Enable quad mode
    cinstr_cfg.opcode = QSPI_CMD_WRSR;
    cinstr_cfg.length = 3;
    cinstr_cfg.wipwait = cinstr_cfg.wren = true;
    nrfx_qspi_cinstr_xfer(&cinstr_cfg, &_flash_dev->status_quad_enable, NULL);

    // Speed up frequency
    nrf_qspi_phy_conf_t phy_if = {
      .sck_freq = _flash_dev->freq,
      .sck_delay = 10,
      .spi_mode = NRF_QSPI_MODE_0,
      .dpmen = false
    };
    nrf_qspi_ifconfig1_set(NRF_QSPI, &phy_if);

    // create mutex
    _qspi_mutex = xSemaphoreCreateMutex();
  }
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

#endif // external flash
#endif // nrf52840
