/**************************************************************************/
/*!
    @file     nffs_port.c
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

#include <stdint.h>
#include <stdbool.h>

#include "kernel.h"
#include "nffs/nffs.h"
#include "common_inc.h"

#ifdef NRF52840_XXAA
#define NFFS_FLASH_ADDR_START     0xED000
#else
#define NFFS_FLASH_ADDR_START     0x6D000
#endif

#define NFFS_FLASH_SIZE           (7*4096)

static uint16_t crc16(const uint8_t *src, size_t len, uint16_t polynomial, uint16_t initial_value, bool pad);

/*
 * NFFS code keeps fs state in RAM but access to these structures is not
 * thread-safe - we need global lock for each fs operation to guarantee two
 * threads won't modify NFFS at the same time.
 */
static SemaphoreHandle_t _nffs_mutex = NULL;

nffs_os_mempool_t nffs_file_pool = sizeof(struct nffs_file);
nffs_os_mempool_t nffs_dir_pool = sizeof(struct nffs_dir);
nffs_os_mempool_t nffs_inode_entry_pool = sizeof(struct nffs_inode_entry);
nffs_os_mempool_t nffs_block_entry_pool = sizeof(struct nffs_hash_entry);
nffs_os_mempool_t nffs_cache_inode_pool = sizeof(struct nffs_cache_inode);
nffs_os_mempool_t nffs_cache_block_pool = sizeof(struct nffs_cache_block);

int nffs_os_mempool_init(void)
{
  return 0;
}

void* nffs_os_mempool_get(nffs_os_mempool_t *pool)
{
  return pvPortMalloc(*pool);
}

int nffs_os_mempool_free(nffs_os_mempool_t *pool, void *block)
{
  vPortFree(block);
  return 0;
}


int nffs_init(void)
{
  _nffs_mutex = xSemaphoreCreateMutex();

  nffs_misc_reset();

//	return fs_register(FS_NFFS, &nffs_fs);

  struct nffs_flash_desc flash_desc =
  {
    .id = 0,
    .sector_count = NRF_FICR->CODESIZE,
    .area_offset = NFFS_FLASH_ADDR_START,
    .area_size = NRF_FICR->CODEPAGESIZE*7
  };

  struct nffs_area_desc descs[NFFS_CONFIG_MAX_AREAS + 1];
	int cnt = NFFS_CONFIG_MAX_AREAS;

	VERIFY_NFFS( nffs_misc_desc_from_flash_area(&flash_desc, &cnt, descs) );

	/* Attempt to restore an existing nffs file system from flash. */
	int rc = nffs_restore_full(descs);

  // If not formatted, format it
  if (FS_ECORRUPT == rc)
  {
    LOG_LV1(NULL, "No FS detected, format");
    rc = nffs_format_full(descs);
  }

  VERIFY_NFFS( rc );

  return rc;
}

int nffs_os_flash_read(uint8_t id, uint32_t address, void *dst, uint32_t num_bytes)
{
  (void) id;
  return (num_bytes == flash_nrf52_read(dst, address, num_bytes)) ? 0 : -1;
}

int nffs_os_flash_write(uint8_t id, uint32_t address, const void *src, uint32_t num_bytes)
{
  (void) id;
  return (num_bytes == flash_nrf52_write(address, src, num_bytes)) ? 0 : -1;
}

int nffs_os_flash_erase(uint8_t id, uint32_t address, uint32_t num_bytes)
{
  (void) id;
  (void) num_bytes;

  return flash_nrf52_erase(address) ? 0 : -1;
}

int nffs_os_flash_info(uint8_t id, uint32_t sector, uint32_t *address, uint32_t *size)
{
  (void) id;

  *address = sector * NRF_FICR->CODEPAGESIZE;
  *size = NRF_FICR->CODEPAGESIZE;

  return 0;
}



uint16_t nffs_os_crc16_ccitt(uint16_t initial_crc, const void *buf, int len, int final)
{
	return crc16(buf, len, 0x1021, initial_crc, final);
}

/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

static uint16_t crc16(const uint8_t *src, size_t len, uint16_t polynomial, uint16_t initial_value, bool pad)
{
	uint16_t crc = initial_value;
	size_t padding = pad ? sizeof(crc) : 0;
	size_t i, b;

	/* src length + padding (if required) */
	for (i = 0; i < len + padding; i++) {

		for (b = 0; b < 8; b++) {
			uint16_t divide = crc & 0x8000;

			crc = (crc << 1);

			/* choose input bytes or implicit trailing zeros */
			if (i < len) {
				crc |= !!(src[i] & (0x80 >> b));
			}

			if (divide) {
				crc = crc ^ polynomial;
			}
		}
	}

	return crc;
}

