/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <assert.h>

#include "nrf.h"
//#include "mcu/nrf52_hal.h"

#include <hal/hal_flash_int.h>
#include "hal/hal_flash.h"
#include "rtos.h"

#include "common_func.h"
#include "verify.h"
#include "nrf52_flash.h"

int hal_flash_sector_info(int idx, uint32_t *address, uint32_t *sz);

static const struct hal_flash_funcs nrf52k_flash_funcs = {
//    .hff_read         = hal_flash_read,
//    .hff_write        = hal_flash_write,
//    .hff_erase_sector = hal_flash_erase_sector,
    .hff_sector_info  = hal_flash_sector_info,
//    .hff_init         = hal_flash_init
};


const struct hal_flash nrf52k_flash_dev = {
    .hf_itf        = &nrf52k_flash_funcs,
    .hf_base_addr  = 0x00000000,
    .hf_size       = 512 * 1024,	/* XXX read from factory info? */
    .hf_sector_cnt = 128,	        /* XXX read from factory info? */
    .hf_align      = 1
};

uint8_t hal_flash_align(uint8_t flash_id)
{
  return nrf52k_flash_dev.hf_align;
}

static int hal_flash_check_addr(const struct hal_flash *hf, uint32_t addr)
{
    if (addr < hf->hf_base_addr || addr > hf->hf_base_addr + hf->hf_size) {
        return -1;
    }
    return 0;
}

const struct hal_flash *hal_bsp_flash_dev(uint8_t flash_id)
{
  return &nrf52k_flash_dev;
}

int hal_flash_read(uint8_t id, uint32_t address, void *dst, uint32_t num_bytes)
{
    (void) id;
    memcpy(dst, (void *)address, num_bytes);
    return 0;
}

int hal_flash_write(uint8_t id, uint32_t address, const void *src, uint32_t num_bytes)
{
  (void) id;

  return nrf52_flash_write(address, src, num_bytes);
}

int hal_flash_erase_sector(uint8_t id, uint32_t sector_address)
{
  (void) id;
  return nrf52_flash_erase_sector(sector_address);
}

int hal_flash_erase(uint8_t id, uint32_t address, uint32_t num_bytes)
{
    const struct hal_flash *hf;
    uint32_t start, size;
    uint32_t end;
    uint32_t end_area;
    int i;
    int rc;

    hf = &nrf52k_flash_dev;

    if (hal_flash_check_addr(hf, address) ||
      hal_flash_check_addr(hf, address + num_bytes)) {
        return -1;
    }

    end = address + num_bytes;
    if (end <= address) {
        /*
         * Check for wrap-around.
         */
        return -1;
    }

    for (i = 0; i < hf->hf_sector_cnt; i++) {
        rc = hal_flash_sector_info(i, &start, &size);
        assert(rc == 0);
        end_area = start + size;
        if (address < end_area && end > start) {
            /*
             * If some region of eraseable area falls inside sector,
             * erase the sector.
             */
            if (hal_flash_erase_sector(id, start)) {
                return -1;
            }
        }
    }
    return 0;
}

int hal_flash_sector_info(int idx, uint32_t *address, uint32_t *sz)
{
  assert(idx < nrf52k_flash_dev.hf_sector_cnt);
  *address = idx * NRF52K_FLASH_SECTOR_SZ;
  *sz = NRF52K_FLASH_SECTOR_SZ;
  return 0;
}
