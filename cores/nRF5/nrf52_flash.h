/**************************************************************************/
/*!
    @file     nrf52_flash.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2017, Adafruit Industries (adafruit.com)
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
#ifndef NRF52_FLASH_H_
#define NRF52_FLASH_H_

#include <stdint.h>
#include <hal/hal_flash_int.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define NRF52K_FLASH_SECTOR_SZ	4096

// const struct hal_flash*  dev is required for nffs flash interface but not used
// by nrf52_flash, just simply pass NULL when invoked

int nrf52_flash_init(const struct hal_flash *dev);
int nrf52_flash_erase_sector(const struct hal_flash *dev, uint32_t sector_address);
int nrf52_flash_write(const struct hal_flash *dev, uint32_t address, const void *src, uint32_t num_bytes);
int nrf52k_flash_read(const struct hal_flash *dev, uint32_t address, void *dst, uint32_t num_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* NRF52_FLASH_H_ */
