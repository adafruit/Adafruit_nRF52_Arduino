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

#include "kernel.h"
#include "nffs/nffs.h"

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
