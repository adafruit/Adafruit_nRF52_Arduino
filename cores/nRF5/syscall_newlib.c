/**************************************************************************/
/*!
    @file     syscall_newlib.c
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

#include <errno.h>
#include <sys/stat.h>
#include <malloc.h>

// defined in linker script
extern unsigned char __HeapBase[];
extern unsigned char __HeapLimit[];

static unsigned char *sbrk_heap_top = __HeapBase;

//volatile uint32_t first_sbrk = 0;
//volatile uint32_t last_sbrk = 0;

caddr_t _sbrk( int incr )
{
  unsigned char *prev_heap;

  if ( sbrk_heap_top + incr > __HeapLimit )
  {
    /* Out of dynamic memory heap space */
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  prev_heap = sbrk_heap_top;

//  if ( !first_sbrk) first_sbrk = sbrk_heap_top;
//  last_sbrk = sbrk_heap_top;

  sbrk_heap_top += incr;

  return (caddr_t) prev_heap;
}

