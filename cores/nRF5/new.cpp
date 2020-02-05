/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdlib.h>
#include "rtos.h"

#include <errno.h>
#include <sys/stat.h>
#include <malloc.h>

void *operator new(size_t size) {
  return rtos_malloc(size);
}

void *operator new[](size_t size) {
  return rtos_malloc(size);
}

void operator delete(void * ptr) {
  rtos_free(ptr);
}

void operator delete[](void * ptr) {
  rtos_free(ptr);
}

void operator delete(void * ptr, unsigned int) {
  rtos_free(ptr);
}

void operator delete[](void * ptr, unsigned int) {
  rtos_free(ptr);
}


extern "C"
{

// defined in linker script
extern unsigned char __HeapBase[];
extern unsigned char __HeapLimit[];

static unsigned char *sbrk_heap_top = __HeapBase;

__attribute__((used))
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

  sbrk_heap_top += incr;

  return (caddr_t) prev_heap;
}

void __malloc_lock(struct _reent *ptr)
{
  (void) ptr;
  vTaskSuspendAll();
}

void __malloc_unlock(struct _reent *ptr)
{
  (void) ptr;
  xTaskResumeAll();
}

}
