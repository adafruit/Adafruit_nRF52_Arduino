/**************************************************************************/
/*!
    @file     common_func.h
    @author   hathach

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2016, Adafruit Industries (adafruit.com)
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

#ifndef _COMMON_FUNC_H_
#define _COMMON_FUNC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define COMMENT_OUT(x)

#define memclr(buffer, size)  memset(buffer, 0, size)
#define varclr(_var)          memclr(_var, sizeof(*(_var)))
#define arrcount(_arr)       ( sizeof(_arr) / sizeof(_arr[0]) )

#define __swap32(x)    __REV(x)                   ///< built-in function to swap Endian of 32-bit number
#define __swap16(u16)  ((uint16_t) __REV16(u16))  ///< built-in function to swap Endian of 16-bit number

#define maxof(a,b) \
    ({ typeof (a) _a = (a); \
       typeof (b) _b = (b); \
       _a > _b ? _a : _b; })

#define minof(a,b) \
    ({ typeof (a) _a = (a); \
       typeof (b) _b = (b); \
       _a < _b ? _a : _b; })

//--------------------------------------------------------------------+
// MACROS
//--------------------------------------------------------------------+
#define U16_HIGH(u16)              ((uint8_t) (((u16) >> 8) & 0x00ff))
#define U16_LOW(u16)               ((uint8_t) ((u16)       & 0x00ff))
#define U16_BYTES_BE(u16)           U16_HIGH(u16), U16_LOW(u16)
#define U16_BYTES_LE(u16)           U16_LOW(u16), U16_HIGH(u16)

#define U32_BYTE1(u32)              ((uint8_t) (((u32) >> 24) & 0x000000ff)) // MSB
#define U32_BYTE2(u32)              ((uint8_t) (((u32) >> 16) & 0x000000ff))
#define U32_BYTE3(u32)              ((uint8_t) (((u32) >>  8) & 0x000000ff))
#define U32_BYTE4(u32)              ((uint8_t) ((u32)         & 0x000000ff)) // LSB

#define U32_FROM_U8(b1, b2, b3, b4) ((uint32_t) (((b1) << 24) + ((b2) << 16) + ((b3) << 8) + (b4)))
#define U32_FROM_U16(high, low)     ((uint32_t) (((high) << 16) | (low)))
#define U16_FROM_U8(high, low)      ((uint32_t) (((high) << 8) | (low)))

#define U32_BYTES_BE(u32)            U32_BYTE1(u32), U32_BYTE2(u32), U32_BYTE3(u32), U32_BYTE4(u32)
#define U32_BYTES_LE(u32)            U32_BYTE4(u32), U32_BYTE3(u32), U32_BYTE2(u32), U32_BYTE1(u32)

#define SWAP16(x)                   ((uint16_t)(((x) << 8) | (((x) & 0xff00) >> 8)))

//--------------------------------------------------------------------+
// DEBUG HELPER
//--------------------------------------------------------------------+
#if CFG_DEBUG == 2
  #define malloc_named( name, size )            ({ printf("[malloc] %s : %d\r\n", name, size); malloc(size); })
#else
  #define malloc_named( name, size )            malloc ( size )
#endif

int cprintf(const char * format, ...);

#if CFG_DEBUG

#define PRINT_LOCATION()      cprintf("%s: %d:\n", __PRETTY_FUNCTION__, __LINE__)
#define PRINT_MESS(x)         cprintf("%s: %d: %s \n"   , __PRETTY_FUNCTION__, __LINE__, (char*)(x))
#define PRTNT_HEAP()          if (CFG_DEBUG == 3) cprintf("\n%s: %d: Heap free: %d\n", __PRETTY_FUNCTION__, __LINE__, util_heap_get_free_size())
#define PRINT_STR(x)          cprintf("%s: %d: " #x " = %s\n"   , __PRETTY_FUNCTION__, __LINE__, (char*)(x) )
#define PRINT_INT(x)          cprintf("%s: %d: " #x " = %ld\n"  , __PRETTY_FUNCTION__, __LINE__, (uint32_t) (x) )

#define PRINT_HEX(x) \
  do {\
    cprintf("%s: %d: " #x " = 0x", __PRETTY_FUNCTION__, __LINE__);\
    char fmt[] = "%00X\n";\
    fmt[2] += 2*sizeof(x); /* Hex with correct size */\
    cprintf(fmt, (x) );\
  }while(0)

#define PRINT_BUFFER(buf, n) \
  do {\
    uint8_t const* p8 = (uint8_t const*) (buf);\
    Serial.print(#buf ": ");\
    for(uint32_t i=0; i<(n); i++) cprintf("%02x ", p8[i]);\
    Serial.println();\
  }while(0)

#else

#define PRINT_LOCATION()
#define PRINT_MESS(x)
#define PRTNT_HEAP()
#define PRINT_STR(x)
#define PRINT_INT(x)
#define PRINT_HEX(x)
#define PRINT_BUFFER(buf, n)

#endif

//--------------------------------------------------------------------+
// INLINE FUNCTION
//--------------------------------------------------------------------+
/// Checks is all values in the supplied array are zero
static inline bool mem_test_zero(void const* buffer, uint32_t size)
{
  uint8_t const* p_mem = (uint8_t const*) buffer;
  for(uint32_t i=0; i<size; i++) if (p_mem[i] != 0)  return false;
  return true;
}

//------------- Min, Max -------------//
static inline uint8_t  min8 (uint8_t  x, uint8_t  y) { return (x < y) ? x : y; }
static inline uint16_t min16(uint16_t x, uint16_t y) { return (x < y) ? x : y; }
static inline uint32_t min32(uint32_t x, uint32_t y) { return (x < y) ? x : y; }

static inline uint8_t  max8 (uint8_t  x, uint8_t  y) { return (x > y) ? x : y; }
static inline uint16_t max16(uint16_t x, uint16_t y) { return (x > y) ? x : y; }
static inline uint32_t max32(uint32_t x, uint32_t y) { return (x > y) ? x : y; }

//------------- Conversion -------------//
/// form an uint32_t from 4 x uint8_t
static inline uint32_t u32_from_u8(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
  return (b1 << 24) + (b2 << 16) + (b3 << 8) + b4;
}

static inline uint16_t u32_high_u16(uint32_t u32)
{
  return (uint16_t) ((u32 >> 16) & 0xffff);
}

static inline uint16_t u32_low_u16(uint32_t u32)
{
  return (uint16_t) (u32 & 0xffff);
}

static inline uint16_t u16_from_u8(uint8_t b1, uint8_t b2)
{
  return (b1 << 8) + b2;
}

static inline uint8_t u16_high_u8(uint16_t u16)
{
  return (uint8_t) ((u16 >> 8) & 0x00ff);
}

static inline uint8_t u16_low_u8(uint16_t u16)
{
  return (uint8_t) (u16 & 0x00ff);
}

//------------- Align -------------//
static inline uint32_t align32 (uint32_t value)
{
  return (value & 0xFFFFFFE0UL);
}

static inline uint32_t align16 (uint32_t value)
{
  return (value & 0xFFFFFFF0UL);
}

static inline uint32_t align4 (uint32_t value)
{
  return (value & 0xFFFFFFFCUL);
}

static inline uint32_t align_n (uint32_t alignment, uint32_t value)
{
  return value & (~(alignment-1));
}

static inline uint32_t align4k (uint32_t value)
{
  return (value & 0xFFFFF000UL);
}

static inline uint32_t offset4k(uint32_t value)
{
  return (value & 0xFFFUL);
}

//------------- Mathematics -------------//
static inline bool is_within(uint32_t lower, uint32_t value, uint32_t upper)
{
  return (lower <= value) && (value <= upper);
}

static inline uint8_t log2_of(uint32_t value)
{
  uint8_t result = 0; // log2 of a value is its MSB's position
  while (value >>= 1) result++;

  return result;
}

// return the number of set bits in value
static inline uint8_t cardinality_of(uint32_t value)
{
  // Brian Kernighan's method goes through as many iterations as there are set bits. So if we have a 32-bit word with only
  // the high bit set, then it will only go once through the loop
  // Published in 1988, the C Programming Language 2nd Ed. (by Brian W. Kernighan and Dennis M. Ritchie)
  // mentions this in exercise 2-9. On April 19, 2006 Don Knuth pointed out to me that this method
  // "was first published by Peter Wegner in CACM 3 (1960), 322. (Also discovered independently by Derrick Lehmer and
  // published in 1964 in a book edited by Beckenbach.)"
  uint8_t count;
  for (count = 0; value; count++)
  {
    value &= value - 1; // clear the least significant bit set
  }

  return count;
}

#ifdef __cplusplus
 }
#endif

#endif /* _COMMON_FUNC_H_ */
