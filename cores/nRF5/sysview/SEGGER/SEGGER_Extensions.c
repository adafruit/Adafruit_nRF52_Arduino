/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 by SimpleHacks, Henry Gabryjelski
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "SEGGER_Extensions.h"
#include <string.h>                 // for memcpy

/* See SEGGER_Extensions.h for important overview of purpose of this file */

#ifndef   SEGGER_RTT_MEMCPY
  #ifdef  MEMCPY
    #define SEGGER_RTT_MEMCPY(pDest, pSrc, NumBytes)      MEMCPY((pDest), (pSrc), (NumBytes))
  #else
    #define SEGGER_RTT_MEMCPY(pDest, pSrc, NumBytes)      memcpy((pDest), (pSrc), (NumBytes))
  #endif
#endif

/*
 * This is a hack to essentially call the `INIT()` macro
 * that is defined in SEGGER_RTT.c (as a static function).
 * 
 * Cannot simply create a wholesale copy of the existing
 * static function in SEGGER_RTT.c, because it assigns
 * compilation-unit-local values to the structure.
 * 
 * The use of `SEGGER_RTT_Init()` is also not advised,
 * as that does a forced-initialization, even if the
 * structures were already initialized.
 * 
 * There are two functions that could be used:
 * 
 * `SEGGE%R_RTT_HasKey()`
 * -- lock-free, unconditionally calls INIT()
 * -- Short, but it does access shared memory
 * 
 * `SEGGER_RTT_SetTerminal()`
 * -- lock-free, unconditionally calls INIT()
 * -- If parameter `TerminalId` is set to an invalid value,
 *    this never accesses any shared memory (outside init)
 * 
 * This hack is required to meet the goal of not
 * modifying the SEGGER-provided SysView files.
 */
/*********************************************************************
 *
 *       _DoInit()
 *
 *  Function description
 *    Initializes the control block an buffers.
 *    May only be called via INIT() to avoid overriding settings.
 *
 */
#define INIT() do {\
    (void)SEGGER_RTT_SetTerminal(0xFFu); \
    } while (0)

/*
 * This is a verbatim copy of the static function defined
 * in SEGGER_RTT.c.
 * 
 * This hack is required to meet the goal of not
 * modifying the SEGGER-provided SysView files.
 */
/*********************************************************************
 *
 *       _WriteBlocking()
 *
 *  Function description
 *    Stores a specified number of characters in SEGGER RTT ring buffer
 *    and updates the associated write pointer which is periodically
 *    read by the host.
 *    The caller is responsible for managing the write chunk sizes as
 *    _WriteBlocking() will block until all data has been posted successfully.
 *
 *  Parameters
 *    pRing        Ring buffer to post to.
 *    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
 *    NumBytes     Number of bytes to be stored in the SEGGER RTT control block.
 * 
 *  Return value
 *    >= 0 - Number of bytes written into buffer.
 */
static unsigned _WriteBlocking(SEGGER_RTT_BUFFER_UP* pRing, const char* pBuffer, unsigned NumBytes) {
  unsigned NumBytesToWrite;
  unsigned NumBytesWritten;
  unsigned RdOff;
  unsigned WrOff;
#if SEGGER_RTT_MEMCPY_USE_BYTELOOP
  char*    pDst;
#endif
  //
  // Write data to buffer and handle wrap-around if necessary
  //
  NumBytesWritten = 0u;
  WrOff = pRing->WrOff;
  do {
    RdOff = pRing->RdOff;                         // May be changed by host (debug probe) in the meantime
    if (RdOff > WrOff) {
      NumBytesToWrite = RdOff - WrOff - 1u;
    } else {
      NumBytesToWrite = pRing->SizeOfBuffer - (WrOff - RdOff + 1u);
    }
    NumBytesToWrite = min(NumBytesToWrite, (pRing->SizeOfBuffer - WrOff));      // Number of bytes that can be written until buffer wrap-around
    NumBytesToWrite = min(NumBytesToWrite, NumBytes);
#if SEGGER_RTT_MEMCPY_USE_BYTELOOP
    pDst = pRing->pBuffer + WrOff;
    NumBytesWritten += NumBytesToWrite;
    NumBytes        -= NumBytesToWrite;
    WrOff           += NumBytesToWrite;
    while (NumBytesToWrite--) {
      *pDst++ = *pBuffer++;
    };
#else
    SEGGER_RTT_MEMCPY(pRing->pBuffer + WrOff, pBuffer, NumBytesToWrite);
    NumBytesWritten += NumBytesToWrite;
    pBuffer         += NumBytesToWrite;
    NumBytes        -= NumBytesToWrite;
    WrOff           += NumBytesToWrite;
#endif
    if (WrOff == pRing->SizeOfBuffer) {
      WrOff = 0u;
    }
    pRing->WrOff = WrOff;
  } while (NumBytes);
  //
  return NumBytesWritten;
}
/*
 * This is a verbatim copy of the static function defined
 * in SEGGER_RTT.c.
 * 
 * This hack is required to meet the goal of not
 * modifying the SEGGER-provided SysView files.
 */
static unsigned char _aTerminalId[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
/*********************************************************************
 *
 *       _PostTerminalSwitch()
 *
 *  Function description
 *    Switch terminal to the given terminal ID.  It is the caller's
 *    responsibility to ensure the terminal ID is correct and there is
 *    enough space in the buffer for this to complete successfully.
 *
 *  Parameters
 *    pRing        Ring buffer to post to.
 *    TerminalId   Terminal ID to switch to.
 */
static void _PostTerminalSwitch(SEGGER_RTT_BUFFER_UP* pRing, unsigned char TerminalId) {
  unsigned char ac[2];

  ac[0] = 0xFFu;
  ac[1] = _aTerminalId[TerminalId];  // Caller made already sure that TerminalId does not exceed our terminal limit
  _WriteBlocking(pRing, (const char*)ac, 2u);
}

/* THERE IS NO EASY HACK TO MAKE THIS WORK.
//static unsigned char _ActiveTerminal; // BUGBUG -- NEED TO EXPOSE FROM SEGGER_RTT.C
*/

/*********************************************************************
 * New function, based substantially on `SEGGER_RTT_TerminalOut()`.
 * 
 * In fact, if wanting to modify the segger sources directly, that
 * function could be changed to simply call this one:
 *
 *       int SEGGER_RTT_TerminalOut (unsigned char TerminalId, const char* s) {
 *         return SEGGER_RTT_TerminalOutBuffer(TerminalId, s, STRLEN(s));
 *       }
 *
 * The purpose of this function is to allow partial output of
 * client-provided buffers (non-null-terminated), without allocating
 * intermediate memory.   Partial output is neccessary to safely
 * outputting to specific terminals.
 */

/*********************************************************************
 *
 *       SEGGER_RTT_TerminalOutBuffer
 *
 *  Function description
 *    Writes a string to the given terminal
 *     without changing the terminal for channel 0.
 *
 *  Parameters
 *    TerminalId   Index of the terminal.
 *    buffer       Array of bytes to be printed to the terminal.
 *    fragLen      The count of bytes to be printed to the terminal.
 *
 *  Return value
 *    >= 0 - Number of bytes written.
 *     < 0 - Error.
 *
 */
int SEGGER_RTT_TerminalOutBuffer (unsigned char TerminalId, const void* pBuffer, size_t BufferSize) {
  int                   Status;
  unsigned              Avail;
  SEGGER_RTT_BUFFER_UP* pRing;
  //
  INIT();
  //
  // Validate terminal ID.
  //
  if (TerminalId < SEGGER_RTT_NUMBER_OF_TERMINALS) { // We only support a certain number of channels
    //
    // Get "to-host" ring buffer.
    //
    pRing = &_SEGGER_RTT.aUp[0];

    //
    // How we output depends upon the mode...
    //
    SEGGER_RTT_LOCK();

    Avail = SEGGER_RTT_GetAvailWriteSpace(0);
    switch (pRing->Flags & SEGGER_RTT_MODE_MASK) {
    case SEGGER_RTT_MODE_NO_BLOCK_SKIP:
      //
      // If we are in skip mode and there is no space for the whole
      // of this output, don't bother switching terminals at all.
      //
      if (Avail < (BufferSize + SEGGER_RTT_TERMINAL_OUT_OVERHEAD)) {
        Status = 0;
      } else {
        _PostTerminalSwitch(pRing, TerminalId);
        Status = (int)_WriteBlocking(pRing, pBuffer, BufferSize);
        
        // IMPOSSIBLE TO FIX WITHOUT MODIFICATION TO SEGGER_RTT.c
        // THE VARIABLE _ActiveTerminal is not exposed, and it
        // is not 
        _PostTerminalSwitch(pRing, _ActiveTerminal);
      }
      break;
    case SEGGER_RTT_MODE_NO_BLOCK_TRIM:
      //
      // If we are in trim mode and there is not enough space for everything,
      // trim the output but always include the terminal switch.  If no room
      // for terminal switch, skip that totally.
      //
      if (Avail < SEGGER_RTT_TERMINAL_OUT_OVERHEAD) {
        Status = -1;
      } else {
        _PostTerminalSwitch(pRing, TerminalId);
        Status = (int)_WriteBlocking(pRing, pBuffer, (BufferSize < (Avail - 4u)) ? BufferSize : (Avail - 4u));
        _PostTerminalSwitch(pRing, _ActiveTerminal);
      }
      break;
    case SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL:
      //
      // If we are in blocking mode, output everything.
      //
      _PostTerminalSwitch(pRing, TerminalId);
      Status = (int)_WriteBlocking(pRing, pBuffer, BufferSize);
      _PostTerminalSwitch(pRing, _ActiveTerminal);
      break;
    default:
      Status = -1;
      break;
    }
    //
    // Finish up.
    //
    SEGGER_RTT_UNLOCK();
  } else {
    Status = -1;
  }
  return Status;
}

/*
 * New function, based substantially on `SEGGER_RTT_ReadNoLock()`.
 * `SEGGER_RTT_Read()` simply takes lock around `SEGGER_RTT_ReadNoLock()`.
 * Thus, this function takes / releases the lock, and internally simply
 * returns the first character of the buffer, without changing any other
 * state.
 * 
 * The purpose of this function is to enable `Stream->Peek()`, and
 * thus provide a drop-in replacement for `Serial`.
 * 
 * A request for inclusion of this function into official RTT source
 * files has been made:
 * https://forum.segger.com/index.php/Thread/6855-SOLVED-Feature-Request-RTT-SEGGER-RTT-Peek/
 */

/*********************************************************************
 *
 *       SEGGER_RTT_Peek
 *
 *  Function description
 *    Reads one character from the SEGGER RTT buffer.
 *    Host has previously stored data there.
 *
 *  Return value
 *    <  0 -   No character available (buffer empty).
 *    >= 0 -   Character which is next in the buffer.
 *
 *  Notes
 *    (1) This function is only specified for accesses to RTT buffer 0.
 */
int SEGGER_RTT_Peek(void) {
  char      c;
  int       r;
  unsigned  RdOff;
  unsigned  WrOff;
  SEGGER_RTT_BUFFER_DOWN* pRing;

  SEGGER_RTT_LOCK();
  INIT();
  pRing = &_SEGGER_RTT.aDown[0]; // Note (1)  This function is only specified for accesses to RTT buffer 0.
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;

  /* WrOff == RdOff: Buffer is empty */
  if (WrOff == RdOff) { // Buffer is empty when these are equal
    r = -1;
  } else { // only reading a single byte, so code can be very simple
    c = *(pRing->pBuffer + RdOff);
    r = (int)(unsigned char)c;
  }
  SEGGER_RTT_UNLOCK();
  return r;
}

