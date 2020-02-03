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

/* See SEGGER_Extensions.h for important overview of purpose of this file */

/*
 * This is a hack to include the original source file directly into this file.
 * This keeps the changes localized, while giving access to the static variables
 * and functions which are only available within that one compilation unit.
 *
 * Specifically, this hack was necessary because the varialble `_ActiveTerminal`,
 * which keeps track of the current terminal being written to, was marked
 * static in that file and not exposed by any getter function.  This made it
 * impossible to provide the functionality of `SEGGER_RTT_TerminalOutBuffer()`
 * function in a safe manner, as it needs to set a terminal, write buffered
 * data, and then ***RESTORE THE PRIOR ACTIVE TERMINAL***.
 *
 * That said, this is still the cleanest solution, given the requirement to
 * no modify the source SEGGER files.  This is because both the conditional
 * `#define` statements and other static functions would need to be duplicated
 * wholesale from that file, if implementing this in an entirely stand-alone
 * file.
 *
 * For example, the functions needed to avoid this hack use at least the
 * following static (and thus defined only within the compilation unit) items:
 * [ ] _aTerminalId
 * [ ] _ActiveTerminal
 * [ ] INIT() -- which is conditional and thus VERY DIFFERENT from SEGGER_RTT_Init()
 * [ ] _PostTerminalSwitch()
 * [ ] _WriteBlocking()
 *
 * Therefore, this hack remains the cleanest solution until the functions are
 * integrated into SEGGER's own distribution.
 */
#include "SEGGER_RTT.c.orig"


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

