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

/* Based on SEGGER SystemView version: 3.10, there are a few items that have not
 * (yet) been integrated into SEGGER's files.  This header and the .c file
 * provide items.
 */

#ifndef EXTENSIONS_TO_SEGGER
#define EXTENSIONS_TO_SEGGER

#include "arduino.h"
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"

/* This is tightly coupled to the version of SEGGER SYSVIEW / RTT.
 * If updating SEGGER SYSVIEW / RTT, see comments in this header
 * and corresponding .c file.
 */
#if !defined(SEGGER_SYSVIEW_MAJOR) || (SEGGER_SYSVIEW_MAJOR !=  3) || \
    !defined(SEGGER_SYSVIEW_MINOR) || (SEGGER_SYSVIEW_MINOR != 10) || \
    !defined(SEGGER_SYSVIEW_REV  ) || (SEGGER_SYSVIEW_REV   !=  0)
    #error "SEGGER SYSVIEW revision mismatch -- requires review of this file and .C counterpart"
#endif

/* NOTE: SEGGER_RTT_Syscalls_GCC.c is *NOT* included in this depot.
 *       instead, those system calls are handled in ....
 */

/*
 * See SEGGER_RTT.c:
 *     `_aTerminalId[]` -- must match this static allocation size
 *     `SEGGER_RTT_SetTerminal()` -- matches limit of terminal count here also
 */
#define SEGGER_RTT_NUMBER_OF_TERMINALS   16
#define SEGGER_RTT_TERMINAL_OUT_OVERHEAD  4


#ifdef __cplusplus
  extern "C" {
#endif

    /*
    * See comments in SEGGER_Extensions.c
    */
    int SEGGER_RTT_Peek(void);

    /*
    * See comments in SEGGER_Extensions.c
    */
    int SEGGER_RTT_TerminalOutBuffer(unsigned char TerminalId, const void* pBuffer, size_t BufferSize);

#ifdef __cplusplus
  }
#endif


#endif /* EXTENSIONS_TO_SEGGER */
