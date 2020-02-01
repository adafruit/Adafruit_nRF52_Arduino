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

#include <SeggerRTTSerial.h>
#include <SEGGER_Extensions.h>


size_t Segger_RTT_Serial_t::write(uint8_t b) {
    return SEGGER_RTT_PutChar(0, (char)b);
}
size_t Segger_RTT_Serial_t::write(const uint8_t *buffer, size_t size) {
    return SEGGER_RTT_Write(0, buffer, size);
}
int Segger_RTT_Serial_t::availableForWrite() {
    return SEGGER_RTT_GetAvailWriteSpace(0);
}
int Segger_RTT_Serial_t::available() { // to read
    return SEGGER_RTT_HasData(0);
}
int Segger_RTT_Serial_t::read() {
    return SEGGER_RTT_GetKey();
}
int Segger_RTT_Serial_t::peek() {
    return -1; // return SEGGER_RTT_Peek();
}
void Segger_RTT_Serial_t::flush() {
    // no-op -- cannot flush as cannot control host
}
// Baud and config are ignored for RTT
void Segger_RTT_Serial_t::begin (uint32_t baud)
{
    (void) baud;
}
void Segger_RTT_Serial_t::begin (uint32_t baud, uint8_t config)
{
    (void) baud;
    (void) config;
}
void Segger_RTT_Serial_t::end(void)
{
    // nothing to do
}
Segger_RTT_Serial_t::operator bool()
{
    return true;
}



