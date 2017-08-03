/*
  SoftwareSerial.cpp - library for Arduino Primo
  Copyright (c) 2016 Arduino. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 */
 
#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <variant.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size

class SoftwareSerial : public Stream
{
private:
  // per object data
  uint8_t _transmitPin;
  uint8_t _receivePin;
  uint32_t _receiveBitMask;
  volatile uint32_t* _receivePortRegister;
  uint32_t _transmitBitMask;
  volatile uint32_t* _transmitPortRegister;
  volatile uint32_t _intMask;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  void recv() __attribute__((__always_inline__));
  uint32_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state) __attribute__((__always_inline__));
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  void setRxIntMsk(bool enable) __attribute__((__always_inline__));


public:
  // public methods
  SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
  ~SoftwareSerial();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool() { return true; }
  
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};

#endif
