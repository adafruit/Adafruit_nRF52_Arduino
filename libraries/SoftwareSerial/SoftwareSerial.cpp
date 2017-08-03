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
 
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <variant.h>
#include <WInterrupts.h>

SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;


SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) :
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{   
  _receivePin = receivePin;
  _transmitPin = transmitPin;
}


SoftwareSerial::~SoftwareSerial()
{
 end();
}

void SoftwareSerial::begin(long speed)
 {  
    setTX(_transmitPin);
    setRX(_receivePin);
    // Precalculate the various delays
    //Calculate the distance between bit in micro seconds
    uint32_t bit_delay = (float(1)/speed)*1000000;
 
    _tx_delay = bit_delay;
  
    //Wait 1/2 bit - 2 micro seconds (time for interrupt to be served)
    _rx_delay_centering = (bit_delay/2) - 2;
    //Wait 1 bit - 2 micro seconds (time in each loop iteration)
    _rx_delay_intrabit = bit_delay - 1;//2
    //Wait 1 bit (the stop one) 
    _rx_delay_stopbit = bit_delay; 

       
      delayMicroseconds(_tx_delay);

      listen();
}

bool SoftwareSerial::listen()
{
  if (!_rx_delay_stopbit)
    return false;

  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    if(_inverse_logic)
        //Start bit high
       _intMask = attachInterrupt(_receivePin, handle_interrupt, RISING);
    else
        //Start bit low
        _intMask = attachInterrupt(_receivePin, handle_interrupt, FALLING);
        
    return true;
  }
 return false;
}

bool SoftwareSerial::stopListening()
{
   if (active_object == this)
   {
     detachInterrupt(_receivePin);
     active_object = NULL;
     return true;
   }
  return false;
}

void SoftwareSerial::end()
{
  stopListening();
}

int SoftwareSerial::read()
{
  if (!isListening()){
    return -1;}


  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail){
    return -1;}

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}  

int SoftwareSerial::available()
{
  if (!isListening())
    return 0;
  
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint32_t* reg = _transmitPortRegister;
  uint32_t reg_mask = _transmitBitMask;
  uint32_t inv_mask = ~_transmitBitMask;
  bool inv = _inverse_logic;
  uint16_t delay = _tx_delay;
  
  if (inv)
    b = ~b;
  // turn off interrupts for a clean txmit
   NRF_GPIOTE->INTENCLR = _intMask;
  // Write the start bit
  if (inv)
    *reg |= reg_mask;
  else
    *reg &= inv_mask;

  delayMicroseconds(delay);


  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      *reg |= reg_mask; // send 1
    else
      *reg &= inv_mask; // send 0

    delayMicroseconds(delay); 
    b >>= 1;
  }

  // restore pin to natural state
  if (inv)
    *reg &= inv_mask;
  else
    *reg |= reg_mask;
  
  NRF_GPIOTE->INTENSET = _intMask;
  
  delayMicroseconds(delay);  
  
  return 1;
}

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  NRF_GPIOTE->INTENCLR = _intMask;
  
  _receive_buffer_head = _receive_buffer_tail = 0;

  NRF_GPIOTE->INTENSET = _intMask;
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}


//private methods

void SoftwareSerial::recv()
{
  uint8_t d = 0;
   
  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {

    NRF_GPIOTE->INTENCLR = _intMask;
 
    // Wait approximately 1/2 of a bit width to "center" the sample
       delayMicroseconds(_rx_delay_centering);
   
    // Read each of the 8 bits
    for (uint8_t i=8; i > 0; --i)
    {
        
     delayMicroseconds(_rx_delay_intrabit);
	 // nRF52 needs another delay less than 1 uSec to be better synchronized
	 // with the highest baud rates
	 __ASM volatile (
       " NOP\n\t"
       " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	   " NOP\n"
	 );

      d >>= 1;

      if (rx_pin_read()){
        d |= 0x80;                  
       }
     
    }
    if (_inverse_logic){
      d = ~d;
    }
    
    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {
      _buffer_overflow = true;
    }

    // skip the stop bit
   delayMicroseconds(_rx_delay_stopbit); 

   NRF_GPIOTE->INTENSET = _intMask;  
  }
}

uint32_t SoftwareSerial::rx_pin_read()
{ 
  return *_receivePortRegister & digitalPinToBitMask(_receivePin);
}

/* static */
inline void SoftwareSerial::handle_interrupt()
{
   if (active_object)
   {
     active_object->recv();
   }
}

void SoftwareSerial::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  NRF_GPIO_Type * port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  NRF_GPIO_Type * port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}
