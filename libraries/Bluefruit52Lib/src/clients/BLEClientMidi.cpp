/**************************************************************************/
/*!
    @file     BLEClientMidi.cpp
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

#include "bluefruit.h"

void blemidi_central_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

BLEClientMidi::BLEClientMidi(uint16_t fifo_depth)
  : BLEClientService(BLEMIDI_UUID_SERVICE), _txd(BLEMIDI_UUID_CHR_IO), //_rxd(blemidi_UUID_CHR_RXD),
    _rx_fifo(1, fifo_depth)
{
  _rx_cb = NULL;
}

bool BLEClientMidi::begin(void)
{
  _rx_fifo.begin();

  // Invoke base class begin()
  BLEClientService::begin();

  //_rxd.begin(this);
  _txd.begin(this);

  // set up notify callback
  _txd.setNotifyCallback(blemidi_central_notify_cb);

  return true;
}

bool BLEClientMidi::begin(int baudrate)
{
  (void) baudrate;
  return begin();
}

bool BLEClientMidi::enableTXD(void)
{
  return _txd.enableNotify();
}

bool BLEClientMidi::disableTXD(void)
{
  return _txd.disableNotify();
}

void BLEClientMidi::setRxCallback( rx_callback_t fp)
{
  _rx_cb = fp;
}

bool BLEClientMidi::discover(uint16_t conn_handle)
{
  // Call Base class discover
  VERIFY( BLEClientService::discover(conn_handle) );
  _conn_hdl = BLE_CONN_HANDLE_INVALID; // make as invalid until we found all chars

  // Discover TXD, RXD characteristics
  VERIFY( 1 == Bluefruit.Discovery.discoverCharacteristic(conn_handle, _txd) );

  _conn_hdl = conn_handle;
  return true;
}

void BLEClientMidi::disconnect(void)
{
  BLEClientService::disconnect();

  flush();
}

void blemidi_central_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEClientMidi& uart_svc = (BLEClientMidi&) chr->parentService();
  uart_svc._rx_fifo.write(data, len);

  // invoke callback
  if ( uart_svc._rx_cb ) uart_svc._rx_cb(uart_svc);
}

/*------------------------------------------------------------------*/
/* STREAM API
 *------------------------------------------------------------------*/
int BLEClientMidi::read (void)
{
  uint8_t ch;
  return read(&ch, 1) ? (int) ch : EOF;
}

int BLEClientMidi::read (uint8_t * buf, size_t size)
{
  return _rx_fifo.read(buf, size);
}

size_t BLEClientMidi::write (uint8_t b)
{
  return write(&b, 1);
}

size_t BLEClientMidi::write (const uint8_t *content, size_t len)
{
  // write without response
  return _txd.write(content, len);
}

int BLEClientMidi::available (void)
{
  return _rx_fifo.count();
}

int BLEClientMidi::peek (void)
{
  uint8_t ch;
  return _rx_fifo.peek(&ch) ? (int) ch : EOF;
}

void BLEClientMidi::flush (void)
{
  _rx_fifo.clear();
}
