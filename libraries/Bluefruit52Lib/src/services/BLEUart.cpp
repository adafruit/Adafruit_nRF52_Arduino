/**************************************************************************/
/*!
    @file     BLEUart.cpp
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

#include "bluefruit.h"
#include "utility/TimeoutTimer.h"

/* UART Serivce: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * UART RXD    : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * UART TXD    : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */

const uint8_t BLEUART_UUID_SERVICE[] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

const uint8_t BLEUART_UUID_CHR_RXD[] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

const uint8_t BLEUART_UUID_CHR_TXD[] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

/**
 * Constructor
 */
BLEUart::BLEUart(uint16_t fifo_depth)
  : BLEService(BLEUART_UUID_SERVICE), _txd(BLEUART_UUID_CHR_TXD), _rxd(BLEUART_UUID_CHR_RXD),
    _rxd_fifo(1, fifo_depth)
{
  _rx_cb        = NULL;

  _buffered_txd = false;
  _buffered_th  = NULL;
}

/**
 * Callback when received new data
 * @param chr
 * @param data
 * @param len
 * @param offset
 */
void bleuart_rxd_cb(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  (void) offset;

  BLEUart& svc = (BLEUart&) chr.parentService();
  svc._rxd_fifo.write(data, len);

  // invoke user callback
  if ( svc._rx_cb ) svc._rx_cb();
}

/**
 * Timer callback periodically to send TX packet (if enabled)
 * @param timer
 */
void bleuart_txd_buffered_handler(TimerHandle_t timer)
{
//  BLEUart& svc = (BLEUart&) pvTimerGetTimerID(timer);
}

void bleuart_txd_cccd_cb(BLECharacteristic& chr, uint16_t value)
{
  BLEUart& svc = (BLEUart&) chr.parentService();

  if ( svc._buffered_th == NULL) return;

  // Enable TXD timer if configured
  if (value & BLE_GATT_HVX_NOTIFICATION)
  {
    xTimerStart(svc._buffered_th, 0);
  }else
  {
    xTimerStop(svc._buffered_th, 0);
  }
}

void BLEUart::setRxCallback( rx_callback_t fp)
{
  _rx_cb = fp;
}

/**
 * Set timeout for buffered TXD packet.
 * Note: packet is sent right away if it reach MTU bytes
 * @param ms
 */
void BLEUart::bufferTXD(bool enable)
{
  _buffered_txd = enable;

  // enable cccd callback to start timer when enabled
  _txd.setCccdWriteCallback( enable ? bleuart_txd_cccd_cb : NULL);
}

err_t BLEUart::begin(void)
{
  _rxd_fifo.begin();

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // Add TXD Characteristic
  _txd.setProperties(CHR_PROPS_NOTIFY);
//  _txd.setMaxLen(BLE_MAX_DATA_PER_MTU);
  // TODO enable encryption when bonding is enabled
  _txd.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _txd.setStringDescriptor("TXD");
  VERIFY_STATUS( _txd.begin() );

  // Add RXD Characteristic
  _rxd.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  _rxd.setWriteCallback(bleuart_rxd_cb);

  // TODO enable encryption when bonding is enabled
  _rxd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);

  _rxd.setStringDescriptor("RXD");
  VERIFY_STATUS(_rxd.begin());

  return ERROR_NONE;
}

bool BLEUart::notifyEnabled(void)
{
  return _txd.notifyEnabled();
}

void BLEUart::_disconnect_cb(void)
{
  if (_buffered_th)
  {
    xTimerDelete(_buffered_th, 0);
    _buffered_th = NULL;
  }
}

void BLEUart::_connect_cb (void)
{
  if ( _buffered_txd)
  {
    // create buffered timer with interval = connection interval
    _buffered_th = xTimerCreate(NULL, (5*ms2tick(Bluefruit.connInterval())) / 4, true, this, bleuart_txd_buffered_handler);
  }
}

/*------------------------------------------------------------------*/
/* STREAM API
 *------------------------------------------------------------------*/
int BLEUart::read (void)
{
  uint8_t ch;
  return read(&ch, 1) ? (int) ch : EOF;
}

int BLEUart::read (uint8_t * buf, size_t size)
{
  return _rxd_fifo.read(buf, size);
}

size_t BLEUart::write (uint8_t b)
{
  return write(&b, 1);
}

size_t BLEUart::write (const uint8_t *content, size_t len)
{
  return _txd.notify(content, len) ? len : 0;
}

int BLEUart::available (void)
{
  return _rxd_fifo.count();
}

int BLEUart::peek (void)
{
  uint8_t ch;
  return _rxd_fifo.peek(&ch) ? (int) ch : EOF;
}

void BLEUart::flush (void)
{
  _rxd_fifo.clear();
}




