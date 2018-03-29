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
  : BLEService(BLEUART_UUID_SERVICE), _txd(BLEUART_UUID_CHR_TXD), _rxd(BLEUART_UUID_CHR_RXD)
{
  _rx_fifo       = NULL;
  _rx_cb         = NULL;
  _rx_fifo_depth = fifo_depth;

  _tx_fifo       = NULL;
  _tx_fifo_depth = 0;
  _tx_buffered   = 0;
  _buffered_th   = NULL;
}

/**
 * Destructor
 */
BLEUart::~BLEUart()
{
  if ( _tx_fifo ) delete _tx_fifo;
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
  svc._rx_fifo->write(data, len);

#if CFG_DEBUG >= 2
  LOG_LV2("BLEUART", "RX: ");
  PRINT_BUFFER(data, len);
#endif

  // invoke user callback
  if ( svc._rx_cb ) svc._rx_cb();
}

/**
 * Timer callback periodically to send TX packet (if enabled).
 * @param timer
 */
void bleuart_txd_buffered_hdlr(TimerHandle_t timer)
{
  BLEUart* svc = (BLEUart*) pvTimerGetTimerID(timer);

  // skip if null (unlikely)
  if ( !svc->_tx_fifo ) return;

  // flush tx data
  (void) svc->flush_tx_buffered();
}

void bleuart_txd_cccd_cb(BLECharacteristic& chr, uint16_t value)
{
  BLEUart& svc = (BLEUart&) chr.parentService();

  if ( svc._buffered_th == NULL) return;

  // Enable TXD timer if configured
  if (value & BLE_GATT_HVX_NOTIFICATION)
  {
    xTimerStart(svc._buffered_th, 0); // if started --> timer got reset
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
 * Enable packet buffered for TXD
 * Note: packet is sent right away if it reach MTU bytes
 * @param enable true or false
 */
void BLEUart::bufferTXD(uint8_t enable, uint16_t fifo_depth)
{
  _tx_buffered = enable;
  _tx_fifo_depth = fifo_depth;

  if ( enable )
  {
    // enable cccd callback to start timer when enabled
    _txd.setCccdWriteCallback(bleuart_txd_cccd_cb);

    // Create FIFO for TX
    if ( _tx_fifo == NULL )
    {
      _tx_fifo = new Adafruit_FIFO(1);
      _tx_fifo->begin(_tx_fifo_depth);
    }
  }else
  {
    _txd.setCccdWriteCallback(NULL);

    if ( _tx_fifo ) delete _tx_fifo;
    // set TX FIFO pointer to NULL after delete
    _tx_fifo = NULL;
  }
}

err_t BLEUart::begin(void)
{
  _rx_fifo = new Adafruit_FIFO(1);
  _rx_fifo->begin(_rx_fifo_depth);

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  uint16_t max_mtu = Bluefruit.Gap.getMaxMtuByConnCfg(CONN_CFG_PERIPHERAL);

  // Add TXD Characteristic
  _txd.setProperties(CHR_PROPS_NOTIFY);
  // TODO enable encryption when bonding is enabled
  _txd.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _txd.setMaxLen( max_mtu );
  _txd.setUserDescriptor("TXD");
  VERIFY_STATUS( _txd.begin() );

  // Add RXD Characteristic
  _rxd.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  _rxd.setWriteCallback(bleuart_rxd_cb);

  // TODO enable encryption when bonding is enabled
  _rxd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  _rxd.setMaxLen( max_mtu );
  _rxd.setUserDescriptor("RXD");
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

    if (_tx_fifo) _tx_fifo->clear();
  }
}

void BLEUart::_connect_cb (void)
{
  if ( _tx_buffered )
  {
    // create TXD timer TODO take connInterval into account
    // ((5*ms2tick(Bluefruit.connInterval())) / 4) / 2
    _buffered_th = xTimerCreate(NULL, ms2tick(10), true, this, bleuart_txd_buffered_hdlr);

    // Start the timer
    xTimerStart(_buffered_th, 0);
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
  return _rx_fifo->read(buf, size);
}

size_t BLEUart::write (uint8_t b)
{
  return write(&b, 1);
}

size_t BLEUart::write (const uint8_t *content, size_t len)
{
  // notify right away if txd buffered is not enabled
  if ( !(_tx_buffered && _tx_fifo) )
  {
    size_t datatosend =  len;
    size_t capacity = (Bluefruit.Gap.getMTU( Bluefruit.connHandle() ) - 3);
    // check to make sure we can send as much data as the caller has requested
    // if not, adjust the amount sent, and return to caller how much actually got sent
    if(datatosend > capacity)
    {
       datatosend = capacity;
    }

    return _txd.notify(content, datatosend) ? datatosend : 0;
  }else
  {
    // skip if not enabled
    if ( !notifyEnabled() ) return 0;

    uint16_t written = _tx_fifo->write(content, len);

    // TODO multiple prph connections
    // Not up to GATT MTU, notify will be sent later by TXD timer handler
    if ( _tx_fifo->count() < (Bluefruit.Gap.getMTU( Bluefruit.connHandle() ) - 3) )
    {
      return len;
    }
    else
    {
      // TX fifo has enough data, send notify right away
      VERIFY( flush_tx_buffered(), 0);

      // still more data left, send them all
      if ( written < len )
      {
         // write any additional data to FIFO,
         // update total number of bytes written to FIFO
         written += _tx_fifo->write(content+written, len-written);
      }

      // return actual number of bytes written to FIFO
      return written;
    }
  }
}

int BLEUart::available (void)
{
  return _rx_fifo->count();
}

int BLEUart::peek (void)
{
  uint8_t ch;
  return _rx_fifo->peek(&ch) ? (int) ch : EOF;
}

void BLEUart::flush (void)
{
  _rx_fifo->clear();
}

bool BLEUart::flush_tx_buffered(void)
{
  uint16_t max_hvx = Bluefruit.Gap.getMTU( Bluefruit.connHandle() ) - 3;
  uint8_t* ff_data = (uint8_t*) rtos_malloc( max_hvx );

  if (!ff_data) return false;

  uint16_t len = _tx_fifo->read(ff_data, max_hvx);
  bool result = true;

  if ( len )
  {
    result = _txd.notify(ff_data, len);
  }

  rtos_free(ff_data);

  return result;
}




