/**************************************************************************/
/*!
    @file     BLEUart.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
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

// Constructor
BLEUart::BLEUart(uint16_t fifo_depth)
  : BLEService(BLEUART_UUID_SERVICE), _txd(BLEUART_UUID_CHR_TXD), _rxd(BLEUART_UUID_CHR_RXD)
{
  _rx_fifo       = NULL;
  _rx_fifo_depth = fifo_depth;

  _rx_cb         = NULL;
  _notify_cb     = NULL;
  _overflow_cb   = NULL;

  _tx_fifo       = NULL;
  _tx_buffered   = false;
}

// Destructor
BLEUart::~BLEUart()
{
  if ( _tx_fifo ) delete _tx_fifo;
}

// Callback when received new data
void BLEUart::bleuart_rxd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  BLEUart& svc = (BLEUart&) chr->parentService();
  uint16_t wrcount = svc._rx_fifo->write(data, len);

  if ( wrcount < len )
  {
    LOG_LV1("MEMORY", "bleuart rxd fifo OVERFLOWED!");

    // invoke overflow callback
    if (svc._overflow_cb) svc._overflow_cb(conn_hdl, len - wrcount);
  }

#if CFG_DEBUG >= 2
  LOG_LV2("BLEUART", "RX: ");
  PRINT_BUFFER(data, len);
#endif

  // invoke user callback
  if ( svc._rx_cb ) svc._rx_cb(conn_hdl);
}

void BLEUart::bleuart_txd_cccd_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value)
{
  BLEUart& svc = (BLEUart&) chr->parentService();

  if ( svc._notify_cb ) svc._notify_cb(conn_hdl, value & BLE_GATT_HVX_NOTIFICATION);
}

void BLEUart::setRxCallback(rx_callback_t fp, bool deferred)
{
  _rx_cb = fp;

  _rxd.setWriteCallback(BLEUart::bleuart_rxd_cb, deferred);
}

void BLEUart::setRxOverflowCallback(rx_overflow_callback_t fp)
{
  _overflow_cb = fp;
}

void BLEUart::setNotifyCallback(notify_callback_t fp)
{
  _notify_cb = fp;
  _txd.setCccdWriteCallback( fp ? BLEUart::bleuart_txd_cccd_cb : NULL );
}

/**
 * Enable packet buffered for TXD
 * Note: packet is sent right away if it reach MTU bytes
 * @param enable true or false
 */
void BLEUart::bufferTXD(bool enable)
{
  _tx_buffered = enable;

  if ( enable )
  {
    // Create FIFO for TXD
    if ( _tx_fifo == NULL )
    {
      _tx_fifo = new Adafruit_FIFO(1);
      _tx_fifo->begin( Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH) );
    }
  }else
  {
    if ( _tx_fifo ) delete _tx_fifo;
  }
}

err_t BLEUart::begin(void)
{
  _rx_fifo = new Adafruit_FIFO(1);
  _rx_fifo->begin(_rx_fifo_depth);

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  uint16_t max_mtu = Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH);

  // Add TXD Characteristic
  _txd.setProperties(CHR_PROPS_NOTIFY);
  // TODO enable encryption when bonding is enabled
  _txd.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  _txd.setMaxLen( max_mtu );
  _txd.setUserDescriptor("TXD");
  VERIFY_STATUS( _txd.begin() );

  // Add RXD Characteristic
  _rxd.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  _rxd.setWriteCallback(BLEUart::bleuart_rxd_cb, true);

  // TODO enable encryption when bonding is enabled
  _rxd.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  _rxd.setMaxLen( max_mtu );
  _rxd.setUserDescriptor("RXD");
  VERIFY_STATUS(_rxd.begin());

  return ERROR_NONE;
}

bool BLEUart::notifyEnabled(void)
{
  return this->notifyEnabled(Bluefruit.connHandle());
}

bool BLEUart::notifyEnabled(uint16_t conn_hdl)
{
  return _txd.notifyEnabled(conn_hdl);
}

/*------------------------------------------------------------------*/
/* STREAM API
 *------------------------------------------------------------------*/
int BLEUart::read (void)
{
  uint8_t ch;
  return read(&ch, 1) ? (int) ch : EOF;
}

int BLEUart::read(uint8_t * buf, size_t size)
{
  return _rx_fifo->read(buf, size);
}

uint8_t BLEUart::read8 (void)
{
  uint8_t num;
  return read(&num, sizeof(num)) ? num : 0;
}

uint16_t BLEUart::read16(void)
{
  uint16_t num;
  return read((uint8_t*) &num, sizeof(num)) ? num : 0;
}

uint32_t BLEUart::read32(void)
{
  uint32_t num;
  return read((uint8_t*) &num, sizeof(num)) ? num : 0;
}

size_t BLEUart::write(uint8_t b)
{
  return this->write(Bluefruit.connHandle(), &b, 1);
}

size_t BLEUart::write(uint16_t conn_hdl, uint8_t b)
{
  return this->write(conn_hdl, &b, 1);
}

size_t BLEUart::write(const uint8_t *content, size_t len)
{
  return this->write(Bluefruit.connHandle(), content, len);
}

size_t BLEUart::write(uint16_t conn_hdl, const uint8_t *content, size_t len)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  VERIFY(conn, 0);

  // skip if not enabled
  if ( !notifyEnabled(conn_hdl) ) return 0;

  // notify right away if txd buffered is not enabled
  if ( !(_tx_buffered && _tx_fifo) )
  {
    return _txd.notify(conn_hdl, content, len) ? len : 0;
  }else
  {
    uint16_t written = _tx_fifo->write(content, len);

    // Not up to GATT MTU, notify will be sent later by TXD timer handler
    if ( _tx_fifo->count() < (conn->getMtu() - 3) )
    {
      return len;
    }
    else
    {
      // TX fifo has enough data, send notify right away
      VERIFY( flushTXD(conn_hdl), 0);

      // still more data left, send them all
      if ( written < len )
      {
        VERIFY(_txd.notify(conn_hdl, content+written, len-written), written);
      }

      return len;
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

bool BLEUart::flushTXD (void)
{
  return flushTXD(Bluefruit.connHandle());
}

bool BLEUart::flushTXD(uint16_t conn_hdl)
{
  BLEConnection* conn = Bluefruit.Connection(conn_hdl);
  VERIFY(conn);

  uint16_t const gatt_mtu = conn->getMtu() - 3;
  uint8_t* ff_data = (uint8_t*) rtos_malloc( gatt_mtu );
  VERIFY(ff_data);

  uint16_t len = _tx_fifo->read(ff_data, gatt_mtu);
  bool result = true;

  if ( len )
  {
    result = _txd.notify(conn_hdl, ff_data, len);
  }

  rtos_free(ff_data);

  return result;
}




