/**************************************************************************/
/*!
    @file     BLECentralUart.h
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
#ifndef BLECLIENTUART_H_
#define BLECLIENTUART_H_

#include "bluefruit_common.h"
#include "utility/adafruit_fifo.h"

#include "BLEClientCharacteristic.h"
#include "BLEClientService.h"

#include "services/BLEUart.h"

class BLEClientUart : public BLEClientService, public Stream
{
  public:
    // Callback Signatures
    typedef void (*rx_callback_t) (BLEClientUart& svc);

    BLEClientUart(uint16_t fifo_depth = BLE_UART_DEFAULT_FIFO_DEPTH);

    virtual bool  begin(void);
    virtual bool  discover(uint16_t conn_handle);

    void setRxCallback( rx_callback_t fp);

    bool enableTXD(void);
    bool disableTXD(void);

    // Stream API
    virtual int       read       ( void );
    virtual int       read       ( uint8_t * buf, size_t size );
            int       read       ( char    * buf, size_t size ) { return read( (uint8_t*) buf, size); }
    virtual size_t    write      ( uint8_t b );
    virtual size_t    write      ( const uint8_t *content, size_t len );
    virtual int       available  ( void );
    virtual int       peek       ( void );
    virtual void      flush      ( void );

    // pull in write(str) and write(buf, size) from Print
    using Print::write;

  protected:
    virtual void  disconnect(void);

  private:
    BLEClientCharacteristic _txd;
    BLEClientCharacteristic _rxd;

    Adafruit_FIFO     _rx_fifo;
    rx_callback_t     _rx_cb;

    friend void bleuart_central_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
};

#endif /* BLECLIENTUART_H_ */
