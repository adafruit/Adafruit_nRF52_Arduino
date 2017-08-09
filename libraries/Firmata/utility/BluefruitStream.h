/**************************************************************************/
/*!
    @file     BluefruitStream.h

    Wrapper for Adafruit Bluefruit nRF52
*/
/**************************************************************************/
#ifndef BLUEFRUITSTREAM_H_
#define BLUEFRUITSTREAM_H_

#ifdef ARDUINO_NRF52_ADAFRUIT

#include <bluefruit.h>

class BLEStream : public Stream
{
  private:
    BLEUart     _bleuart;
    const char* _name;
    uint16_t    _conn_min, _conn_max;

  public:
    BLEStream(void);

    void begin(void);
    bool poll();
    void setLocalName(const char* str) { _name = str; }
    void setConnectionInterval(uint16_t min, uint16_t max) { _conn_min = min; _conn_max = max; }
    void setFlushInterval(int interval) {} // flush interval is handled internally by BLEUart

    // Forward all stream API to bleuart
    virtual int    available (void)         { return _bleuart.available(); }
    virtual int    peek      (void)         { return _bleuart.peek();      }
    virtual int    read      (void)         { return _bleuart.read();      }
    virtual void   flush     (void)         { return _bleuart.flush();     }
    virtual size_t write (uint8_t byte)     { return _bleuart.write(byte); }

    virtual size_t write ( const uint8_t *content, size_t len )
    {
      return _bleuart.write(content, len);
    }
    using Print::write;

//    virtual operator bool();
};

BLEStream::BLEStream(void)
{
  _name = NULL;
  _conn_min = 9;  // 9*1.25=11.25 ms
  _conn_max = 16; // 16*1.25=20ms
}

void BLEStream::begin(void)
{
  Bluefruit.begin();

  // set name
  if (_name) Bluefruit.setName(_name);

  // set connection interval
  Bluefruit.setConnInterval(_conn_min, _conn_max);

  // Configure and Start BLE Uart Service
  // Firmata use several small write(1) --> buffering TXD is required to run smoothly
  // Enable buffering TXD
  _bleuart.begin();
  _bleuart.bufferTXD(true);

  /*------------- Advertising -------------*/
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(_bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**
 * Flush is handled by BLEUart internally.
 * @return true if connected and bleuart notify is enabled
 */
bool BLEStream::poll(void)
{
  return Bluefruit.connected() && _bleuart.notifyEnabled();
}

#endif

#endif /* BLUEFRUITSTREAM_H_ */
