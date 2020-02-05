/**************************************************************************/
/*!
    @file     BLEMidi.cpp
    @author   hathach (tinyusb.org) & toddtreece

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

// GCC 5x new feature to detect optional include
#ifdef __has_include
#if __has_include("MIDI.h")
  #include <MIDI.h>
  #define MIDI_LIB_INCLUDED
#endif
#endif


/* MIDI Service: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
 * MIDI I/O    : 7772E5DB-3868-4112-A1A9-F2669D106BF3
 */

const uint8_t BLEMIDI_UUID_SERVICE[] =
{
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

const uint8_t BLEMIDI_UUID_CHR_IO[] =
{
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};

/*------------------------------------------------------------------*/
/* MIDI Data Type
 *------------------------------------------------------------------*/
typedef union ATTR_PACKED
{
  struct {
    uint8_t timestamp_hi : 6;
    uint8_t              : 1;
    uint8_t start_bit    : 1;
  };

  uint8_t byte;
} midi_header_t;

VERIFY_STATIC ( sizeof(midi_header_t) == 1 );

typedef union ATTR_PACKED
{
  struct {
    uint8_t timestamp_low : 7;
    uint8_t start_bit : 1;
  };

  uint8_t byte;
} midi_timestamp_t;

VERIFY_STATIC ( sizeof(midi_timestamp_t) == 1 );

typedef struct ATTR_PACKED
{
  midi_header_t header;
  midi_timestamp_t timestamp;
  uint8_t data[BLE_MIDI_TX_BUFFER_SIZE];
} midi_event_packet_t;

VERIFY_STATIC ( sizeof(midi_event_packet_t) == (BLE_MIDI_TX_BUFFER_SIZE + 2) );

typedef struct ATTR_PACKED
{
  midi_header_t header;
  uint8_t data[BLE_MIDI_TX_BUFFER_SIZE];
} midi_split_packet_t;

VERIFY_STATIC ( sizeof(midi_split_packet_t) == (BLE_MIDI_TX_BUFFER_SIZE + 1) );

/*------------------------------------------------------------------*/
/* IMPLEMENTATION
 *------------------------------------------------------------------*/
BLEMidi::BLEMidi(uint16_t fifo_depth)
  : BLEService(BLEMIDI_UUID_SERVICE), _io(BLEMIDI_UUID_CHR_IO), _rxd_fifo(1, fifo_depth)
{
  _write_cb    = NULL;
  _midilib_obj = NULL;
}

bool BLEMidi::notifyEnabled(void)
{
  return _io.notifyEnabled();
}

bool BLEMidi::notifyEnabled(uint16_t conn_hdl)
{
  return _io.notifyEnabled(conn_hdl);
}

void BLEMidi::setWriteCallback(midi_write_cb_t fp)
{
  _write_cb = fp;
}

void BLEMidi::autoMIDIread(void* midi_obj)
{
  _midilib_obj = midi_obj;
}
void BLEMidi::begin(int baudrate)
{
  (void) baudrate;
  begin();
}

err_t BLEMidi::begin(void)
{
  _rxd_fifo.begin();

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // IO characteristic
  _io.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY);
  _io.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
  _io.setWriteCallback(BLEMidi::blemidi_write_cb);

  VERIFY_STATUS( _io.begin() );

  // Attempt to change the connection interval to 11.25-15 ms when starting HID
  Bluefruit.Periph.setConnInterval(9, 12);

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* Callbacks
 *------------------------------------------------------------------*/
void BLEMidi::blemidi_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  if ( len < 3 ) return;

  BLEMidi& midi_svc = (BLEMidi&) chr->parentService();
  midi_svc._write_handler(conn_hdl, data, len);
}

void BLEMidi::_write_handler(uint16_t conn_hdl, uint8_t* data, uint16_t len)
{
  // drop the BLE MIDI header byte
  data++;
  len--;

  while (len)
  {
    // timestamp low byte followed by a MIDI status,
    // so we drop the timestamp low byte
    if ( isStatusByte(data[0]) && isStatusByte(data[1]) )
    {
      data++;
      len--;
    }
    // timestamp low byte on it's own (running status),
    // so we drop the timestamp low byte
    else if ( isStatusByte(data[0]) && ! isStatusByte(data[1]) )
    {
      data++;
      len--;
    }

    // write the status or MIDI data to the FIFO
    _rxd_fifo.write(data++, 1);
    len--;
  }

  // Call write callback if configured
  if ( _write_cb ) _write_cb(conn_hdl);

#ifdef MIDI_LIB_INCLUDED
  // read while possible if configured
  if ( _midilib_obj )
  {
    while( ((midi::MidiInterface<BLEMidi>*)_midilib_obj)->read() ) { }
  }
#endif

}

/*------------------------------------------------------------------*/
/* Stream API
 *------------------------------------------------------------------*/
int BLEMidi::read ( void )
{
  uint8_t ch;
  return _rxd_fifo.read(&ch) ? (int) ch : EOF;
}


size_t BLEMidi::write ( uint8_t b )
{
  // MIDI Library will write event byte by byte.
  // We need to buffer the data until we have a full event,
  // or until we reach the TX buffer limit.
  static uint8_t count = 0;
  static uint8_t buf[BLE_MIDI_TX_BUFFER_SIZE] = { 0 };

  // the current byte is a sysex end status message,
  // and we still have an existing buffer. send the
  // existing buffer and clear it so we can send
  // the sysex end status message with the appropriate
  // BLE header and timestamp bytes.
  if(b == 0xF7 && count > 0)
  {
    // send and clear the last of the existing buffer.
    // it should contain the final bytes in the sysex payload.
    if (isStatusByte(buf[0]))
      send(buf, count);
    else
      sendSplit(buf, count);

    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  // add the current byte to the buffer
  buf[count++] = b;

  // send matching 1, 2, or 3 byte messages
  // and clear the buffer
  if ( (oneByteMessage(buf[0]) && count == 1) ||
       (twoByteMessage(buf[0]) && count == 2) ||
       (threeByteMessage(buf[0]) && count == 3) )
  {
    send(buf, count);
    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  // do we have a full buffer at this point?
  if(count == BLE_MIDI_TX_BUFFER_SIZE)
  {
    // send a full or split message depending
    // on the type of the first byte in the buffer
    if (isStatusByte(buf[0]))
      send(buf, count);
    else
      sendSplit(buf, count);

    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  return 1;
}

int BLEMidi::available ( void )
{
  return _rxd_fifo.count();
}

int BLEMidi::peek ( void )
{
  uint8_t ch;
  return _rxd_fifo.peek(&ch) ? (int) ch : EOF;
}

void BLEMidi::flush ( void )
{
  _rxd_fifo.clear();
}

/*------------------------------------------------------------------*/
/* Message Type Helpers
 *------------------------------------------------------------------*/
bool BLEMidi::isStatusByte( uint8_t b )
{
  // if the bit 7 is set, then it's a MIDI status message
  if (bitRead(b, 7))
    return true;
  else
    return false;
}

bool BLEMidi::oneByteMessage( uint8_t status )
{
  // system messages
  if (status >= 0xF4) return true;

  // system common
  if (status == 0xF1) return true;

  // sysex end
  if (status == 0xF7) return true;

  return false;
}

bool BLEMidi::twoByteMessage( uint8_t status )
{
  // program change, aftertouch
  if (status >= 0xC0 && status <= 0xDF) return true;

  // song select
  if (status == 0xF3) return true;

  return false;
}

bool BLEMidi::threeByteMessage( uint8_t status )
{
  // note off, note on, aftertouch, control change
  if (status >= 0x80 && status <= 0xBF) return true;

  // pitch wheel change
  if (status >= 0xE0 && status <= 0xEF) return true;

  // song position pointer
  if (status == 0xF2) return true;

  return false;
}

/*------------------------------------------------------------------*/
/* Send Event (notify)
 *------------------------------------------------------------------*/
bool BLEMidi::send(uint8_t data[], uint8_t len)
{
  uint32_t tstamp = millis();

  midi_event_packet_t event =
  {
      .header = {{
          .timestamp_hi = (uint8_t) ((tstamp & 0x1F80UL) >> 7),
          .start_bit    = 1
      }},

      .timestamp = {{
          .timestamp_low = (uint8_t) (tstamp & 0x7FUL),
          .start_bit     = 1
      }}
  };

  memcpy(event.data, data, len);

  // send data length + 1 byte for header + 1 byte for timestamp
  return _io.notify(&event, len + 2);
}

bool BLEMidi::sendSplit(uint8_t data[], uint8_t len)
{
  uint32_t tstamp = millis();

  midi_split_packet_t event =
  {
      .header = {{
          .timestamp_hi = (uint8_t) ((tstamp & 0x1F80UL) >> 7),
          .start_bit    = 1
      }}
  };

  memcpy(event.data, data, len);

  // send data length + 1 byte for header
  // don't include the second timestamp byte
  return _io.notify(&event, len + 1);
}
