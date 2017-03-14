/**************************************************************************/
/*!
    @file     BLEMidi.cpp
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

// GCC 5x new feature to detect optional include
#ifdef __has_include
#if __has_include("MIDI.h")
  #include <MIDI.h>
  #define MIDI_LIB_INCLUDED
#endif
#endif

#define SYSEX_START   0xF0
#define SYSEX_END     0xF7

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
  uint8_t data[3];
} midi_event_packet_t;

VERIFY_STATIC ( sizeof(midi_event_packet_t) == 5 );

void blemidi_write_cb(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset);

/*------------------------------------------------------------------*/
/* IMPLEMENTATION
 *------------------------------------------------------------------*/
BLEMidi::BLEMidi(uint16_t fifo_depth)
  : BLEService(BLEMIDI_UUID_SERVICE), _io(BLEMIDI_UUID_CHR_IO), _rxd_fifo(fifo_depth, 1)
{
  _write_cb    = NULL;
  _midilib_obj = NULL;
  _receiving_sysex = false;
}

bool BLEMidi::notifyEnabled(void)
{
  return Bluefruit.connBonded() && _io.notifyEnabled();
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
  VERIFY_STATUS( this->addToGatt() );

  // IO characteristic
  _io.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY);
  _io.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
  _io.setWriteCallback(blemidi_write_cb);

  VERIFY_STATUS( _io.begin() );

  // Attempt to change the connection interval to 11.25-15 ms when starting HID
  Bluefruit.setConnInterval(9, 12);

  return ERROR_NONE;
}

/*------------------------------------------------------------------*/
/* Callbacks
 *------------------------------------------------------------------*/
void blemidi_write_cb(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  (void) offset;
  if ( len < 3 ) return;

  BLEMidi& midi_svc = (BLEMidi&) chr.parentService();
  midi_svc._write_handler(data, len);
}

void BLEMidi::_write_handler(uint8_t* data, uint16_t len)
{
  // First 3 bytes is always : Header + Timestamp + Status
  midi_header_t header;
  uint16_t tstamp = 0;
  uint8_t  status = 0;

  header.byte = *data++;
  len--;

  /* SysEx message
   * Header | Timestamp | SysEx Start (0xF0) | SysEx Data ...... | timestamp | SysEx End (0xF7)
   * For each MTU packet, SysEx Data is preceded by Header
   */

  // Start packet of SysEx
  if ( !_receiving_sysex && (data[1] == SYSEX_START) )
  {
    // skip timestamp
    data++;
    len--;

    _receiving_sysex = true;
  }

  // Receiving SysEx (including first packet above)
  if (_receiving_sysex)
  {
    // If final packet --> skip timestamp
    if ( (data[len-1] == SYSEX_END) && bitRead(data[len-2],7) )
    {
      _rxd_fifo.write(data, len-2);
      _rxd_fifo.write(&data[len-1]); // SYSEX_END

      _receiving_sysex = false; // done with SysEx
    }else
    {
      _rxd_fifo.write(data, len);
    }
  }else
  {
    /* Normal Event data
     * event : 0x00 - 0x7F
     * status: 0x80 - 0xEF
     */

    while (len)
    {
      if ( bitRead(data[0], 7) )
      {
        // Start of new full event
        midi_timestamp_t timestamp;

        timestamp.byte = *data++;
        len--;

        tstamp = (header.timestamp_hi << 7) | timestamp.timestamp_low;
        (void) tstamp;

        status = *data++;
        len--;

        // Status must have 7th-bit set, otherwise something is wrong !!!
        if ( !bitRead(status, 7) ) return;

        uint8_t tempbuf[3] = { status, data[0], data[1] };

        _rxd_fifo.write(tempbuf, 3);
        if ( _write_cb ) _write_cb();

        len  -= 2;
        data += 2;
      }
      else
      {
        // Running event
        uint8_t tempbuf[3] = { status, data[0], data[1] };

        _rxd_fifo.write(tempbuf, 3);
        if ( _write_cb ) _write_cb();

        len  -= 2;
        data += 2;
      }
    }
  }

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
  // MIDI Library will write event byte by byte. Locally buffered
  // Until we gather all 3 bytes
  static uint8_t count = 0;
  static uint8_t buf[3] = { 0 };


  // Not SysEx message, keep accumulating data
  if ( buf[0] != 0xf0 )
  {
    buf[count++] = b;

    if ( count == 3 )
    {
      count = 0;

      send(buf);
    }
  }else
  {
    // skip until we reach 0xF7
    if (b == 0xF7)
    {
      buf[0] = 0;
      count  = 0;
    }
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
/* Send Event (notify)
 *------------------------------------------------------------------*/
 err_t BLEMidi::send(uint8_t data[])
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

  memcpy(event.data, data, 3);

  VERIFY_STATUS( _io.notify(&event, sizeof(event)) );

  return ERROR_NONE;
}

err_t BLEMidi::send(uint8_t status, uint8_t byte1, uint8_t byte2)
{
  uint8_t data[] = { status, byte1, byte2 };
  return send(data);
}

