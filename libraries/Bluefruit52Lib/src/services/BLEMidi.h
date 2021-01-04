/**************************************************************************/
/*!
    @file     BLEMidi.h
    @author   hathach (tinyusb.org) & toddtreece

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com) All rights reserved.

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
#ifndef BLEMIDI_H_
#define BLEMIDI_H_

#include "bluefruit_common.h"
#include "utility/adafruit_fifo.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

#define MIDI_CREATE_BLE_INSTANCE(midiService)   MIDI_CREATE_INSTANCE(BLEMidi, midiService, MIDI)

#define BLE_MIDI_DEFAULT_FIFO_DEPTH   128
#define BLE_MIDI_TX_BUFFER_SIZE       3

extern const uint8_t BLEMIDI_UUID_SERVICE[];
extern const uint8_t BLEMIDI_UUID_CHR_IO[];

class BLEMidi: public BLEService, public Stream
{
  public:
    typedef void (*midi_write_cb_t) (uint16_t conn_hdl);

    BLEMidi(uint16_t fifo_depth = BLE_MIDI_DEFAULT_FIFO_DEPTH);

    virtual err_t begin(void);
    void begin(int baudrate); // MidiInterface

    bool notifyEnabled(void);
    bool notifyEnabled(uint16_t conn_hdl);

    bool send(uint8_t data[], uint8_t len);
    bool sendSplit(uint8_t data[], uint8_t len);

    // message type helpers
    bool isStatusByte(uint8_t b);
    bool oneByteMessage(uint8_t status);
    bool twoByteMessage(uint8_t status);
    bool threeByteMessage(uint8_t status);

    void setWriteCallback(midi_write_cb_t fp);
    void autoMIDIread(void* midi_obj);

    bool beginTransmission(uint8_t type);
    void endTransmission();

    // Stream API for MIDI Interface
    virtual int    read       ( void );
    virtual size_t write      ( uint8_t b );
    virtual int    available  ( void );
    virtual int    peek       ( void );
    virtual void   flush      ( void );

    using Print::write; // pull in write(str) and write(buf, size) from Print

  private:
    BLECharacteristic _io;

    Adafruit_FIFO     _rxd_fifo;
    midi_write_cb_t   _write_cb;

    void* _midilib_obj;

    void _write_handler(uint16_t conn_hdl, uint8_t* data, uint16_t len);

    static void blemidi_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
};



#endif /* BLEMIDI_H_ */
