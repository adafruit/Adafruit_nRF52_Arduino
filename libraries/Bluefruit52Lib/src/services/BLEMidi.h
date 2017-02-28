/**************************************************************************/
/*!
    @file     BLEMidi.h
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
#ifndef BLEMIDI_H_
#define BLEMIDI_H_

#include "bluefruit_common.h"
#include "utility/adafruit_fifo.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

extern const uint8_t BLEMIDI_UUID_SERVICE[];
extern const uint8_t BLEMIDI_UUID_CHR_IO[];

enum
{
  MIDI_TYPE_NOTE_OFF         = 0x8,
  MIDI_TYPE_NOTE_ON          = 0x9,
  MIDI_TYPE_AFTER_TOUCH      = 0xA,
  MIDI_TYPE_CONTROL_CHANGE   = 0xB,
  MIDI_TYPE_PROGRAM_CHANGE   = 0xC,
  MIDI_TYPE_CHANNEL_PRESSURE = 0xD,
  MIDI_TYPE_PITCH_WHEEL      = 0xE,
};

class BLEMidi: public BLEService
{
  public:
    BLEMidi(void);

    virtual err_t start(void);

    bool  configured();

    err_t send(uint8_t data[]);
    err_t send(uint8_t status, uint8_t byte1, uint8_t byte2);

    void setWriteCallback(BLECharacteristic::write_cb_t fp);

  private:
    BLECharacteristic _io;

    friend void blemidi_write_cb(BLECharacteristic& chr, ble_gatts_evt_write_t* request);
};



#endif /* BLEMIDI_H_ */
