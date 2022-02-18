/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example requires following library
 * - https://github.com/FortySevenEffects/arduino_midi_library/ (MIDI Library)
 *
 * For BLE MIDI Setup on PC/mobile checkout our learn guide
 * - https://learn.adafruit.com/wireless-untztrument-using-ble-midi/ble-midi-setup
 */


#include <bluefruit.h>
#include <MIDI.h>

BLEDis bledis;
BLEMidi blemidi;

// Create a new instance of the Arduino MIDI Library,
// and attach BluefruitLE MIDI as the transport.
MIDI_CREATE_BLE_INSTANCE(blemidi);

// Variable that holds the current position in the sequence.
unsigned int position = 0;

// Store example melody as an array of note values
byte note_sequence[] = {
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Adafruit Bluefruit52 MIDI over Bluetooth LE Example");

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // Setup the on board blue LED to be enabled on CONNECT
  Bluefruit.autoConnLed(true);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call blemidi service's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);
  
  // Set up and start advertising
  startAdv();

  // Start MIDI read loop
  Scheduler.startLoop(midiRead);
}

void startAdv(void)
{
  // Set General Discoverable Mode flag
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertise TX Power
  Bluefruit.Advertising.addTxPower();

  // Advertise BLE MIDI Service
  Bluefruit.Advertising.addService(blemidi);

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

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Log when a note is pressed.
  Serial.printf("Note on: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
  Serial.println();
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Log when a note is released.
  Serial.printf("Note off: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
  Serial.println();
}

void loop()
{
  // Don't continue if we aren't connected.
  if (! Bluefruit.connected()) {
    return;
  }

  // Don't continue if the connected device isn't ready to receive messages.
  if (! blemidi.notifyEnabled()) {
    return;
  }

  // Setup variables for the current and previous
  // positions in the note sequence.
  unsigned int current = position;
  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  unsigned int previous = (current == 0) ? (sizeof(note_sequence)-1) : current - 1;

  // Send Note On for current position at full velocity (127) on channel 1.
  MIDI.sendNoteOn(note_sequence[current], 127, 1);

  // Send Note Off for previous note.
  MIDI.sendNoteOff(note_sequence[previous], 0, 1);

  // Increment position
  position++;

  // If we are at the end of the sequence, start over.
  if (position >= sizeof(note_sequence)) {
    position = 0;
  }

  delay(286);

}

void midiRead()
{
  // Don't continue if we aren't connected.
  if (! Bluefruit.connected()) {
    return;
  }

  // Don't continue if the connected device isn't ready to receive messages.
  if (! blemidi.notifyEnabled()) {
    return;
  }

  // read any new MIDI messages
  MIDI.read();
}

