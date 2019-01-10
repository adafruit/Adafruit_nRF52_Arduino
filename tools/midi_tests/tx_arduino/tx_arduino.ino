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
#include <bluefruit.h>
#include <MIDI.h>

BLEMidi blemidi;

// Create a new instance of the Arduino MIDI Library,
// and attach BluefruitLE MIDI as the transport.
MIDI_CREATE_BLE_INSTANCE(blemidi);

bool startTest = false;
int on_count = 0;
int off_count = 0;

unsigned long on_interval = 5;
unsigned long off_interval = on_interval;

unsigned long prev_on = millis();
unsigned long prev_off = millis();

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 BLEMIDI TX Test");
  Serial.println("---------------------------\n");

  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52 MIDI");

  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Set General Discoverable Mode flag
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertise BLE MIDI Service
  Bluefruit.Advertising.addService(blemidi);

  // Advertise device name in the Scan Response
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

  Serial.println("Waiting for connection...");
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

  unsigned long currentMillis = millis();

  if(! startTest)  {
    Serial.println();
    Serial.println("Start the node TX test and press a key in the serial monitor to start the test...");
    while(Serial.available() == 0) { }
    while(Serial.available()) { Serial.read(); }
    startTest = true;
    prev_on = currentMillis;
    prev_off = currentMillis;
    Serial.println("Starting test.");
  }

  if(on_count >= 128 && off_count >= 128) {
    Serial.println("Test finished. Resetting.");
    startTest = false;
    on_count = 0;
    off_count = 0;
    return;
  }

  if (currentMillis - prev_on >= on_interval && on_count < 128) {
    MIDI.sendNoteOn(on_count, 100, 1);
    on_count++;
    prev_on = currentMillis;
  }

  if (currentMillis - prev_off >= off_interval && off_count < 128) {
    MIDI.sendNoteOff(off_count, 0, 1);
    off_count++;
    prev_off = currentMillis;
  }

}
