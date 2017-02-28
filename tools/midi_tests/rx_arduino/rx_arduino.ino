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

BLEMidi blemidi;

int on_count = 0;
int off_count = 0;

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 BLEMIDI RX Test");

  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");
  
  blemidi.setWriteCallback(midi_callback);
  blemidi.start();
  
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();
  Bluefruit.addAdvService(blemidi);
  Bluefruit.addScanRespName();
  
  Bluefruit.startAdvertising();
}

void loop() {}

void midi_callback(uint32_t timestamp, uint8_t data[])
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(data[0], HEX); Serial.print(" ");
  Serial.print(data[1] , HEX); Serial.print(" ");
  Serial.print(data[2] , HEX); Serial.print(" ");

  if(data[0] == 0x90 && data[2] == 0x00) {
    off_count++;
  } else if(data[0] == 0x90 && data[2] == 0x7F) {
    on_count++;
  }

  Serial.println();

  Serial.print("note on count: ");
  Serial.println(on_count, DEC);
  Serial.print("note off count: ");
  Serial.println(off_count, DEC);
  
}

