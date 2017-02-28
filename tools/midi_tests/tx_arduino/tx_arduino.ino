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

  Serial.println("Bluefruit52 BLEMIDI TX Test");

  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  blemidi.start();
  
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();
  Bluefruit.addAdvService(blemidi);
  Bluefruit.addScanRespName();
  
  Bluefruit.startAdvertising();
  
}

void loop()
{
  
  if (! Bluefruit.connected())
    return;
    
  if (! blemidi.configured())
    return;
    
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
    blemidi.send(0x90, on_count, 0x64);
    on_count++;
    prev_on = currentMillis;
  }

  if (currentMillis - prev_off >= off_interval && off_count < 128) {
    blemidi.send(0x90, off_count, 0x00);
    off_count++;
    prev_off = currentMillis;
  }
  
}
