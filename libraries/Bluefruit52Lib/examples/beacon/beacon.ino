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

// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
// 0x004C is Apple
#define MANUFACTURER_ID   0x004C 

// AirLocate UUID: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
uint8_t beaconUuid[16] = 
{ 
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0, 
};

// uuid, major, minor, rssi @1m
BLEBeacon beacon(beaconUuid, 0x0001, 0x0000, -54);

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Beacon Example");

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  beacon.setManufacturer(MANUFACTURER_ID);

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{  
  Bluefruit.Advertising.setBeacon(beacon);

  char* adv = Bluefruit.Advertising.getData();

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_BUILTIN);
  delay(1000);
}

