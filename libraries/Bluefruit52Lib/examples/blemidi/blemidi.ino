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

BLEDis bledis;
BLEMidi blemidi;

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 BLEMIDI Example");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.setHardwareRev("Rev E");
  bledis.setFirmwareRev("0.1.0");
  bledis.start();

  // Configure and Start BLE MIDI Service
  blemidi.start();

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.startAdvertising();
}

void setupAdv(void)
{
  //Bluefruit.addAdvTxPower();
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.addAdvService(blemidi);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.addScanRespName();
}

void loop()
{

}
