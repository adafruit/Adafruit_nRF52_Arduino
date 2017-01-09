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
BLEUart bleuart;
BLEBas blebas;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("BLEUART Example");

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit 52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.setHardwareRev("Rev E");
  bledis.setFirmwareRev("0.1.0");
  bledis.start();

  // Configure and Start BLE Uart Service
  bleuart.start();

  // Start BLE Battery Service
  blebas.start();
  blebas.update(100);

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
  Bluefruit.addAdvService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.addScanName();
}

void loop() 
{
  // forward from Serial to BLEUART
  if (Serial.available())
  {
    // delay to get enough inputs, since we have limited number of transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
  }

  // forward from BLEUART to Serial
  if ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }
}

