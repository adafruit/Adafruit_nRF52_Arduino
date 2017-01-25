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
BLEHidAdafruit blehid;

bool hasKeyPressed = false;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 HID Keyboard Example");

  Bluefruit.begin();
  // HID Device can have min connection interval up to 9*1.25 = 11.25 ms
  Bluefruit.setConnInterval(9, 16); // min = 11.25 ms, max = 20ms
  Bluefruit.setName("Bluefruit52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.setHardwareRev("Rev E");
  bledis.setFirmwareRev("0.1.0");
  bledis.start();

  // Configure and Start BLE Uart Service
  bleuart.start();

  // BLE Hid
  blehid.start();

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.startAdvertising();
}

void setupAdv(void)
{  
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();

  Bluefruit.addAdvApperance(BLE_APPEARANCE_HID_KEYBOARD);
  
  // Include ble hid service
  Bluefruit.addAdvService(blehid);

  // There is enough room for Name in Advertising packet
  Bluefruit.addAdvName();
}

void loop() 
{
  // Only send KeyRelease if previously pressed, to avoid sending
  // multiple keyRelease report (that consume memories and bandwidth)
  if ( hasKeyPressed )
  {
    hasKeyPressed = false;
    blehid.keyRelease();

    // delay a bit after a report
    delay(5);
  }
    
  if (Serial.available())
  {
    char ch = (char) Serial.read();

    // ehco
    Serial.write(ch); 

    blehid.keyPress(ch);
    hasKeyPressed = true;

    // delay a bit after a report
    delay(5);
  }
}

