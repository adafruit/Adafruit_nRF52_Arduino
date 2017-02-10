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

#define MOVE_STEP    10

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 HID Mouse Example");
  Serial.println("- Enter 'WASD' for move mouse (up, left, down, right)");
  Serial.println("- Enter 'LRMBF'  for pressing mouse buttons (left, right, middle, backward, forward)");
  Serial.println("- Enter 'X'    for releasing mouse buttons");

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

  Bluefruit.addAdvApperance(BLE_APPEARANCE_HID_MOUSE);
  
  // Include ble hid service
  Bluefruit.addAdvService(blehid);

  // There is enough room for Name in Advertising packet
  Bluefruit.addAdvName();
}

void loop() 
{    
  if (Serial.available())
  {
    char ch = (char) Serial.read();

    // convert to upper case
    ch = (char) toupper(ch);
    
    // ehco
    Serial.println(ch);
  
    switch(ch)
    {
      // WASD is mouse move
      case 'W':
        blehid.mouseMove(0, -MOVE_STEP);
      break;

      case 'A':
        blehid.mouseMove(-MOVE_STEP, 0);
      break;

      case 'S':
        blehid.mouseMove(0, MOVE_STEP);
      break;

      case 'D':
        blehid.mouseMove(MOVE_STEP, 0);
      break;

      // LRM is mouse button
      case 'L':
        blehid.mouseButtonPress(MOUSE_BUTTON_LEFT);
      break;

      case 'R':
        blehid.mouseButtonPress(MOUSE_BUTTON_RIGHT);
      break;

      case 'M':
        blehid.mouseButtonPress(MOUSE_BUTTON_MIDDLE);
      break;

      case 'B':
        blehid.mouseButtonPress(MOUSE_BUTTON_BACKWARD);
      break;

      case 'F':
        // This key is not always supported by OS
        blehid.mouseButtonPress(MOUSE_BUTTON_FORWARD);
      break;

      case 'X':
        blehid.mouseButtonPress(0);
      break;

      default: break;
    }
  }
}

