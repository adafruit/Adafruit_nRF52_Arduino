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
BLEHidAdafruit blehid;

#define MOVE_STEP    10

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 HID Mouse Example");
  Serial.println("-----------------------------\n");
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open an application that accepts mouse input");
  Serial.println();

  Serial.println("Enter following characters");
  Serial.println("- 'WASD'  to move mouse (up, left, down, right)");
  Serial.println("- 'LRMBF' to press mouse button(s) (left, right, middle, backward, forward)");
  Serial.println("- 'X'     to release mouse button(s)");

  Bluefruit.begin();
  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();
  
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

void loop() 
{    
  if (Serial.available())
  {
    char ch = (char) Serial.read();

    // convert to upper case
    ch = (char) toupper(ch);
    
    // echo
    Serial.println(ch);
  
    switch(ch)
    {
      // WASD to move the mouse
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

      // LRMBF for mouse button(s)
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
        // This key is not always supported by every OS
        blehid.mouseButtonPress(MOUSE_BUTTON_FORWARD);
      break;

      case 'X':
        // X to release all buttons
        blehid.mouseButtonRelease();
      break;

      default: break;
    }
  }
}

