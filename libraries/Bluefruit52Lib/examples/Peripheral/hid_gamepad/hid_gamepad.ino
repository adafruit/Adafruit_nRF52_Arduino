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

hid_gamepad_report_t gp;        // defined in hid.h from Adafruit_TinyUSB_ArduinoCore
hid_gamepad_button_bm_t bbm;    // defined in hid.h from Adafruit_TinyUSB_ArduinoCore

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 HID Gamepad Example");
  Serial.println("-----------------------------\n");
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open an application that accepts mouse input");
  Serial.println();

  Bluefruit.begin();

  // HID Device can have a min connection interval of 9*1.25 = 11.25 ms
  Bluefruit.Periph.setConnInterval(9, 16); // min = 9*1.25=11.25 ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);                 // Check bluefruit.h for supported values
  Bluefruit.setName("nRF52 Gamepad");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  // BLE HID
  blehid.begin();

  // Set up and start advertising
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);

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

  Serial.print("Attempting to connect");
  uint8_t i=0;
  while(! Bluefruit.connected()) { 
    Serial.print("."); delay(100);
    if((++i)>50) {
      i=0; Serial.println();
    }
  };
  delay(1000);//just in case
  Serial.println("\nConnection successful");
}

void loop() 
{    
  // Reset buttons
  Serial.println("No pressing buttons");
  gp.buttons = 0;
  gp.x       = 0;
  gp.y       = 0;
  gp.z       = 0;
  gp.r_z     = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  
  // Joystick 1 UP
  Serial.println("Joystick 1 UP");
  gp.x = 0;
  gp.y = -127;
  blehid.gamepadReport(0, &gp);
  delay(2000);
  
  // Joystick 1 DOWN
  Serial.println("Joystick 1 DOWN");
  gp.x = 0;
  gp.y = 127;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 1 LEFT
  Serial.println("Joystick 1 LEFT");
  gp.x = -127;
  gp.y = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 1 RIGHT
  Serial.println("Joystick 1 RIGHT");
  gp.x = 127;
  gp.y = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 1 CENTER
  Serial.println("Joystick 1 CENTER");
  gp.x = 0;
  gp.y = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);


  // Joystick 2 UP
  Serial.println("Joystick 2 UP");
  gp.z   = 0;
  gp.r_z = 127;
  blehid.gamepadReport(0, &gp);
  delay(2000);
  
  // Joystick 2 DOWN
  Serial.println("Joystick 2 DOWN");
  gp.z   = 0;
  gp.r_z = -127;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 2 LEFT
  Serial.println("Joystick 2 LEFT");
  gp.z   = -127;
  gp.r_z = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 2 RIGHT
  Serial.println("Joystick 2 RIGHT");
  gp.z   = 127;
  gp.r_z = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  // Joystick 2 CENTER
  Serial.println("Joystick 2 CENTER");
  gp.z   = 0;
  gp.r_z = 0;
  blehid.gamepadReport(0, &gp);
  delay(2000);

  
  // Test buttons
  for (int i=0; i<=15; ++i)
  {
    Serial.print("Pressing button "); Serial.println(i+1);
    gp.buttons = (0x00 | TU_BIT(i));
    blehid.gamepadReport(0, &gp);
    delay(1000);
  }


  // Random touch
  Serial.println("Random touch");
  gp.buttons = random(0, 0xffff);
  gp.x       = random(-127, 128);
  gp.y       = random(-127, 128);
  gp.z       = random(-127, 128);
  gp.r_z     = random(-127, 128);
  blehid.gamepadReport(0, &gp);
  delay(2000);
}
