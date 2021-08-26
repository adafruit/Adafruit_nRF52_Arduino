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

/* This sketch demonstrate how to use BLEHidGamepad to send
 * button, dpad, and joystick report
 */

#include <bluefruit.h>

BLEDis bledis;
BLEHidGamepad blegamepad;

// defined in hid.h from Adafruit_TinyUSB_Arduino
hid_gamepad_report_t gp;

void setup()
{
  Serial.begin(115200);

#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial ) delay(10);
#endif

  Serial.println("Bluefruit52 HID Gamepad Example");
  Serial.println("-------------------------------\n");

  Serial.println();
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open an application that accepts gamepad input");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blegamepad.begin();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms
   */
  /* Bluefruit.Periph.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blegamepad);

  // There is enough room for the dev name in the advertising packet
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
  // nothing to do if not connected or
  if ( !Bluefruit.connected() ) return;

  Serial.println("No pressing buttons");
  gp.x       = 0;
  gp.y       = 0;
  gp.z       = 0;
  gp.rz      = 0;
  gp.rx      = 0;
  gp.ry      = 0;
  gp.hat     = 0;
  gp.buttons = 0;
  blegamepad.report(&gp);
  delay(1000);

  //------------- DPAD / HAT -------------//
  Serial.println("Hat/DPAD UP");
  gp.hat = GAMEPAD_HAT_UP;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD UP RIGHT");
  gp.hat = GAMEPAD_HAT_UP_RIGHT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD RIGHT");
  gp.hat = GAMEPAD_HAT_RIGHT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD DOWN RIGHT");
  gp.hat = GAMEPAD_HAT_DOWN_RIGHT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD DOWN");
  gp.hat = GAMEPAD_HAT_DOWN;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD DOWN LEFT");
  gp.hat = GAMEPAD_HAT_DOWN_LEFT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD LEFT");
  gp.hat = GAMEPAD_HAT_LEFT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD UP LEFT");
  gp.hat = GAMEPAD_HAT_UP_LEFT;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Hat/DPAD CENTER");
  gp.hat = GAMEPAD_HAT_CENTERED;
  blegamepad.report(&gp);
  delay(1000);

  //------------- Joystick 1 -------------//

  Serial.println("Joystick 1 UP");
  gp.x = 0;
  gp.y = -127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 1 DOWN");
  gp.x = 0;
  gp.y = 127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 1 RIGHT");
  gp.x = 127;
  gp.y = 0;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 1 LEFT");
  gp.x = -127;
  gp.y = 0;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 1 CENTER");
  gp.x = 0;
  gp.y = 0;
  blegamepad.report(&gp);
  delay(1000);


  //------------- Joystick 2 -------------//
  Serial.println("Joystick 2 UP");
  gp.z  = 0;
  gp.rz = 127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 2 DOWN");
  gp.z  = 0;
  gp.rz = -127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 2 RIGHT");
  gp.z  = 127;
  gp.rz = 0;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 2 LEFT");
  gp.z  = -127;
  gp.rz = 0;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Joystick 2 CENTER");
  gp.z  = 0;
  gp.rz = 0;
  blegamepad.report(&gp);
  delay(1000);

  //------------- Analog Trigger 1 -------------//
  Serial.println("Analog Trigger 1 UP");
  gp.rx = 127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Analog Trigger 1 DOWN");
  gp.rx = -127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Analog Trigger 1 CENTER");
  gp.rx = 0;
  blegamepad.report(&gp);
  delay(1000);

  //------------- Analog Trigger 2 -------------//
  Serial.println("Analog Trigger 2 UP");
  gp.ry = 127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Analog Trigger 2 DOWN");
  gp.ry = -127;
  blegamepad.report(&gp);
  delay(1000);

  Serial.println("Analog Trigger 2 CENTER");
  gp.ry = 0;
  blegamepad.report(&gp);
  delay(1000);

  //------------- Buttons -------------//
  for (int i=0; i<16; ++i)
  {
    Serial.print("Pressing button "); Serial.println(i);
    gp.buttons = (1ul << i);
    blegamepad.report(&gp);
    delay(1000);
  }

  // Random touch
  Serial.println("Random touch");
  gp.x       = random(-127, 128);
  gp.y       = random(-127, 128);
  gp.z       = random(-127, 128);
  gp.rz      = random(-127, 128);
  gp.rx      = random(-127, 128);
  gp.ry      = random(-127, 128);
  gp.hat     = random(0,      9);
  gp.buttons = random(0, 0xffff);
  blegamepad.report(&gp);
  delay(1000);
}
