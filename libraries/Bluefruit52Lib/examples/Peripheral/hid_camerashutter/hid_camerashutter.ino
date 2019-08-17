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

/*
 * This sketch uses the HID Consumer Key API to send the Volume Down
 * key when PIN_SHUTTER is grounded. This will cause your mobile device
 * to capture a photo when you are in the camera app.
 *
 * For Feather nRF52840 PIN_SHUTTER is conveniently user switch.
 */
#include <bluefruit.h>

BLEDis bledis;
BLEHidAdafruit blehid;

#define PIN_SHUTTER   7

void setup()
{
  pinMode(PIN_SHUTTER, INPUT_PULLUP);

  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 HID Camera Shutter Example");
  Serial.println("--------------------------------------\n");

  Serial.println();
  Serial.println("Go to your phone's Bluetooth settings to pair your device");
  Serial.println("then open the camera application");

  Serial.println();
  Serial.printf("Set pin %d to GND to capture a photo\n", PIN_SHUTTER);
  Serial.println();

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Configure and start DIS (Device Information Service)
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE devices to have a min connection interval >= 20m
   * (The smaller the connection interval the faster we can send data).
   * However, for HID and MIDI device Apple will accept a min connection
   * interval as low as 11.25 ms. Therefore BLEHidAdafruit::begin() will try to
   * set the min and max connection interval to 11.25 ms and 15 ms respectively
   * for the best performance.
   */
  blehid.begin();

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
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

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
  // Skip if shutter pin is not Ground
  if ( digitalRead(PIN_SHUTTER) == 1 ) return;

  // Make sure we are connected and bonded/paired
  for (uint16_t conn_hdl=0; conn_hdl < BLE_MAX_CONNECTION; conn_hdl++)
  {
    BLEConnection* connection = Bluefruit.Connection(conn_hdl);

    if ( connection && connection->connected() && connection->paired() )
    {
      // Turn on red LED when we start sending data
      digitalWrite(LED_RED, 1);

      // Send the 'volume down' key press to the peer
      // Check tinyusb/src/class/hid/hid.h for a list of valid consumer usage codes
      blehid.consumerKeyPress(conn_hdl, HID_USAGE_CONSUMER_VOLUME_DECREMENT);

      // Delay a bit between reports
      delay(10);

      // Send key release
      blehid.consumerKeyRelease(conn_hdl);

      // Turn off the red LED
      digitalWrite(LED_RED, 0);
    }
  }

  // Delay to avoid constant capturing
  delay(500);
}
