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
#include <BLEHomekit.h>

BLEHomekit homekit;
HAPAccessoryInfo hap_info;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Homekit Light Bulb Example");
  Serial.println("--------------------------\n");

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit52");

  hap_info.begin();

  // Set up and start advertising
  startAdv();
  
  dbgMemInfo();
}

void startAdv(void)
{  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 30
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.setStopCallback(adv_stop_callback); // callback when adv is stopped

  /* Homekit include advertising interval as part of its adv data
   * - Therefore setInterval() must be call before setData(homekit).
   * - Furthermore since Adverting Data (for homekit) changes when adv interval change
   * from fast to slow. We do either way to stay consistence
   *    1. Advertise with fixed interval (fast = slow)
   *    2. Register stop callback for adv stop, then re-set the Advertising data to match
   *    the changed interval. Which is what this sketch does.
   */
  Bluefruit.Advertising.setData(homekit);

  // Stop after 30 secconds
  Bluefruit.Advertising.start(30);
}

void adv_stop_callback(void)
{
  // change interval to only slow since we alredy passed first 30 seconds
  Bluefruit.Advertising.setInterval(244, 244);

  // Reset advertising data for homekit.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.setData(homekit);

  // Advertising indefintely.
  Bluefruit.Advertising.start(0);
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_RED);
  delay(1000);
}

