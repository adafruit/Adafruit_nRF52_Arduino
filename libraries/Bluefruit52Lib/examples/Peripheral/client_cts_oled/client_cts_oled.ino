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

/* This sketch demonstrates the client Current Time Service using the
 * BLEClientCts API(). After uploading, go to iOS setting and connect
 * to Bluefruit52, and then press PAIR.
 * 
 * Note: Currently only iOS act as a CTS server, Android does not. The
 * easiest way to test this sketch is using an iOS device.
 * 
 * This sketch is similar to client_cts but also uses a Feather OLED Wing
 * to display time https://www.adafruit.com/product/2900
 *
 * Current Time Service info:
 *   https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.current_time.xml
 *   https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.current_time.xml
 *   https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.local_time_information.xml
 */

#include <bluefruit.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 oled(OLED_RESET);

// BLE Client Current Time Service
BLEClientCts  bleCTime;

void setup()
{
  // init with the I2C addr 0x3C (for the 128x32) and show splashscreen
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();

  oled.setTextSize(1);// max is 4 line, 21 chars each
  oled.setTextColor(WHITE);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);

  // Configure CTS client
  bleCTime.begin();

  // Set up and start advertising
  startAdv();

  // splash screen effect
  delay(100);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Waiting to connect");
  oled.display();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_GENERIC_CLOCK);

  // Include CTS client UUID
  Bluefruit.Advertising.addService(bleCTime);

  // Includes name
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
  // This example only support 1 connection
  uint16_t const conn_handle = 0;
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // connection exist, connected, and secured
  if ( !(conn && conn->connected() && conn->secured()) ) return;

  // Skip if service is not yet discovered
  if ( !bleCTime.discovered() ) return;

  // Get Time from iOS once per second
  // Note it is not advised to update this quickly
  // Application should use local clock and update time after 
  // a long period (e.g an hour or day)
  bleCTime.getCurrentTime();
  bleCTime.getLocalTimeInfo();

  printTime();

  delay(1000);
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Connected.");
  oled.print("Discovering ... ");
  oled.display();

  if ( bleCTime.discover(conn_handle) )
  {
    oled.println("OK");

    // Current Time Service requires pairing to work
    // request Pairing if not bonded
    oled.print("Paring      ... ");
    oled.display();

    conn->requestPairing();
  }
}

void connection_secured_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if ( !conn->secured() )
  {
    // It is possible that connection is still not secured by this time.
    // This happens (central only) when we try to encrypt connection using stored bond keys
    // but peer reject it (probably it remove its stored key).
    // Therefore we will request an pairing again --> callback again when encrypted
    conn->requestPairing();
  }
  else
  {
    oled.println("Secured");

    if ( bleCTime.discovered() )
    {
      bleCTime.enableAdjust();

      oled.println("Receiving Time...");
      oled.display();

      bleCTime.getCurrentTime();
      bleCTime.getLocalTimeInfo();
    }
  }
}

void printTime(void)
{
  const char * day_of_week_str[] = { "n/a", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
  const char * month_str[] = { "na", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
  
  oled.clearDisplay();
  oled.setCursor(0, 0);

  oled.print(day_of_week_str[bleCTime.Time.weekday]);
  oled.printf(" %s %02d %04d\n", month_str[bleCTime.Time.month], bleCTime.Time.day, bleCTime.Time.year);
  

  oled.setTextSize(2);
  oled.printf(" %02d:%02d:%02d\n", bleCTime.Time.hour, bleCTime.Time.minute, bleCTime.Time.second);

  int utc_offset =  bleCTime.LocalInfo.timezone*15; // in 15 minutes unit
  
  oled.setTextSize(1);
  oled.printf("UTC %+d:%02d, ", utc_offset/60, utc_offset%60);
  oled.printf("DST %+.1f", ((float) bleCTime.LocalInfo.dst_offset*15)/60 );
  oled.println();

  oled.display();
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Waiting to connect");
  oled.display();
}
