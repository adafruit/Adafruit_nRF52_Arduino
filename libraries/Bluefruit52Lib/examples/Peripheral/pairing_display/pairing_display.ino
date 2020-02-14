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

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_nRFCrypto.h>

/* This sketch demonstrates Pairing process using static Passkey aka PIN.
 * This sketch is essentially the same as bleuart.ino except the BLE Uart
 * service requires Security Mode with Man-In-The-Middle protection i.e
 * using 6 digits PIN for pairing.
 */

/* This sketch demonstrates the "Image Upload" feature of Bluefruit Mobile App.
 * Following TFT Display are supported
 *  - https://www.adafruit.com/product/3315
 *  - https://www.adafruit.com/product/3651
 *  - https://www.adafruit.com/product/4367
 */

#define TFT_NONE            0
#define TFT_35_FEATHERWING  1
#define TFT_24_FEATHERWING  2
#define TFT_GIZMO           3

// [Configurable] Please select one of above supported Display to match your hardware setup
#define TFT_IN_USE          TFT_NONE


#if defined(ARDUINO_NRF52832_FEATHER)
  // Feather nRF52832
  #define TFT_DC   11
  #define TFT_CS   31

#elif defined(ARDUINO_NRF52840_CIRCUITPLAY)
  // Circuit Playground Bluefruit for use with TFT 1.5" GIZMO
  #define TFT_DC         1
  #define TFT_CS         0
  #define TFT_BACKLIGHT  A3

#else
  // Default for others
  #define TFT_DC   10
  #define TFT_CS   9
#endif


#if   TFT_IN_USE == TFT_35_FEATHERWING
  #include "Adafruit_HX8357.h"
  Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC);

#elif TFT_IN_USE == TFT_24_FEATHERWING
  #include <Adafruit_ILI9341.h>
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#elif TFT_IN_USE == TFT_GIZMO
  #include "Adafruit_ST7789.h"
  Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, TFT_CS, TFT_DC, -1);

#endif

#define COLOR_WHITE     0xFFFF
#define COLOR_BLACK     0x0000
#define COLOR_YELLOW    0xFFE0
#define COLOR_GREEN     0x07E0
#define COLOR_RED       0xF800

// BLE Service
BLEUart bleuart; // uart over ble

void setup()
{
  Serial.begin(115200);
  while(!Serial) delay(1);

#if TFT_IN_USE == TFT_GIZMO
  tft.init(240, 240);
  tft.setRotation(2);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on

#elif TFT_IN_USE != TFT_NONE
  tft.begin();

#endif

  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

//  Serial.println("Setting pairing PIN to: " PAIRING_PIN);
  Bluefruit.Pairing.setDisplayCallback(pairing_display_callback);
  Bluefruit.Pairing.setCompleteCallback(pairing_complete_callback);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start BLE Uart Service
  // Set Permission to access BLE Uart is to require man-in-the-middle protection
  // This will cause central to perform pairing with static PIN we set above
  Serial.println("Configure BLE Uart to require man-in-the-middle protection for PIN pairing");
  bleuart.setPermission(SECMODE_ENC_WITH_MITM);
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Your phone should pop-up PIN input");
  Serial.println("Once connected, enter character(s) that you wish to send");

#if TFT_IN_USE != TFT_NONE
  tft.fillScreen(COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.println("Advertising ... ");
#endif
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
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
  // Forward data from HW Serial to BLEUART
  while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write( buf, count );
  }

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }
}

void pairing_display_callback(uint16_t conn_handle, uint8_t const passkey[6])
{
  Serial.println("Enter this code on your phone to pair with Bluefruit:");
  Serial.printf("    %.6s\n", passkey);

#if TFT_IN_USE != TFT_NONE
  tft.printf("Enter this code on your phone to pair with Bluefruit:\n\n");
  tft.setTextColor(COLOR_YELLOW);
  tft.setTextSize(4);
  tft.printf("  %.6s\n", passkey);

  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
#endif
}

void pairing_complete_callback(uint16_t conn_handle, uint8_t auth_status)
{
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    Serial.println("Succeeded");
  }else
  {
    Serial.println("Failed");
  }

#if TFT_IN_USE != TFT_NONE
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    tft.setTextColor(COLOR_GREEN);
    tft.println("Succeeded");
  }else
  {
    tft.setTextColor(COLOR_RED);
    tft.println("Failed");
  }

  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
#endif
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

#if TFT_IN_USE != TFT_NONE
  tft.fillScreen(COLOR_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Connected");
#endif
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

#if TFT_IN_USE != TFT_NONE
  tft.println("Advertising ...");
#endif
}
