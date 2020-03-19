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

#if defined(ARDUINO_NRF52840_CIRCUITPLAY) || defined(ARDUINO_NRF52840_CLUE)
  #define USE_ARCADA
#endif

#ifdef USE_ARCADA
#include <Adafruit_Arcada.h>

Adafruit_Arcada arcada;
Adafruit_SPITFT* tft;
#endif

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_nRFCrypto.h>

/* This sketch demonstrates Pairing process using dynamic Passkey.
 * This sketch is essentially the same as bleuart.ino except the BLE Uart
 * service requires Security Mode with Man-In-The-Middle protection i.e
 *
 * BLE Pairing procedure is complicated, it is advisable for users to go through
 * these articles to get familiar with the procedure and terminology
 * - https://www.bluetooth.com/blog/bluetooth-pairing-part-1-pairing-feature-exchange/
 * - https://www.bluetooth.com/blog/bluetooth-pairing-part-2-key-generation-methods/
 * - https://www.bluetooth.com/blog/bluetooth-pairing-passkey-entry/
 * - https://www.bluetooth.com/blog/bluetooth-pairing-part-4/
 */

/* This sketch demonstrates the "Image Upload" feature of Bluefruit Mobile App.
 * Following TFT Display are supported
 *  - Circuit Playground Bluefruit: https://www.adafruit.com/product/4333
 *  - TFT Gizmo : https://www.adafruit.com/product/4367
 *  - Adafruit CLUE : https://www.adafruit.com/product/4500
 */

// BLE Service
BLEUart bleuart; // uart over ble

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Pairing Display Example");
  Serial.println("-----------------------------------\n");

  arcada.arcadaBegin();
  arcada.displayBegin();
  arcada.setBacklight(255);

  tft = arcada.display;
  tft->setCursor(0, 0);
  tft->setTextWrap(true);
  tft->setTextSize(2);

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

  // clear bonds if BUTTON A is pressed
  Serial.println("Hold button A to clear bonds ..... ");
  delay(2000);
  if (0 == digitalRead(PIN_BUTTON1))
  {
    Serial.println("Clear all central bonds");
    Bluefruit.Periph.clearBonds();
  }

  // To use dynamic PassKey for pairing, we need to have
  // - IO capacities at least DISPPLAY
  // - Register callback to display/print dynamic passkey for central
  // For complete mapping of the IO Capabilities to Key Generation Method, check out this article
  // https://www.bluetooth.com/blog/bluetooth-pairing-part-2-key-generation-methods/
  Bluefruit.Pairing.setIOCaps(true, true, false); // display = true, yes/no = true, keyboard = false
  Bluefruit.Pairing.setPasskeyCallback(pairing_passkey_callback);

  // Set complete callback to print the pairing result
  Bluefruit.Pairing.setCompleteCallback(pairing_complete_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Pairing.setSecuredCallback(connection_secured_callback);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start BLE Uart Service
  // Set Permission to access BLE Uart is to require man-in-the-middle protection
  // This will cause central to perform pairing with static PIN we set above
  Serial.println("Configure BLE Uart to require man-in-the-middle protection for PIN pairing");
  bleuart.setPermission(SECMODE_ENC_WITH_MITM);
  bleuart.begin();

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->print("Advertising...");
#endif

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Your phone should pop-up PIN input");
  Serial.println("Once connected, enter character(s) that you wish to send");

  // Set up and start advertising
  startAdv();
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


// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Connected");
#endif
}

// callback invoked when pairing passkey is generated
// - passkey: 6 keys (without null terminator) for displaying
// - match_request: true when authentication method is Numberic Comparison.
//                  Then this callback's return value is used to accept (true) or
//                  reject (false) the pairing process. Otherwise, return value has no effect
bool pairing_passkey_callback(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
  Serial.println("Pairing Passkey");
  Serial.printf("    %.3s %.3s\n", passkey, passkey+3);

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->println("Pairing Passkey\n");
  tft->setTextColor(ARCADA_YELLOW);
  tft->setTextSize(4);
  tft->printf("  %.3s %.3s\n", passkey, passkey+3);

  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
#endif


  if (match_request)
  {
    Serial.println("Do you want to pair");
    Serial.println("Press Button A to accept, Button B to reject");

    #ifdef USE_ARCADA
    tft->println("\nDo you accept ?\n\n");

    tft->setTextSize(3);
    tft->setTextColor(ARCADA_GREEN);
    tft->print("<< Yes");
    tft->setTextColor(ARCADA_RED);
    tft->println("  No >>");

    tft->setTextColor(ARCADA_WHITE);
    tft->setTextSize(2);
    tft->println();
    #endif

    // wait until either button is pressed
    while( digitalRead(PIN_BUTTON1) && digitalRead(PIN_BUTTON2) ) { }

    // wait until either button is pressed
    uint32_t start_time = millis();
    while( digitalRead(PIN_BUTTON1) && digitalRead(PIN_BUTTON2) )
    {
      // 30 seconds timeout
      if ( millis() > start_time + 30000 ) break;
    }

    // A = accept
    if ( 0 == digitalRead(PIN_BUTTON1) ) return true;

    // B = reject
    if ( 0 == digitalRead(PIN_BUTTON2) ) return false;

    return false;
  }

  return true;
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

  #ifdef USE_ARCADA
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    tft->setTextColor(ARCADA_GREEN);
    tft->println("Succeeded");
  }else
  {
    tft->setTextColor(ARCADA_RED);
    tft->println("Failed");
  }

  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
  #endif
}

void connection_secured_callback(uint16_t conn_handle)
{
  Serial.println("Secured");

  #ifdef USE_ARCADA
  tft->setTextColor(ARCADA_YELLOW);
  tft->println("secured");
  tft->setTextColor(ARCADA_WHITE);
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

#ifdef USE_ARCADA
  tft->println("Advertising ...");
#endif
}
