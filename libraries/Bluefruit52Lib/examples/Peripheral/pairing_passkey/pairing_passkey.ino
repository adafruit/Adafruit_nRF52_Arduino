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
 *
 * IF TFT enabled board such as CLUE is used, the passkey will also display on the
 * TFT. Following boards with TFT are supported
 *  - Adafruit CLUE : https://www.adafruit.com/product/4500
 *  - Circuit Playground Bluefruit: https://www.adafruit.com/product/4333
 *  - TFT Gizmo : https://www.adafruit.com/product/4367
 */

#if defined(ARDUINO_NRF52840_CIRCUITPLAY) || defined(ARDUINO_NRF52840_CLUE)
  #define USE_ARCADA
#endif

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#ifdef USE_ARCADA
  #include <Adafruit_Arcada.h>

  Adafruit_Arcada arcada;
  Adafruit_SPITFT* tft;

#else
  // Use built-in buttons if available, else use A0, A1
  #ifdef PIN_BUTTON1
    #define BUTTON_YES    PIN_BUTTON1
  #else
    #define BUTTON_YES    A0
  #endif

  #ifdef PIN_BUTTON2
    #define BUTTON_NO   PIN_BUTTON2
  #else
    #define BUTTON_NO   A1
  #endif
#endif

// BLE Service
BLEUart bleuart; // uart over ble

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Pairing Display Example");
  Serial.println("-----------------------------------\n");

#ifdef USE_ARCADA
  arcada.arcadaBegin();
  arcada.displayBegin();
  arcada.setBacklight(255);

  tft = arcada.display;
  tft->setCursor(0, 0);
  tft->setTextWrap(true);
  tft->setTextSize(2);
#else
  pinMode(BUTTON_YES, INPUT_PULLUP);
  pinMode(BUTTON_NO, INPUT_PULLUP);
#endif

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

  /* To use dynamic PassKey for pairing, we need to have
   * - IO capacities at least DISPPLAY
   *    - Display only: user have to enter 6-digit passkey on their phone
   *    - DIsplay + Yes/No: user ony need to press Accept on both central and device
   * - Register callback to display/print dynamic passkey for central
   *
   * For complete mapping of the IO Capabilities to Key Generation Method, check out this article
   * https://www.bluetooth.com/blog/bluetooth-pairing-part-2-key-generation-methods/
   */
  Bluefruit.Security.setIOCaps(true, true, false); // display = true, yes/no = true, keyboard = false
  Bluefruit.Security.setPairPasskeyCallback(pairing_passkey_callback);

  // Set complete callback to print the pairing result
  Bluefruit.Security.setPairCompleteCallback(pairing_complete_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start BLE Uart Service
  // Set Permission to access BLE Uart is to require man-in-the-middle protection
  // This will cause central to perform pairing with a generated passkey, the passkey will
  // be printed on display or Serial and wait for our input
  Serial.println("Configure BLE Uart to require man-in-the-middle protection for PIN pairing");
  bleuart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
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

  // match_request means peer wait for our approval (return true)
  if (match_request)
  {
    Serial.println("Do you want to pair");
    Serial.println("Press Button Left to decline, Button Right to Accept");

    // timeout for pressing button
    uint32_t start_time = millis();

#ifdef USE_ARCADA
    tft->println("\nDo you accept ?\n\n");
    tft->setTextSize(3);

    // Yes <-> No on CPB is reversed since GIZMO TFT is on the back of CPB
    #if ARDUINO_NRF52840_CIRCUITPLAY
      tft->setTextColor(ARCADA_GREEN);
      tft->print("< Yes");
      tft->setTextColor(ARCADA_RED);
      tft->println("    No >");
    #else
      tft->setTextColor(ARCADA_RED);
      tft->print("< No");
      tft->setTextColor(ARCADA_GREEN);
      tft->println("    Yes >");
    #endif

    tft->setTextColor(ARCADA_WHITE);
    tft->setTextSize(2);
    tft->println();

    // wait until either button is pressed (30 seconds timeout)
    uint32_t justReleased;
    do
    {
      // Peer is disconnected while waiting for input
      if ( !Bluefruit.connected(conn_handle) ) break;

      // time out
      if ( millis() > start_time + 30000 ) break;

      arcada.readButtons();
      justReleased = arcada.justReleasedButtons();
    } while ( !(justReleased & (ARCADA_BUTTONMASK_LEFT | ARCADA_BUTTONMASK_RIGHT) ) );

    // Right = accept
    if (justReleased & ARCADA_BUTTONMASK_RIGHT) return true;

    // Left = decline
    if (justReleased & ARCADA_BUTTONMASK_LEFT) return false;

#else
    // wait until either button is pressed (30 seconds timeout)
    while( digitalRead(BUTTON_YES) && digitalRead(BUTTON_NO) )
    {
      // Peer is disconnected while waiting for input
      if ( !Bluefruit.connected(conn_handle) ) break;

      // time out
      if ( millis() > start_time + 30000 ) break;
    }

    if ( 0 == digitalRead(BUTTON_YES) ) return true;

    if ( 0 == digitalRead(BUTTON_NO) ) return false;
#endif

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
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Advertising ...");
#endif
}
