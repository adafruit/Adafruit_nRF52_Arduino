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

/* This sketch demonstrates Pairing process using dynamic Passkey.
 * This sketch is essentially the same as central_bleuart.ino except the BLE Uart
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
 * tft-> Following boards with TFT are supported
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

BLEClientUart clientUart; // bleuart client

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central Pairing Example");
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

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  // clear bonds if BUTTON A is pressed
//  Serial.println("Hold button A to clear bonds ..... ");
//  Serial.flush();
//  delay(2000);
//  if (0 == digitalRead(PIN_BUTTON1))
//  {
//    Serial.println("Clear all central bonds");
//    Bluefruit.Central.clearBonds();
//  }

  // To use dynamic PassKey for pairing, we need to have
  // - IO capacities at least DISPPLAY
  // - Register callback to display/print dynamic passkey for central
  // For complete mapping of the IO Capabilities to Key Generation Method, check out this article
  // https://www.bluetooth.com/blog/bluetooth-pairing-part-2-key-generation-methods/
  Bluefruit.Security.setIOCaps(true, true, false); // display = true, yes/no = true, keyboard = false
  Bluefruit.Security.setPairPasskeyCallback(pairing_passkey_callback);

  // Set complete callback to print the pairing result
  Bluefruit.Security.setPairCompleteCallback(pairing_complete_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Scanning ...");
#endif

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }else
  {      
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  Serial.println("Connected");

#ifdef USE_ARCADA
  tft->fillScreen(ARCADA_BLACK);
  tft->setTextSize(2);
  tft->setCursor(0, 0);
  tft->println("Connected");
#endif

  // If we are not bonded with peer previously -> send pairing request
  // Else wait for the connection secured callback
  if ( !conn->bonded() )
  {
    conn->requestPairing();
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

#ifdef USE_ARCADA
  tft->println("Scanning ...");
#endif
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read() );
  }
}

bool pairing_passkey_callback(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
  Serial.println("Pairing Passkey");
  Serial.printf("    %.3s %.3s\n\n", passkey, passkey+3);

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
    uint32_t justReleased = 0;
    do
    {
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
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    Serial.println("Succeeded");
  }else
  {
    Serial.println("Failed");

    // disconnect
    conn->disconnect();
  }

#ifdef USE_ARCADA
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    tft->setTextColor(ARCADA_GREEN);
    tft->print("Succeeded ");
  }else
  {
    tft->setTextColor(ARCADA_RED);
    tft->print("Failed ");
  }

  tft->setTextColor(ARCADA_WHITE);
  tft->setTextSize(2);
#endif
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
    Serial.println("Secured");

    #ifdef USE_ARCADA
    tft->setTextColor(ARCADA_YELLOW);
    tft->println("secured");
    tft->setTextColor(ARCADA_WHITE);
    #endif

    Serial.print("Discovering BLE Uart Service ... ");
    if ( clientUart.discover(conn_handle) )
    {
      Serial.println("Found it");

      Serial.println("Enable TXD's notify");
      clientUart.enableTXD();

      Serial.println("Ready to receive from peripheral");
    }else
    {
      Serial.println("Found NONE");

      // disconnect since we couldn't find bleuart service
      conn->disconnect();
    }
  }
}

void loop()
{
  uint16_t const conn_handle = 0; // this example only support 1 connection
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // skip if connection not exist or not connected
  if ( !conn && conn->connected() ) return;

  // In this example we only read & write when connection is secured
  if ( conn->secured() )
  {
    // Not discovered yet
    if ( clientUart.discovered() )
    {
      // Discovered means in working state
      // Get Serial input and send to Peripheral
      if ( Serial.available() )
      {
        delay(2); // delay a bit for all characters to arrive

        char str[20+1] = { 0 };
        Serial.readBytes(str, 20);

        clientUart.print( str );
      }
    }
  }
}
