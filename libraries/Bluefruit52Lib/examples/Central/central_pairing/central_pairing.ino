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

#define TFT_NO_DISPLAY      0
#define TFT_GIZMO           1 // used with Circuit Playground Bluefruit
#define TFT_CLUE            2 // CLUE's on-board display
#define TFT_24_FEATHERWING  3
#define TFT_35_FEATHERWING  4


#if defined(ARDUINO_NRF52840_CIRCUITPLAY)
  // Circuit Playground Bluefruit use with TFT GIZMO
  #define TFT_IN_USE  TFT_GIZMO
  #define DEVICE_NAME   "CPLAY"

  #include "Adafruit_ST7789.h"
  Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, 0, 1, -1); // CS = 0, DC = 1

#elif defined(ARDUINO_NRF52840_CLUE)
  // CLUE use on-board TFT
  #define TFT_IN_USE  TFT_CLUE
  #define DEVICE_NAME   "CLUE"

  #include "Adafruit_ST7789.h"
  Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

#else
  // [Configurable] For other boards please select which external display to match your hardware setup
  #define TFT_IN_USE     TFT_24_FEATHERWING
  #define DEVICE_NAME   "Feather"

  #if defined(ARDUINO_NRF52832_FEATHER)
    // Feather nRF52832 pin map is different from others
    #define TFT_DC   11
    #define TFT_CS   31
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
  #endif // TFT

#endif // board variants


#define COLOR_WHITE     0xFFFF
#define COLOR_BLACK     0x0000
#define COLOR_YELLOW    0xFFE0
#define COLOR_GREEN     0x07E0
#define COLOR_RED       0xF800

BLEClientUart clientUart; // bleuart client

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central Pairing Example");
  Serial.println("-----------------------------------\n");
  
#if TFT_IN_USE == TFT_CLUE
  tft.init(240, 240);
  tft.setRotation(1);

  // Screen refresh rate control (datasheet 9.2.18, FRCTRL2)
  uint8_t rtna = 0x01;
  tft.sendCommand(0xC6, &rtna, 1);;

  // turn back light on
  uint8_t backlight = PIN_TFT_LITE;
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);

#elif TFT_IN_USE == TFT_GIZMO
  tft.init(240, 240);
  tft.setRotation(2);

  // turn back light on
  uint8_t backlight = A3;
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);

#elif TFT_IN_USE != TFT_NO_DISPLAY
  tft.begin();

#endif // TFT

  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setName("Bluefruit52 Central");
  
  // clear bonds if BUTTON A is pressed
  Serial.println("Hold button A to clear bonds ..... ");
  delay(2000);
  if (0 == digitalRead(PIN_BUTTON1))
  {
    Serial.println("Clear all central bonds");
    Bluefruit.Central.clearBonds();
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

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

#if TFT_IN_USE != TFT_NO_DISPLAY
  tft.fillScreen(COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.print("Scanning...");
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

#if TFT_IN_USE != TFT_NO_DISPLAY
  tft.println("connected");
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
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  Serial.print("[RX]: ");
  
  while ( uart_svc.available() )
  {
    Serial.print( (char) uart_svc.read() );
  }

  Serial.println();
}

bool pairing_passkey_callback(uint16_t conn_handle, uint8_t const passkey[6], bool match_request)
{
  Serial.println("Pairing Passkey");
  Serial.printf("    %.3s %.3s\n\n", passkey, passkey+3);

#if TFT_IN_USE != TFT_NO_DISPLAY
  tft.fillScreen(COLOR_BLACK);
  tft.println("Pairing Passkey\n");
  tft.setTextColor(COLOR_YELLOW);
  tft.setTextSize(4);
  tft.printf("  %.3s %.3s\n", passkey, passkey+3);

  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
#endif

  if (match_request)
  {
    Serial.println("Do you want to pair ?");
    Serial.println("Press Button A to accept, Button B to reject");

    #if TFT_IN_USE != TFT_NO_DISPLAY
    tft.println("\nDo you accept ?\n\n");

    tft.setTextSize(3);
    tft.setTextColor(COLOR_GREEN);
    tft.print("<< Yes");
    tft.setTextColor(COLOR_RED);
    tft.println("  No >>");

    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.println();
    #endif

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

#if TFT_IN_USE != TFT_NO_DISPLAY
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
  {
    tft.setTextColor(COLOR_GREEN);
    tft.print("Succeeded ");
  }else
  {
    tft.setTextColor(COLOR_RED);
    tft.print("Failed ");
  }

  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
#endif
}

void connection_secured_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if ( !conn->secured() )
  {
    // It is possible that connection is still not secured by this time.
    // This happens when we try to encrypt connection using stored bond keys
    // but peer reject it (probably it remove its stored key).
    // Therefore we will request an pairing again --> callback again when encrypted
    conn->requestPairing();
  }
  else
  {
    Serial.println(" Secured");

    #if TFT_IN_USE != TFT_NO_DISPLAY
    tft.setTextColor(COLOR_YELLOW);
    tft.println("secured");
    tft.setTextColor(COLOR_WHITE);
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
      Bluefruit.disconnect(conn_handle);
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
