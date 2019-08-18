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

/* This sketch demonstrates the "Image Upload" feature of Bluefruit Mobile App.
 * FeatherWing OLED is used to display uploaded image
 *  - https://www.adafruit.com/product/3315
 */

#include <bluefruit.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

BLEUart bleuart; // uart over ble

/* The Image Transfer module sends the image of your choice to Bluefruit LE over UART.
 * Each image sent begins with
 * - A single byte char “!” (0x21)
 * - Image width (uint16 little endian, 2 bytes)
 * - Image height (uint16 little endian, 2 bytes)
 * - Pixel data encoded as RGB 24-bit and suffixed by a single byte CRC.
 *
 * Format: [ ‘!’ ] [ uint16 width ] [ uint16 height ] [ r g b ] [ r g b ] [ r g b ] … [ CRC ]
 */

uint16_t imageWidth = 0;
uint16_t imageHeight = 0;

// Statistics for speed testing
uint32_t rxCount = 0;
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;

#ifdef ARDUINO_NRF52832_FEATHER
   #define TFT_DC   11
   #define TFT_CS   31
#endif

#ifdef ARDUINO_NRF52840_FEATHER
   #define TFT_DC   10
   #define TFT_CS   9
#endif

#ifdef ARDUINO_NRF52840_CIRCUITPLAY
   #define TFT_DC   A7
   #define TFT_CS   A6
#endif


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup()
{
  Serial.begin(115200);

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12); // 7.5 - 15 ms

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(bleuart_rx_callback);

  // Set up and start advertising
  startAdv();

  // splash screen effect
  delay(100);


  tft.println("Advertising ... ");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_GENERIC_CLOCK);

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
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
  // 3 seconds has passed and there is no data received
  // then reset rx count
  if ( (rxCount > 0) && (rxLastTime + 1000 < millis()) )
  {
    print_speed(rxCount, rxLastTime-rxStartTime);
    rxCount = 0;
  }
}

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  tft.println("Connected");

  conn->requestPHY();
  tft.println("Switching PHY");

  conn->requestDataLengthUpdate();
  tft.println("Updating Data Length");

  conn->requestMtuExchange(247);
  tft.println("Exchanging MTU");

  tft.println("Ready to receive Image");
}

void print_speed(uint32_t count, uint32_t ms)
{
  Serial.print("Received ");
  Serial.print(count);
  Serial.print(" bytes in ");
  Serial.print(ms / 1000.0F, 2);
  Serial.println(" seconds.");

  Serial.print("Speed : ");
  Serial.print( (count / 1000.0F) / (ms / 1000.0F), 2);
  Serial.println(" KB/s.\r\n");
}

void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;

  rxLastTime = millis();

  // first packet
  if ( rxCount == 0 )
  {
    rxStartTime = millis();

    // Incorrect format, possibly corrupted data
    if ( bleuart.read() != '!' ) bleuart.flush();
    rxCount++;

    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
  }

  rxCount += bleuart.available();
  bleuart.flush(); // empty rx fifo
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;

  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.println("Advertising ...");
}