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
#include <SPI.h>
#include <Adafruit_GFX.h>

/* This sketch demonstrates the "Image Upload" feature of Bluefruit Mobile App.
 * Following TFT Display are supported
 *  - TFT 3.5" : FeatherWing https://www.adafruit.com/product/3651
 *  - TFT 2.4" : FeatherWing https://www.adafruit.com/product/3315
 *  - TFT Gizmo : https://www.adafruit.com/product/4367
 *  - Adafruit CLUE : https://www.adafruit.com/product/4500
 */

#if defined(ARDUINO_NRF52840_CIRCUITPLAY)
  // Circuit Playground Bluefruit use with TFT GIZMO
  #define DEVICE_NAME   "CPLAY"

  #include "Adafruit_ST7789.h"
  Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, 0, 1, -1); // CS = 0, DC = 1

#elif defined(ARDUINO_NRF52840_CLUE)
  #define DEVICE_NAME   "CLUE"

  // CLUE use on-board TFT
  #include "Adafruit_ST7789.h"
  Adafruit_ST7789 tft = Adafruit_ST7789(&SPI1, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

#else
  #define DEVICE_NAME   "Feather"

  #define TFT_35_FEATHERWING  1
  #define TFT_24_FEATHERWING  2

  // [Configurable] For other boards please select which external display to match your hardware setup
  #define TFT_IN_USE     TFT_24_FEATHERWING

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

  #else
    #error "TFT display is not supported"
  #endif // TFT

#endif // board variants

// Universal color
#define COLOR_WHITE     0xFFFF
#define COLOR_BLACK     0x0000
#define COLOR_YELLOW    0xFFE0
#define COLOR_GREEN     0x07E0
#define COLOR_RED       0xF800

// Declaring Uart over BLE with large buffer to hold image data
// Depending on the Image Resolution and Transfer Mode especially without response
// or Interleaved with high ratio. You may need to increase this buffer size
BLEUart bleuart(10*1024);

/* The Image Transfer module sends the image of your choice to Bluefruit LE over UART.
 * Each image sent begins with
 * - A single byte char '!' (0x21) followed by 'I' helper for image
 * - Color depth: 24-bit for RGB 888, 16-bit for RGB 565
 * - Image width (uint16 little endian, 2 bytes)
 * - Image height (uint16 little endian, 2 bytes)
 * - Pixel data encoded as RGB 16/24 bit and suffixed by a single byte CRC.
 *
 * Format: [ '!' ] [ 'I' ] [uint8_t color bit] [ uint16 width ] [ uint16 height ] [ r g b ] [ r g b ] [ r g b ] … [ CRC ]
 */

uint16_t imageWidth = 0;
uint16_t imageHeight = 0;
uint8_t  imageColorBit = 0;

uint32_t totalPixel = 0; // received pixel

// pixel line buffer, should be large enough to hold an image width
uint16_t pixel_buf[512];

// Statistics for speed testing
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;

// for print out message to TFT once
bool bleuart_overflowed = false;

void setup()
{
  Serial.begin(115200);

#if defined(ARDUINO_NRF52840_CIRCUITPLAY)
  tft.init(240, 240);
  tft.setRotation(2);

  // turn back light on
  uint8_t backlight = A3;
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);

#elif defined(ARDUINO_NRF52840_CLUE)
  tft.init(240, 240);
  tft.setRotation(1);

  // Screen refresh rate control (datasheet 9.2.18, FRCTRL2)
  uint8_t rtna = 0x01;
  tft.sendCommand(0xC6, &rtna, 1);;

  // turn back light on
  uint8_t backlight = PIN_TFT_LITE;
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);

#else
  tft.begin();

#endif
  
  tft.fillScreen(COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(1);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12); // 7.5 - 15 ms

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Due to huge amount of image data
  // NRF52832 doesn't have enough SRAM to queue up received packets using deferred callbacks.
  // Therefore it must process data as soon as it comes, this can be done by
  // changing the default "deferred" option to false to invoke callback immediately.
  // However, the transfer speed will be affected since immediate callback will block BLE task
  // to process data especially when tft.drawRGBBitmap() is calling.
#ifdef NRF52840_XXAA
  // 2nd argument is true to deferred callbacks i.e queue it up in separated callback Task
  bleuart.setRxCallback(bleuart_rx_callback, true);
#else
  // 2nd argument is false to invoke callbacks immediately (thus blocking other ble events)
  bleuart.setRxCallback(bleuart_rx_callback, false);
#endif

  bleuart.setRxOverflowCallback(bleuart_overflow_callback);

  // Set up and start advertising
  startAdv();

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
  // nothing to do
}

// Invoked when receiving data from bleuart
// Pull data from bleuart fifo & draw image as soon as possible,
// Otherwise bleuart fifo can be overflowed
void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;

  rxLastTime = millis();

  // Received new Image
  if ( (imageWidth == 0) && (imageHeight == 0) )
  {
    // take note of time of first packet
    rxStartTime = millis();

    // Skip all data until '!I' is found
    while( bleuart.available() && bleuart.read() != '!' )  { }
    if (bleuart.read() != 'I') return;

    if ( !bleuart.available() ) return;

    imageColorBit = bleuart.read8();
    imageWidth  = bleuart.read16();
    imageHeight = bleuart.read16();

    totalPixel = 0;

    tft.fillScreen(COLOR_BLACK);
    tft.setCursor(0, 0);

    // Print out the current connection info
    BLEConnection* conn = Bluefruit.Connection(conn_hdl);
    Serial.printf("Connection Info: PHY = %d Mbps, Conn Interval = %.2f ms, Data Length = %d, MTU = %d\n",
                  conn->getPHY(), conn->getConnectionInterval()*1.25f, conn->getDataLength(), conn->getMtu());

    Serial.printf("Receiving an %dx%d Image with %d-bit color\n", imageWidth, imageHeight, imageColorBit);
  }

  // Extract pixel data to buffer and draw image line by line
  while ( bleuart.available() >= 3 )
  {
    // TFT FeatherWing use 16-bit RGB 655 color, need to convert if input is 24-bit color
    if ( imageColorBit == 24 )
    {
      uint8_t rgb[3];
      bleuart.read(rgb, 3);
      pixel_buf[totalPixel % imageWidth] = ((rgb[0] & 0xF8) << 8) | ((rgb[1] & 0xFC) << 3) | (rgb[2] >> 3);
    }
    else if ( imageColorBit == 16 )
    {
      // native 16-bit 655 color
      pixel_buf[totalPixel % imageWidth] = bleuart.read16();
    }

    totalPixel++;

    // have enough to draw an image line
    if ( (totalPixel % imageWidth) == 0 )
    {
      tft.drawRGBBitmap(0, totalPixel/imageWidth, pixel_buf, imageWidth, 1);
    }
  }

  // all pixel data is received
  if ( totalPixel == imageWidth*imageHeight )
  {
    uint8_t crc = bleuart.read();
    (void) crc;
    // do checksum later

    // print speed summary
    print_summary(totalPixel*(imageColorBit/8) + 8, rxLastTime-rxStartTime);

    // reset and waiting for new image
    imageColorBit = 0;
    imageWidth = imageHeight = 0;
    totalPixel = 0;
  }
}

void connect_callback(uint16_t conn_handle)
{
  tft.println("Connected");
  tft.setTextColor(COLOR_GREEN);
  tft.println("Ready to receive new image");
  tft.setTextColor(COLOR_WHITE);
}

void print_summary(uint32_t count, uint32_t ms)
{
  float sec = ms / 1000.0F;

  // Print to serial
  Serial.printf("Received %d bytes in %.2f seconds\n", count, sec);
  Serial.printf("Speed: %.2f KB/s\n\n", (count / 1024.0F) / sec);
  Serial.println("Ready to receive new image");

  // Also print to TFT with color text
  tft.setCursor(0, imageHeight+5);
  tft.print("Received ");

  tft.setTextColor(COLOR_YELLOW);
  tft.print(count);
  tft.setTextColor(COLOR_WHITE);

  tft.print(" bytes in ");

  tft.setTextColor(COLOR_YELLOW);
  tft.print(sec, 2);
  tft.setTextColor(COLOR_WHITE);

  tft.println(" seconds");

  tft.print("Speed: ");
  tft.setTextColor(COLOR_YELLOW);
  tft.print( (count / 1024.0F) / sec, 2);
  tft.setTextColor(COLOR_WHITE);
  tft.print(" KB/s for ");

  tft.setTextColor(COLOR_YELLOW);
  tft.print(imageWidth); tft.print("x"); tft.print(imageHeight);

  tft.setTextColor(COLOR_WHITE);
  tft.println(" Image");

  tft.setTextColor(COLOR_GREEN);
  tft.println("Ready to receive new image");
  tft.setTextColor(COLOR_WHITE);
}

void bleuart_overflow_callback(uint16_t conn_hdl, uint16_t leftover)
{
  (void) conn_hdl;
  (void) leftover;
  
  Serial.println("BLEUART rx buffer OVERFLOWED!");
  Serial.println("Please increase buffer size for bleuart");

  // only print the first time this occur, need disconnect to reset
  if (!bleuart_overflowed)
  {
    tft.setCursor(0, imageHeight+5);

    tft.setTextColor(COLOR_RED);
    tft.println("BLEUART rx buffer OVERFLOWED!");

    tft.setTextColor(COLOR_WHITE);
    tft.print("Please increase buffer size for bleuart");
  }

  bleuart_overflowed = true;
}

/**
 * invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  tft.fillScreen(COLOR_BLACK);
  tft.setCursor(0, 0);
  tft.println("Advertising ...");

  imageColorBit = 0;
  imageWidth = imageHeight = 0;
  totalPixel = 0;
  bleuart_overflowed = false;

  bleuart.flush();
}
