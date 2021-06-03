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
#include "Adafruit_EPD.h"

/* This sketch demonstrates the "Image Upload" feature of Bluefruit Mobile App.
 * Following TFT Display are supported
 *  - https://www.adafruit.com/product/4428
 */

#define EINK_GIZMO          1

// one of above supported TFT add-on
#define DISPLAY_IN_USE      EINK_GIZMO

#define EPD_CS      0
#define EPD_DC      1
#define SRAM_CS     -1
#define EPD_RESET   PIN_A3 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)

Adafruit_IL0373 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);


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

void setup()
{
  Serial.begin(115200);

  display.begin();
  display.clearBuffer();

#if DISPLAY_IN_USE == EINK_GIZMO
  display.setRotation(3);
#else
  // todo more eink displays
#endif

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

  Serial.println("Advertising ... ");
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

    display.clearBuffer();
    display.fillScreen(EPD_WHITE);
    display.setCursor(0, 0);

    // Print out the current connection info
    BLEConnection* conn = Bluefruit.Connection(conn_hdl);
    Serial.printf("Connection Info: PHY = %d Mbps, Conn Interval = %.2f ms, Data Length = %d, MTU = %d\n",
                  conn->getPHY(), conn->getConnectionInterval()*1.25f, conn->getDataLength(), conn->getMtu());
    Serial.printf("Receving an %dx%d Image with %d bit color\n", imageWidth, imageHeight, imageColorBit);
  }

  // Extract pixel data to buffer and draw image line by line
  while ( bleuart.available() >= 3 )
  {
    uint8_t red, green, blue;

    if ( imageColorBit == 24 )
    {
      // Application send 24-bit color
      red = bleuart.read();
      green = bleuart.read();
      blue = bleuart.read();
    }
    else if ( imageColorBit == 16 )
    {
      // Application send 16-bit 565 color
      uint16_t c565 = bleuart.read16();

      red = (c565 & 0xf800) >> 8;
      green = (c565 & 0x07e0) >> 3;
      blue = (c565 & 0x001f) << 3;
    }else
    {
      Serial.println("Error: incorrect color bits ");
      while(1) yield();
    }

    // Convert RGB into Eink Color
    uint8_t c = 0;
    if ((red < 0x80) && (green < 0x80) && (blue < 0x80)) {
      c = EPD_BLACK; // try to infer black
    } else if ((red >= 0x80) && (green >= 0x80) && (blue >= 0x80)) {
      c = EPD_WHITE;
    } else if (red >= 0x80) {
      c = EPD_RED; // try to infer red color
    }

    // store to pixel buffer
    pixel_buf[totalPixel % imageWidth] = c;

    totalPixel++;

    // have enough to draw an image line
    if ( (totalPixel % imageWidth) == 0 )
    {
      display.drawRGBBitmap(0, totalPixel/imageWidth, pixel_buf, imageWidth, 1);
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

    // Display on Eink, will probably take dozens of seconds
    Serial.println("Displaying image (~20 seconds) .....");
    display.display();

    // reset and waiting for new image
    imageColorBit = 0;
    imageWidth = imageHeight = 0;
    totalPixel = 0;

    Serial.println("Ready to receive new image");
  }
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  Serial.println("Ready to receive new image");
}

void print_summary(uint32_t count, uint32_t ms)
{
  float sec = ms / 1000.0F;

  Serial.printf("Received %d bytes in %.2f seconds\n", count, sec);
  Serial.printf("Speed: %.2f KB/s\n\n", (count / 1024.0F) / sec);
}

void bleuart_overflow_callback(uint16_t conn_hdl, uint16_t leftover)
{
  (void) conn_hdl;
  (void) leftover;
  
  Serial.println("BLEUART rx buffer OVERFLOWED!");
  Serial.println("Please increase buffer size for bleuart");
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

  imageColorBit = 0;
  imageWidth = imageHeight = 0;
  totalPixel = 0;

  bleuart.flush();
}
