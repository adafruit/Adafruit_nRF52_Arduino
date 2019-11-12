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
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_CircuitPlayground.h>

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

/* All Adafruit Service/Characteristic UUID128 share the same
 * Base UUID : ADAFxxx-C332-42A8-93BD-25E905756CB8
 *
 * Share Characteristics
 *  - Measurement Period  0001 | int32_t | Read + Write | ms between measurements, -1: stop reading, 0: update when changes
 *
 * Temperature service    0100
 *  - Temperature         0101 | float    | Read + Notify | degree in Celsius
 *  - Measurement Period  0001
 *
 * Accelerometer service  0200
 *  - Accel Data          0201 | float (x, y, z) | Read + Notify | accel x, y, z
 *  - Measurement Period  0001
 *
 * Light sensor service   0300
 *  - Lux Data            0301 | uint16_t | Read + Notify
 *  - Measurement Period  0001
 *
 * Sound sensor service   0400
 *  - Sound Data          0401 | int16_t  | Read + Notify
 *  - Measurement Period  0001
 *
 * Captouch service       0500
 *  - Sensitivity         0501 | uint32_t | difference that Capn should report immediately
 *  - Cap0                0510 | int32_t  | Read + Notify
 *  - .........................
 *  - Cap7                0517 | int32_t  | Read + Notify
 *  - Measurement Period  0001 | uint32_t | Read + Write
 *
 * Button service         0600
 *  - Button Data         0601 | uint16_t | Read + Notify | e.g (Slide sw, Left, Right)
 *  - Measurement Period  0001 | uint32_t | Read + Write
 *
 * PIN I/O service        0700
 *  - Pin Dir             0701 | uint64_t | Read + Write | bit 1 is In, 0 is Out
 *  - Pin Data            0702 | uint64_t | Read + Write |
 *  - PWM Control         0710 | struct { uint8_t pin, uint16_t value, uint32_t period_ms} | as microbit, value is 0-1024 for duty cycle
 *
 * PIN Analog             0800
 *  - A0                  0810 | uint16_t | Read + Notify
 *    .........................
 *  - A7                  0817 | uint16_t | Read + Notify
 *  - Measurement Period  0001 | uint32_t | Read + Write
 *
 * Neopixel Service       0900
 *   - Pixel Pin          0901 | uint8_t  | Read + Write
 *   - Pixel Count        0902 | uint16_t | Read + Write
 *   - Pixel Type         0903 | uint16_t | Read + Write | NEO_RGB, NEO_GRB etc ..
 *   - Pixel Data         0904 | RGB array| Write
 *
 * Gamepad Service        0A00
 *   - Gamepad Button     0A01 | uint16_t  | Write
 *   - Left XY            0A02 | int8_t[2] | Write
 *   - Right XY           0A03 | int8_t[2] | Write
 *
 * Image Service          0B00
 *    - Format            0B01 | uint16_t[3] | Write | width + height + type, type is enum value e.g RGB_655, RGB_888, JPG, GIF etc ...
 *    - Data              0B02 | pixel[]     | Write | will save to flash as `default_image`, and auto load on reset ?
 *    - Fill              0B03 | uint8_t[3]  | Write | Fill with solid color
 *    - Load File         0B04 | utf8_t      | Write | Load image from qspi flash ?
 */

/* Adafruit NeoPixel Service
 * - Service: ADAF0002-C332-42A8-93BD-25E905756CB8
 *    - Count : ADAF0003-C332-42A8-93BD-25E905756CB8
 *    - Type  : ADAF0004-C332-42A8-93BD-25E905756CB8
 *    - Data  : ADAF0005-C332-42A8-93BD-25E905756CB8
 */

/* Adafruit Accelerometer Service
 * using micro:bit Accelerometer Service definition
 * https://lancaster-university.github.io/microbit-docs/resources/bluetooth/bluetooth_profile.html
 *
 * - Service: E95D-0753-251D-470A-A062-FA1922DFA9A8
 *    - Data   : E95D-CA4B-251D-470A-A062-FA1922DFA9A8
 *    - Period : E95D-FB24-251D-470A-A062-FA1922DFA9A8
 */

BLEAdafruitTemperature bleTemp;
BLEAdafruitNeopixel bleNeopixel;
BLEAdafruitAccel bleAccel;


 
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  CircuitPlayground.begin();

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Adafruit service
  bleTemp.begin();
  bleNeopixel.begin();
  bleAccel.begin();
  

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
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
  if ( !Bluefruit.connected() ) return;
  if ( !bleTemp.Temperature.notifyEnabled() ) return;

  int16_t temp = (int16_t) (100*CircuitPlayground.temperature());
  bleTemp.Temperature.notify16((uint16_t) temp);

  delay(1000*bleTemp.MeasurementInterval.read16());
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
}
