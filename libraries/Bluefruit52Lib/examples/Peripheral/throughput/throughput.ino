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

// String to send in the throughput test
#define TEST_STRING     "01234567899876543210"
const int TEST_STRLEN = strlen(TEST_STRING);

// Number of total data sent ( 1024 times the test string)
#define TOTAL_BYTES     (1024 * strlen(TEST_STRING))

BLEDis bledis;
BLEUart bleuart;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 Throughput Example");
  Serial.println("------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);          // Maximum TX power = 4 dBm
  Bluefruit.setName("Bluefruit52");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

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

void connect_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  Serial.println("Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  uint32_t start, stop, sent;
  uint32_t remaining = TOTAL_BYTES;
  start = stop = sent = 0;

  if (Bluefruit.connected() && bleuart.notifyEnabled())
  {
    // Wait for user input before trying again
    Serial.println("Connected. Send a key and press enter to start test");
    getUserInput();
    
    Serial.print("Sending ");
    Serial.print(remaining);
    Serial.println(" bytes ...");

    start = millis();
    while (remaining > 0)
    {
      bleuart.print(TEST_STRING);

      sent      += TEST_STRLEN;
      remaining -= TEST_STRLEN;

      // Only print every 100th time
      // if ( (sent % (100*TEST_STRLEN) ) == 0 )
      // {
      //  Serial.print("Sent: "); Serial.print(sent);
      //  Serial.print(" Remaining: "); Serial.println(remaining);
      // }
    }
    stop = millis() - start;

    Serial.print("Sent ");
    Serial.print(sent);
    Serial.print(" bytes in ");
    Serial.print(stop / 1000.0F, 2);
    Serial.println(" seconds.");

    Serial.println("Speed ");
    Serial.print( (sent / 1000.0F) / (stop / 1000.0F), 2);
    Serial.println(" KB/s.\r\n");
  }
}

/**************************************************************************/
/*!
    @brief  Get user input from Serial
*/
/**************************************************************************/
char* getUserInput(void)
{
  static char inputs[64+1];
  memset(inputs, 0, sizeof(inputs));

  // wait until data is available
  while( Serial.available() == 0 ) { delay(1); }

  uint8_t count=0;
  do
  {
    count += Serial.readBytes(inputs+count, 64);
  } while( (count < 64) && Serial.available() );

  return inputs;
}
