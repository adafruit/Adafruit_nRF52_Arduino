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

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.setHardwareRev("Rev E");
  bledis.setFirmwareRev(ARDUINO_BSP_VERSION); // ARDUINO_BSP_VERSION is BSP Libraries version
  bledis.start();

  // Configure and Start BLE Uart Service
  bleuart.start();

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.startAdvertising();
}

void setupAdv(void)
{
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.addAdvService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.addScanRespName();
}

void connect_callback(void)
{
  Serial.println("Connected");
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
  Serial.println("Bluefruit will auto start advertising (default)");
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
