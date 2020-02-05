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

/* Best result is 
 *  - 8.74 KB/s with 20 ms, MTU = 23
 *  - 23.62 KB/s with 7.5 ms, MTU = 23
 *  - 47.85 KB/s with 15 ms, MTU = 247 
 */

// data to send in the throughput test
char test_data[256] = { 0 };

// Number of packet to sent
// actualy number of bytes depends on the MTU of the connection
#define PACKET_NUM    1000

BLEDis bledis;
BLEUart bleuart;

uint32_t rxCount = 0;
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Throughput Example");
  Serial.println("------------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

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

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  bleuart.setRxCallback(bleuart_rx_callback);
  bleuart.setNotifyCallback(bleuart_notify_callback);

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
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println("Connected");

  // request PHY changed to 2MB
  Serial.println("Request to change PHY");
  conn->requestPHY();

  // request to update data length
  Serial.println("Request to change Data Length");
  conn->requestDataLengthUpdate();
    
  // request mtu exchange
  Serial.println("Request to change MTU");
  conn->requestMtuExchange(247);

  // request connection interval of 7.5 ms
  //conn->requestConnectionParameter(6); // in unit of 1.25

  // delay a bit for all the request to complete
  delay(1000);
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

void bleuart_rx_callback(uint16_t conn_hdl)
{
  (void) conn_hdl;
  
  rxLastTime = millis();
  
  // first packet
  if ( rxCount == 0 )
  {
    rxStartTime = millis();
  }

  uint32_t count = bleuart.available();

  rxCount += count;
  bleuart.flush(); // empty rx fifo

  // Serial.printf("RX %d bytes\n", count);
}

void bleuart_notify_callback(uint16_t conn_hdl, bool enabled)
{
  if ( enabled )
  {
    Serial.println("Send a key and press enter to start test");
  }
}

void print_speed(const char* text, uint32_t count, uint32_t ms)
{
  Serial.print(text);
  Serial.print(count);
  Serial.print(" bytes in ");
  Serial.print(ms / 1000.0F, 2);
  Serial.println(" seconds.");

  Serial.print("Speed : ");
  Serial.print( (count / 1000.0F) / (ms / 1000.0F), 2);
  Serial.println(" KB/s.\r\n");  
}

void test_throughput(void)
{
  uint32_t start, sent;
  start = sent = 0;

  const uint16_t data_mtu = Bluefruit.Connection(0)->getMtu() - 3;
  memset(test_data, '1', data_mtu);

  uint32_t remaining = data_mtu*PACKET_NUM;

  Serial.print("Sending ");
  Serial.print(remaining);
  Serial.println(" bytes ...");
  Serial.flush();
  delay(1);  

  start = millis();
  while ( (remaining > 0) && Bluefruit.connected() && bleuart.notifyEnabled() )
  {
    if ( !bleuart.write(test_data, data_mtu) ) break;

    sent      += data_mtu;
    remaining -= data_mtu;
  }

  print_speed("Sent ", sent, millis() - start);
}

void loop(void)
{  
  if (Bluefruit.connected() && bleuart.notifyEnabled())
  {
    // Wait for user input before trying again
    if ( Serial.available() )
    {
      getUserInput();
      test_throughput();
    }
    
    // 3 seconds has passed and there is no data received
    // then reset rx count
    if ( (rxCount > 0) && (rxLastTime + 1000 < millis()) )
    {
      print_speed("Received ", rxCount, rxLastTime-rxStartTime);
      rxCount = 0;
    }
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
