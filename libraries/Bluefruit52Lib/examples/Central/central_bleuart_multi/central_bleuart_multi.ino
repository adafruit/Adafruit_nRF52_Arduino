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

/* This sketch demonstrates the central API() that allows you to connect
 * to multiple peripherals boards (Bluefruit nRF52 in peripheral mode, or
 * any Bluefruit nRF51 boards).
 *
 * One or more Bluefruit boards, configured as a peripheral with the
 * bleuart service running are required for this demo.
 *
 * This sketch will: 
 *  - Read data from the HW serial port (normally USB serial, accessible
 *    via the Serial Monitor for example), and send any incoming data to
 *    all other peripherals connected to the central device.
 *  - Forward any incoming bleuart messages from a peripheral to all of
 *    the other connected devices.
 * 
 * It is recommended to give each peripheral board a distinct name in order
 * to more easily distinguish the individual devices.
 * 
 * Connection Handle Explanation
 * -----------------------------
 * The total number of connections is BLE_MAX_CONNECTION (20)
 * 
 * The 'connection handle' is an integer number assigned by the SoftDevice
 * (Nordic's proprietary BLE stack). Each connection will receive it's own
 * numeric 'handle' starting from 0 to BLE_MAX_CONNECTION-1, depending on the order
 * of connection(s).
 *
 * - E.g If our Central board connects to a mobile phone first (running as a peripheral),
 * then afterwards connects to another Bluefruit board running in peripheral mode, then
 * the connection handle of mobile phone is 0, and the handle for the Bluefruit
 * board is 1, and so on.
 */

/* LED PATTERNS
 * ------------
 * LED_RED   - Blinks pattern changes based on the number of connections.
 * LED_BLUE  - Blinks constantly when scanning
 */

#include <bluefruit.h>

// Struct containing peripheral info
typedef struct
{
  char name[16+1];

  uint16_t conn_handle;

  // Each prph need its own bleuart client service
  BLEClientUart bleuart;
} prph_info_t;

/* Peripheral info array (one per peripheral device)
 * 
 * There are 'BLE_MAX_CONNECTION' central connections, but the
 * the connection handle can be numerically larger (for example if
 * the peripheral role is also used, such as connecting to a mobile
 * device). As such, we need to convert connection handles <-> the array
 * index where appropriate to prevent out of array accesses.
 * 
 * Note: One can simply declares the array with BLE_MAX_CONNECTION and use connection
 * handle as index directly with the expense of SRAM.
 */
prph_info_t prphs[BLE_MAX_CONNECTION];

// Software Timer for blinking the RED LED
SoftwareTimer blinkTimer;
uint8_t connection_num = 0; // for blink pattern

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Initialize blinkTimer for 100 ms and start it
  blinkTimer.begin(100, blink_timer_callback);
  blinkTimer.start();

  Serial.println("Bluefruit52 Central Multi BLEUART Example");
  Serial.println("-----------------------------------------\n");
  
  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(0, 4);

  // Set Name
  Bluefruit.setName("Bluefruit52 Central");
  
  // Init peripheral pool
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    
    // All of BLE Central Uart Serivce
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service in advertising
   * - Don't use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised  
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if ( id < 0 ) return;
  
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;
  
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  Serial.print("Discovering BLE UART service ... ");

  if ( peer->bleuart.discover(conn_handle) )
  {
    Serial.println("Found it");
    Serial.println("Enabling TXD characteristic's CCCD notify bit");
    peer->bleuart.enableTXD();

    Serial.println("Continue scanning for more peripherals");
    Bluefruit.Scanner.start(0);

    Serial.println("Enter some text in the Serial Monitor to send it to all connected peripherals:");
  } else
  {
    Serial.println("Found ... NOTHING!");

    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  

  connection_num++;
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

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");
}

/**
 * Callback invoked when BLE UART data is received
 * @param uart_svc Reference object to the service where the data 
 * arrived.
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();

  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];
  
  // Print sender's name
  Serial.printf("[From %s]: ", peer->name);

  // Read then forward to all peripherals
  while ( uart_svc.available() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    
    if ( uart_svc.read(buf,sizeof(buf)-1) )
    {
      Serial.println(buf);
      sendAll(buf);
    }
  }
}

/**
 * Helper function to send a string to all connected peripherals
 */
void sendAll(const char* str)
{
  Serial.print("[Send to All]: ");
  Serial.println(str);
  
  for(uint8_t id=0; id < BLE_MAX_CONNECTION; id++)
  {
    prph_info_t* peer = &prphs[id];

    if ( peer->bleuart.discovered() )
    {
      peer->bleuart.print(str);
    }
  }
}

void loop()
{
  // First check if we are connected to any peripherals
  if ( Bluefruit.Central.connected() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    
    // Read from HW Serial (normally USB Serial) and send to all peripherals
    if ( Serial.readBytes(buf, sizeof(buf)-1) )
    {
      sendAll(buf);
    }
  }
}

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle)
{
  for(int id=0; id<BLE_MAX_CONNECTION; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;

  // Period of sequence is 10 times (1 second). 
  // RED LED will toggle first 2*n times (on/off) and remain off for the rest of period
  // Where n = number of connection
  static uint8_t count = 0;

  if ( count < 2*connection_num ) digitalToggle(LED_RED);
  if ( count % 2 && digitalRead(LED_RED)) digitalWrite(LED_RED, LOW); // issue #98

  count++;
  if (count >= 10) count = 0;
}
