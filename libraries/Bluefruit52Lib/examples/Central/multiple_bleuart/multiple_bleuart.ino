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

/* This sketch demonstrate the central API() to connect multiple peripherals. 
 * One or more Bluefruit board that configured as peripheral bleuart is required 
 * for the demo. The sektch will 
 * - Read from HW Serial and send to all connected peripherals
 * - Forward any messages from a peripherals to all.
 * 
 * It is advised to name peripherals board differently to make it
 * easier to regconize.
 * 
 * Note: Connection Handle explanation
 * 
 * There are total concurrent connections BLE_MAX_CONN = BLE_PRPH_MAX_CONN + BLE_CENTRAL_MAX_CONN.
 * 
 * Connection handle is an integer number assigned by SoftDevice (ble stack) 
 * for each connection starting from 0 to to BLE_MAX_CONN-1, depending on the order
 * of connection.
 * - E.g If our board connect to mobile phone first (as peripheral), afterward, it connects
 * to another bluefruit board as central. Then the connection handle of mobile is 0, and
 * handle of bluefruit board is 1. And so on.
 */

/* LED_RED fast sequence blink the number of connections.
 * LED_BLUE blink constantly when scanning
 */
#include <bluefruit.h>

// Struct contain peripheral info
typedef struct
{
  char name[32];

  // Each prph need its own client service
  BLEClientUart bleuart;
} prph_info_t;

/* Peripherals info array indexed by connnection handle.
 * 
 * Note: Although there is only BLE_CENTRAL_MAX_CONN central connections
 * The conn handle can be larger (if peripheral role is used, e.g connect to mobile).
 * We just declare this array using BLE_MAX_CONN to prevent out of array access.
 * 
 * If you are tight on memory, you can declare this pool to only BLE_CENTRAL_MAX_CONN and
 * have a seperated mapping array from connection handle to array index.
 */
prph_info_t prphs[BLE_MAX_CONN];


// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;
uint8_t connection_num = 0; // for blink pattern

void setup() 
{
  Serial.begin(115200);

  // Initialize blinkTimer for 100 ms and start it
  blinkTimer.begin(100, blink_timer_callback);
  blinkTimer.start();

  Serial.println("Bluefruit52 Central Mulitple BLEUART Example");
  Serial.println("--------------------------------------------");
  
  // Enable both peripheral and central
  Bluefruit.begin(true, true);
  Bluefruit.setName("Bluefruit52 Central");
  
  // Init All of BLE Central Uart Serivce
  for (uint8_t idx=0; idx<BLE_CENTRAL_MAX_CONN; idx++)
  {
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(uart_rx_callback);
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  prph_info_t* peer = &prphs[conn_handle];
  
  Bluefruit.Gap.getPeerName(conn_handle, peer->name, 32);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  Serial.print("Discovering BLE Uart Service ... ");

  if ( peer->bleuart.discover(conn_handle) )
  {
    Serial.println("Found it");
    Serial.println("Enable TXD's notify");
    peer->bleuart.enableTXD();

    Serial.println("Continue scanning for more peripherals");
    Bluefruit.Scanner.start(0);

    Serial.println("Enter any texts to send to all peripherals:");
  }else
  {
    Serial.println("Found NONE");

    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }

  connection_num++;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("Disconnected");

  connection_num--;
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived.
 */
void uart_rx_callback(BLEClientUart& uart_svc)
{
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();
  prph_info_t* peer = &prphs[conn_handle];
  
  // Print sender's name
  Serial.printf("[From %s]: ", peer->name);

  // Read Then forward to all peripherals
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

void sendAll(const char* str)
{
  Serial.print("[Send to All]: ");
  Serial.println(str);  
  for(uint8_t conn=0; conn < BLE_MAX_CONN; conn++)
  {
    prph_info_t* peer = &prphs[conn];

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
    
    // Read from HW Serial and send to all peripherals
    if ( Serial.readBytes(buf, sizeof(buf)-1) )
    {
      sendAll(buf);
    }
  }
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

  if ( count < 2*connection_num) digitalToggle(LED_RED);

  count++;
  if (count >= 10) count = 0;
}
