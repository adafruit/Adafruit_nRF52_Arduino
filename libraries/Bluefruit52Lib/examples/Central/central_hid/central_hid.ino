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

/*
 * This sketch demonstrate the central API(). A additional bluefruit
 * that has bleuart as peripheral is required for the demo.
 */
#include <bluefruit.h>

BLEClientHidAdafruit hid;

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central HID (Keyboard + Mouse) Example");
  Serial.println("--------------------------------------------------\n");
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Init BLE Central Uart Serivce
  hid.begin();
  //clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

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
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);             // 0 = Don't stop scanning after n seconds

  Bluefruit.Scanner.filterService(hid);   // only report HID service
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Connect to device
  Bluefruit.Central.connect(report);
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  Serial.print("Discovering HID  Service ... ");

  if ( hid.discover(conn_handle) )
  {
    Serial.println("Found it");

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
    uint8_t hidInfo[4];
    hid.getHidInfo(hidInfo);

    Serial.printf("HID version: %d.%d\n", hidInfo[0], hidInfo[1]);
    Serial.print("Country code: "); Serial.println(hidInfo[2]);
    Serial.printf("HID Flags  : 0x%02X\n", hidInfo[3]);
    

//    Serial.println("Enable TXD's notify");
//    clientUart.enableTXD();

    Serial.println("Ready to receive from peripheral");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }  
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
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
//void bleuart_rx_callback(BLEClientUart& uart_svc)
//{
//  Serial.print("[RX]: ");
//  
//  while ( uart_svc.available() )
//  {
//    Serial.print( (char) uart_svc.read() );
//  }
//
//  Serial.println();
//}

void loop()
{
  // nothing to do
  if ( !hid.discovered() ) return;

}

