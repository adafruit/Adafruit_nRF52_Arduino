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

BLECentralUart bleCentralUart;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central BLEUART Example");
  Serial.println("-----------------------------------");
  
  // up to 1 peripheral conn and 1 central conn
  Bluefruit.begin(true, true);
  Bluefruit.setName("Bluefruit52");

  // Init BLE Central Uart Serivce
  bleCentralUart.begin();
  bleCentralUart.setRxCallback(uart_rx_callback);

  // Increase BLink rate to different from PrPh advertising mode 
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  // Start Central Scan
  Bluefruit.Central.setScanCallback(scan_callback);
  Bluefruit.Central.startScanning();
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Central.checkUuidInScan(report, BLEUART_UUID_SERVICE) )
  {
    Serial.println("BLE UART service detected");
    Serial.println("Attempt to connect ... ");

    // Connect to device with bleuart service in advertising
    // Use Min & Max Connection Interval default value
    Bluefruit.Central.connect(report);
  }
}

void connect_callback(void)
{
  Serial.println("Connected");
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;
  
  Serial.println("Disconnected");
  Serial.println("Bluefruit will auto start scanning (default)");
}

void uart_rx_callback(void)
{
  Serial.print("[RX]: ");
  
  while ( bleCentralUart.available() )
  {
    Serial.print( (char) bleCentralUart.read() );
  }

  Serial.println();
}

void loop() 
{
  if ( Bluefruit.Central.connected() )
  {
    // Not discovered yet
    if ( !bleCentralUart.discovered() )
    {
      Serial.print("Discovering BLE Uart Service ... ");

      if ( bleCentralUart.discover() )
      {
        Serial.println("Found it");
  
        Serial.println("Enable TXD's notify");
        bleCentralUart.enableNotify();
  
        Serial.println("Ready to receive from peripheral");
      }else
      {
        Serial.println("Found NONE");
      }
    }else
    {
      // Discovered means in working state
      // Get Serial input and send to Peripheral
      if ( Serial.available() )
      {
        bleCentralUart.write( (char) Serial.read() );
      }
    }
  }
}

