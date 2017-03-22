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

BLECentralUart bleCentralUart;

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central BLEUART Example");

  // up to 1 peripheral conn and 1 central conn
  Bluefruit.begin(true, true);
  Bluefruit.setName("Bluefruit52");

  // Init BLE Central Uart Serivce
  bleCentralUart.begin();

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

void loop() 
{
  if ( Bluefruit.Central.connected() )
  {
    Serial.print("Discovering BLE Uart Service ... ");

    if ( bleCentralUart.discover() )
    {
      Serial.println("Found it");

      Serial.println("Enable TXD's notify");
      bleCentralUart.enableNotify();

      Serial.println("Ready to receive from peripheral");
      while ( 1 )
      {
        if ( bleCentralUart.available() )
        {
          Serial.print( (char) bleCentralUart.read() );
        }
        delay(1);
      }
    }else
    {
      Serial.println("Found NONE");
    }    
  }
}

