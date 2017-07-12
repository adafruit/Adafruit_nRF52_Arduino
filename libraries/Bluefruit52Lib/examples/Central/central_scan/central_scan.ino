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

void setup() 
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 Central Scan Example");

  // up to 1 peripheral conn and 1 central conn
  Bluefruit.begin(true, true);
  Bluefruit.setName("Bluefruit52");

  // Start Central Scan
  Bluefruit.setConnLedInterval(250);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.start(0);

  Serial.println("Scanning ...");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  Serial.println("Timestamp Addr              Rssi Data");

  Serial.printf("%09d ", millis());
  
  Serial.printBuffer(report->peer_addr.addr, 6, ':');
  Serial.print(" ");

  Serial.print(report->rssi);
  Serial.print("  ");

  Serial.printBuffer(report->data, report->dlen, '-');
  Serial.println();

  // Check if advertising contain BleUart service
  if ( Bluefruit.Central.checkUuidInScan(report, BLEUART_UUID_SERVICE) )
  {
    Serial.println("                       BLE UART service detected");
  }

  Serial.println();
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_RED);

  delay(1000);
}

