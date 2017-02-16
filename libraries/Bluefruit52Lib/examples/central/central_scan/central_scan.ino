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
  Bluefruit.begin(0, 1);
  Bluefruit.setName("Bluefruit52");

  // Start Central Scan
  Bluefruit.setConnLedInterval(250);
  Bluefruit.setScanCallback(scan_callback);
  Bluefruit.startScanning();
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  PRINT_BUFFER(report->data, report->dlen);
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_BUILTIN);

  delay(1000);
}

