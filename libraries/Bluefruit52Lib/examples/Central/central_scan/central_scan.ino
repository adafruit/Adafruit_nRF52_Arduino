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
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central Scan Example");
  Serial.println("--------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Start Central Scan
  Bluefruit.setConnLedInterval(250);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.start(0);

  Serial.println("Scanning ...");
}

//
// Parse the bluetooth device name from the scan data record
//
bool get_device_name(uint8_t *pData, int iLen, char *szName)
{
int iOff = 0;
unsigned char ucType, ucLen;
bool bGotString = false;

   while (iOff < iLen)
   {
      ucType = pData[iOff++]; // device type
      ucLen = pData[iOff++];  // followed by 1 byte field length
      if (ucType == 0x13) // device name
      {
         if (ucLen + iOff <= iLen) // valid length?
         {
            memcpy(szName, &pData[iOff], ucLen);
            szName[ucLen] = 0; // zero terminate the string
            bGotString = true;
         }
      }
      iOff += ucLen; // next data field
   }
   return bGotString;
} /* get_device_name() */

void scan_callback(ble_gap_evt_adv_report_t* report)
{
char szName[32];
 
  if (get_device_name(report->data.p_data, report->data.len, szName))
  {
    Serial.printf("Device Name = ");
    Serial.println(szName);
  }

  Serial.println("Timestamp Addr              Rssi Data");

  Serial.printf("%09d ", millis());
  
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print(" ");

  Serial.print(report->rssi);
  Serial.print("  ");

  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  Serial.println();

  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.println("                       BLE UART service detected");
  }

  Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void loop() 
{
  // nothing to do
}
