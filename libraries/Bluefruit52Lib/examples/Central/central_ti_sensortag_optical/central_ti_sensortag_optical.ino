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

/* This sketch shows how to use BLEClientService and BLEClientCharacteristic
 * to implement a custom client that is used to talk with Gatt server on
 * the TI SensorTag peripheral device. 
 * 
 * Online uuid/hex converters
 * https://yupana-engineering.com/online-uuid-to-c-array-converter
 * https://www.scadacore.com/tools/programming-calculators/online-hex-converter/
 * https://codebeautify.org/string-hex-converter
 *
 * Adafruit Bluefruit Feather nRF52
 * https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/blescanner 
 * 
 * Sensor Tag   
 * http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User%27s_Guide#Optical_Sensor
 * 
 * Scan Response (Extended Advertisement data retrieval)
 * https://devzone.nordicsemi.com/tutorials/b/bluetooth-low-energy/posts/ble-advertising-a-beginners-tutorial
 *
 */ 

 #include <bluefruit.h>

/* TI Sensor Tag UUID Definitions 
  * Base SensorTag UUID pattern:      F000-0000-0451-4000-B000-000000000000
  * Optical Service:                  0xA070
  * Optical Characteristic:           0xAA71
  * Optical Data Collection Enabler:  0xAA72 
  * Optical Measurement Period:       0xAA73
  * Local Name:                       "CC2650 SensorTag" 
 */
 
 uint8_t SENSORTAG_OPTICAL_SERVICE_UUID[] =               {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x70,0xAA,0x00,0xF0};
 uint8_t SENSORTAG_OPTICAL_CHARACTERISTIC_UUID[] =        {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x71,0xAA,0x00,0xF0};
 uint8_t SENSORTAG_OPTICAL_ENABLE_CHARACTERISTIC_UUID[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x72,0xAA,0x00,0xF0};
 uint8_t SENSORTAG_OPTICAL_PERIOD_CHARACTERISTIC_UUID[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0,0x00,0x40,0x51,0x04,0x73,0xAA,0x00,0xF0};
 uint8_t SENSORTAG_ADV_COMPLETE_LOCAL_NAME[] =            {0x43,0x43,0x32,0x36,0x35,0x30,0x20,0x53,0x65,0x6e,0x73,0x6f,0x72,0x54,0x61,0x67};  

BLEClientService        sensorTagOpticalService(SENSORTAG_OPTICAL_SERVICE_UUID);
BLEClientCharacteristic sensorTagOpticalCharacteristic(SENSORTAG_OPTICAL_CHARACTERISTIC_UUID); 
BLEClientCharacteristic opticalCharacteristicDataCollectionEnabler(SENSORTAG_OPTICAL_ENABLE_CHARACTERISTIC_UUID);
BLEClientCharacteristic opticalCharacteristicPeriod(SENSORTAG_OPTICAL_PERIOD_CHARACTERISTIC_UUID);

 
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central TI Sensor Tag Example");
  Serial.println("-----------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Bluefruit.setName("Bluefruit52 TI SensorTag Central");

  // Initialize Service
  sensorTagOpticalService.begin();

  // set up callback for receiving measurement
  sensorTagOpticalCharacteristic.setNotifyCallback(sensortag_optical_notify_callback);
  sensorTagOpticalCharacteristic.begin();

  // set up the characteristic to enable the optical component on the device 
  opticalCharacteristicDataCollectionEnabler.begin();
  
  // set up the characteristic to adjust the measurement period 
  opticalCharacteristicPeriod.begin();

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
 
  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - No UUID Filter 
   * - Use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  // Bluefruit.Scanner.filterUuid();      // do not not set anything here or Scan Response won't work
  Bluefruit.Scanner.useActiveScan(true);  // required for SensorTag to reveal the Local Name in the advertisement. 
  Bluefruit.Scanner.start(0);             // 0 = Don't stop scanning after n seconds
}

void loop()
{
  // do nothing
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{ 
  Serial.println("");
  Serial.println("Scan Callback");
  printReport( report ); 
  
  /* Choose a peripheral to connect with by searching for an advertisement packet with a 
  Complete Local Name matching our target device*/
  uint8_t buffer[BLE_GAP_ADV_SET_DATA_SIZE_MAX] = { 0 };

  Serial.print("Parsing report for Local Name ... ");
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.println("Found Local Name");
    Serial.printf("%14s %s\n", "Local Name:", buffer);

    Serial.print("   Local Name data: ");
    printHexList(buffer, BLE_GAP_ADV_SET_DATA_SIZE_MAX ); 

    Serial.print("Determining Local Name Match ... ");
    if ( !memcmp( buffer, SENSORTAG_ADV_COMPLETE_LOCAL_NAME, sizeof(SENSORTAG_ADV_COMPLETE_LOCAL_NAME)) )
    {
      Serial.println("Local Name Match!");

      Serial.println("Connecting to Peripheral ... ");
      Bluefruit.Central.connect(report);
    }
    else
    {
      Serial.println("No Match");
      Bluefruit.Scanner.resume(); // continue scanning
    } 
  } 
  else
  {
    Serial.println("Failed");

    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }  
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("");
  Serial.print("Connect Callback, conn_handle: "); 
  Serial.println( conn_handle );
   
  // If Service is not found, disconnect and return
  Serial.print("Discovering Optical Service ... ");
  if ( !sensorTagOpticalService.discover(conn_handle) )
  {
    Serial.println("No Service Found");

    // disconnect since we couldn't find service
    Bluefruit.disconnect(conn_handle);

    return;
  } 
  Serial.println("Service Found");

  // Once service is found, we continue to discover the primary characteristic
  Serial.print("Discovering Optical Characteristic ... ");
  if ( !sensorTagOpticalCharacteristic.discover() )
  {
    // Measurement chr is mandatory, if it is not found (valid), then disconnect
    Serial.println("No Characteristic Found. Characteristic is mandatory but not found. ");
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Characteristic Found");
 
  // Data Collection Charactistic. Find and enable. 
  // You enable data collection on the Characteristic before the peripheral will start measuring values.
  // This is different than setting the Notify descriptor, which is handled below.
  Serial.print("Discovering Data Collection Configuration Characteristic ... ");
  if ( !opticalCharacteristicDataCollectionEnabler.discover() )
  {
    Serial.println("No Characteristic Found. Characteristic is mandatory but not found.");   
    Bluefruit.disconnect(conn_handle);
    return;
  }
  Serial.println("Characteristic Found"); 
  // Found it, now write 0x01 to turn on optical data collection
  Serial.print("Enabling Data Collection, return value: ");
  Serial.println( opticalCharacteristicDataCollectionEnabler.write8( 0x01 ) );
 
  // Measurement Period Characteristic. Find and adjust. 
  Serial.print("Measurement Period Characteristic ... ");
  if ( !opticalCharacteristicPeriod.discover() )
  {
    Serial.println("No Characteristic Found, but not mandatory so not disconnecting");    
  }
  Serial.println("Characteristic Found");
  // Found it, now adjust it: 
  // Resolution 10 ms. Range 100 ms (0x0A) to 2.55 sec (0xFF). Default is 800 milliseconds (0x50).
  // 19 is 250
  Serial.print("Adjusting Measurement Period, return value: ");
  // Serial.println( opticalCharacteristicPeriod.write8( 0xFF ) ); // Slowest
  Serial.println( opticalCharacteristicPeriod.write8( 0x0A ) ); // Fastest

  // Reaching here means we are ready to go, let's enable notification on measurement chr
  Serial.print("Enabling Notify on the Characteristic ... ");
  if ( sensorTagOpticalCharacteristic.enableNotify() )
  {
    Serial.println("enableNotify success, now ready to receive Characteristic values");
  }else
  {
    Serial.println("Couldn't enable notify for Characteristic. Increase DEBUG LEVEL for troubleshooting");
  }
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

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be sensorTagOpticalCharacteristic
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void sensortag_optical_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{ 
  Serial.print("Optical data: "); 
  uint16_t tData = (uint16_t) ( ( data[0] & 0xFF ) | ( data[1] & 0xFF ) << 8 );
  Serial.print( sensorOpt3001Convert( tData ) ); 
  Serial.println( " Lux" );
} 

/*
 * Conversion to Lux. Algorithm from the TI Sensor Tag Wiki documentation page
 */
float sensorOpt3001Convert(uint16_t rawData)
{
    uint16_t e, m;
 
    m = rawData & 0x0FFF;
    e = (rawData & 0xF000) >> 12;
 
    /** e on 4 bits stored in a 16 bit unsigned => it can store 2 << (e - 1) with e < 16 */
    e = (e == 0) ? 1 : 2 << (e - 1);
 
    return m * (0.01 * e);
}

/* Prints a hex list to the Serial Monitor */
void printHexList(uint8_t* buffer, uint8_t len)
{  
  // print forward order
  for(int i=0; i<len; i++)
  { 
    Serial.printf("%02X-", buffer[i]);
  } 
  Serial.println();  
}

void printReport( const ble_gap_evt_adv_report_t* report )
{
  Serial.print( "  rssi: " );
  Serial.println( report->rssi );
  Serial.print( "  scan_rsp: " );
  Serial.println( report->type.scan_response );
//  Serial.print( "  type: " );
//  Serial.println( report->type );
  Serial.print( "  dlen: " );
  Serial.println( report->data.len );  
  Serial.print( "  data: " );
  for( int i = 0; i < report->data.len; i+= sizeof(uint8_t) )
  {
    Serial.printf( "%02X-", report->data.p_data[ i ] );
  }
  Serial.println(""); 
}
