#include <bluefruit.h>

BLEDis bledis;
BLEClientDis bleClientDis;
BLEAncsLoopback bleancs;

char buffer[128];

const uint8_t ANCS_LOOPBACK_GATT_SERVICE[] =
{
  0xFD, 0xC2, 0x3A, 0x2B, 0xEC, 0xE2, 0xEE, 0x9B, 
  0x94, 0x49, 0x6F, 0x6E, 0x8B, 0x13, 0xF6, 0x39
};

const uint8_t ANCS_LOOPBACK_GATT_NOTIFICATION_CHR[] =
{
  0x57, 0xD7, 0x34, 0xB0, 0x0C, 0x78, 0x2F, 0x94, 
  0xCB, 0x4D, 0x7C, 0x73, 0x28, 0x30, 0x94, 0x76
};

const uint8_t ANCS_LOOPBACK_GATT_CONTROL_POINT_CHR[] =
{
  0x75, 0x0B, 0xE6, 0x0F, 0xFF, 0x3E, 0xB3, 0xB5, 
  0xA3, 0x49, 0xDA, 0x24, 0x7C, 0xD4, 0x6F, 0xD2
};

const uint8_t ANCS_LOOPBACK_GATT_DATA_SOURCE_CHR[] =
{
  0x5B, 0x7B, 0xA8, 0x73, 0xD7, 0xB4, 0xA6, 0xA4, 
  0x83, 0x4E, 0x4A, 0xB2, 0x8C, 0x5F, 0xA3, 0x62
};

BLEService        loopbackGattService     = BLEService       (ANCS_LOOPBACK_GATT_SERVICE);
BLECharacteristic loopbackNotificationChr = BLECharacteristic(ANCS_LOOPBACK_GATT_NOTIFICATION_CHR);
BLECharacteristic loopbackControlPointChr = BLECharacteristic(ANCS_LOOPBACK_GATT_CONTROL_POINT_CHR);
BLECharacteristic loopbackDataSourceChr   = BLECharacteristic(ANCS_LOOPBACK_GATT_DATA_SOURCE_CHR);

void setup () 
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Ancs characteristics loopback"));
  Serial.println(F("-----------------------------\n"));

  Bluefruit.configPrphConn(128, 3, 1, 1);
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DM ancs loopback");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Drivemode Inc.");
  bledis.setModel("Drivemode Socket alpha");
  bledis.begin();

  // Configure DIS client
  bleClientDis.begin();

  // Configure ANCS client
  bleancs.begin();
  bleancs.setNotificationCallback(ancs_loopback_notification_callback);
  bleancs.setDataCallback(ancs_loopback_data_callback);

  // Configure ANCS Loopback GATT
  loopbackGattService.begin();
  
  loopbackNotificationChr.setProperties(CHR_PROPS_NOTIFY);
  loopbackNotificationChr.begin();

  loopbackControlPointChr.setProperties(CHR_PROPS_WRITE);
  loopbackControlPointChr.begin();

  loopbackDataSourceChr.setProperties(CHR_PROPS_NOTIFY);
  loopbackDataSourceChr.begin();
  
  startAdvertise();
}

void loop () {}

void startAdvertise(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_UNKNOWN);

  Bluefruit.Advertising.addService(loopbackGattService);
  
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 msu
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start();
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println(conn_handle);
  Serial.println(F("Connected"));

  Serial.print(F("Discovering DIS ... "));
  if ( bleClientDis.discover(conn_handle) )
  {
    Serial.println(F("Discovered"));

    // Read and print Manufacturer string
    memset(buffer, 0, sizeof(buffer));
    if ( bleClientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print(F("Manufacturer: "));
      Serial.println(buffer);
    }

    // Read and print Model Number string
    memset(buffer, 0, sizeof(buffer));
    if ( bleClientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print(F("Model: "));
      Serial.println(buffer);
    }

    Serial.println();
  }

  discoverANCS(conn_handle);
}

void discoverANCS(uint16_t conn_handle) {
  Serial.print(F("Discovering ANCS ... "));
  if ( bleancs.discover(conn_handle) )
  {
    Serial.println(F("Discovered"));

    // ANCS requires pairing to work, it makes sense to request security here as well
    Serial.print(F("Attempting to PAIR with the iOS device, please press PAIR on your phone ... "));
    if ( ! Bluefruit.connPaired() ) {
      if ( Bluefruit.requestPairing() )
      {
        Serial.println(F("Done"));
        Serial.println(F("Enabling notifications"));
        Serial.println();
        bleancs.enableNotification();
      }
    }
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;
  Serial.println();
  Serial.println(F("Disconnected"));
}

void ancs_loopback_notification_callback(uint8_t* data, uint16_t len)
{
  Serial.println("received notification");
  loopbackNotificationChr.notify(data, len);
}

void ancsLoopbackWriteCallback(BLECharacteristic& chr, uint8_t* data, uint16_t len, uint16_t offset)
{
  Serial.println("writing to Control point");
  bleancs.requestNotificationData(data, len);
}

void ancs_loopback_data_callback(uint8_t* data, uint16_t len)
{
  Serial.println("reiceved the information requested");
  loopbackDataSourceChr.notify(data, len);
}
