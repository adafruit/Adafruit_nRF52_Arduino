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

// BLE Service
BLEAncs  bleancs;

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 BLE ANCS Example");
  Serial.println("----------------------------");

  Serial.println("Go to iOS's Bluetooth settings to connect to Bluefruit");

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Service
  bleancs.begin();
  
  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include ancs 128-bit uuid
  Bluefruit.Advertising.addService(bleancs);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void loop()
{
  // not connected, nothing to do
  if ( !Bluefruit.connected() ) return ;

  // Discover service if not yet
  if ( !bleancs.discovered() )
  {
    Serial.print("Discovering ANCS ... ");
    if ( bleancs.discover( Bluefruit.connHandle() ) )
    {
      Serial.println("Found it");

      // ANCS requires pairing to work, it makes sense to request security here as well
      Serial.println("Attempt to bond/pair with iOS, please press PAIR on your phone");
      if ( Bluefruit.requestPairing() )
      {
        Serial.println("Enable Notification Source's CCCD");      
        bleancs.enableNotification();    
      }
    }
  }
}

void connect_callback(void)
{
  Serial.println("Connected");
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
  Serial.println("Bluefruit will auto start advertising (default)");
}
