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

/* This sketch demonstrate the BLEAncs service. After uploading, go to
 * iOS setting and connect to Bluefruit, and then press PAIR. Bluefruit
 * will print out any notification meesages to Serial Monitor
 */

#include <bluefruit.h>

// BLE Client Service
BLEClientDis  bleClientDis;
BLEAncs       bleancs;

char buffer[128];

// Check BLEAncs.h for AncsNotification_t
const char* EVENT_STR[] = { "Added", "Modified", "Removed" };
const char* CAT_STR  [] =
{
  "Other"             , "Incoming Call"       , "Missed Call", "Voice Mail"   ,
  "Social"            , "Schedule"            , "Email"      , "News"         ,
  "Health and Fitness", "Business and Finance", "Location"   , "Entertainment"
};

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 BLE ANCS Example");
  Serial.println("----------------------------\n");

  Serial.println("Go to iOS's Bluetooth settings and connect to Bluefruit52");
  Serial.println("It may appear up as 'Accessory' depending on your OS version.");

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure DIS client
  bleClientDis.begin();

  // Configure ANCS client
  bleancs.begin();
  bleancs.setNotificationCallback(ancs_notification_callback);

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include ANCS 128-bit uuid
  Bluefruit.Advertising.addService(bleancs);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  // Not connected, wait for a connection
  if ( !Bluefruit.connected() ) return;

  // If service is not yet discovered
  if ( !bleancs.discovered() ) return;

  // Your code here
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  
  Serial.print("Discovering DIS ... ");
  if ( bleClientDis.discover(conn_handle) )
  {
    Serial.println("Discovered");
    
    // Read and print Manufacturer string
    memset(buffer, 0, sizeof(buffer));
    if ( bleClientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // Read and print Model Number string
    memset(buffer, 0, sizeof(buffer));
    if ( bleClientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }

  Serial.print("Discovering ANCS ... ");
  if ( bleancs.discover(conn_handle) )
  {
    Serial.println("Discovered");

    // ANCS requires pairing to work, it makes sense to request security here as well
    Serial.print("Attempting to PAIR with the iOS device, please press PAIR on your phone ... ");
    if ( Bluefruit.requestPairing(conn_handle) )
    {
      Serial.println("Done");
      Serial.println("Enabling notifications");
      Serial.println();
      bleancs.enableNotification();

      Serial.println("| Event    | Category (count)     | Title          | Message         | App ID               | App Name        |");
      Serial.println("---------------------------------------------------------------------------------------------------------------");
    }
  }
}

void ancs_notification_callback(AncsNotification_t* notif)
{
  int n;
  Serial.printf("| %-8s | ", EVENT_STR[notif->eventID]);

  // Print Category with padding
  n = Serial.printf("%s (%d)", CAT_STR[notif->categoryID], notif->categoryCount);
  for (int i=n; i<20; i++) Serial.print(' ');
  Serial.print(" | ");

  // Get notification Title
  // iDevice often includes Unicode "Bidirection Text Control" in the Title.
  // Most strings have U+202D at the beginning and U+202C at the end. You may
  // want to remove them.
  // U+202D is E2-80-AD, U+202C is E2-80-AC in UTF-8
  memset(buffer, 0, sizeof(buffer));
  bleancs.getAttribute(notif->uid, ANCS_ATTR_TITLE, buffer, sizeof(buffer));
  Serial.printf("%-14s | ", buffer);
  
  // Get notification Message
  memset(buffer, 0, sizeof(buffer));
  bleancs.getAttribute(notif->uid, ANCS_ATTR_MESSAGE, buffer, sizeof(buffer));
  Serial.printf("%-15s | ", buffer);
  
  // Get App ID and store in the app_id variable
  char app_id[64] = { 0 };
  memset(buffer, 0, sizeof(buffer));
  bleancs.getAttribute(notif->uid, ANCS_ATTR_APP_IDENTIFIER, buffer, sizeof(buffer));
  strcpy(app_id, buffer);
  Serial.printf("%-20s | ", app_id);

  // Get Application Name
  memset(buffer, 0, sizeof(buffer));
  bleancs.getAppAttribute(app_id, ANCS_APP_ATTR_DISPLAY_NAME, buffer, sizeof(buffer));
  Serial.printf("%-15s | ", buffer);

  Serial.println();

  // Automatically accept incoming calls using 'performAction'
  if ( notif->categoryID == ANCS_CAT_INCOMING_CALL && notif->eventID == ANCS_EVT_NOTIFICATION_ADDED)
  {
    Serial.println("Incoming call accepted");
    bleancs.performAction(notif->uid, ANCS_ACTION_POSITIVE);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
