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
 * This sketch demonstrate the central API(). An additional bluefruit
 * that has blehid as peripheral is required for the demo.
 */
#include <bluefruit.h>

// Polling or callback implementation
#define POLLING       1

BLEClientHidAdafruit hid;

// Last checked report, to detect if there is changes between reports
hid_keyboard_report_t last_kbd_report = { 0 };
hid_mouse_report_t last_mse_report = { 0 };

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central HID (Keyboard + Mouse) Example");
  Serial.println("--------------------------------------------------\n");
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Init BLE Central Hid Serivce
  hid.begin();

  #if POLLING == 0  
  hid.setKeyboardReportCallback(keyboard_report_callback);
  #endif

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept HID service in advertising
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.filterService(hid);   // only report HID service
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);             // 0 = Don't stop scanning after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with hid service advertised  
  // Connect to the device with hid service in advertising packet
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

    // HID device mostly require pairing/bonding
    if ( !Bluefruit.requestPairing(conn_handle) )
    {
      Serial.print("Failed to paired");
      return;
    }

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
    uint8_t hidInfo[4];
    hid.getHidInfo(hidInfo);

    Serial.printf("HID version: %d.%d\n", hidInfo[0], hidInfo[1]);
    Serial.print("Country code: "); Serial.println(hidInfo[2]);
    Serial.printf("HID Flags  : 0x%02X\n", hidInfo[3]);

    // BLEClientHidAdafruit currently only supports Boot Protocol Mode
    // for Keyboard and Mouse. Let's set the protocol mode on prph to Boot Mode
    hid.setBootMode(true);

    // Enable Keyboard report notification if present on prph
    if ( hid.keyboardPresent() ) hid.enableKeyboard();

    // Enable Mouse report notification if present on prph
    if ( hid.mousePresent() ) hid.enableMouse();
    
    Serial.println("Ready to receive from peripheral");
  }else
  {
    Serial.println("Found NONE");
    
    // disconnect since we couldn't find blehid service
    Bluefruit.disconnect(conn_handle);
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

void loop()
{
  
#if POLLING == 1
  // nothing to do if hid not discovered
  if ( !hid.discovered() ) return;
  
  /*------------- Polling Keyboard  -------------*/
  hid_keyboard_report_t kbd_report;

  // Get latest report
  hid.getKeyboardReport(&kbd_report);

  processKeyboardReport(&kbd_report);


  // polling interval is 5 ms
  delay(5);
#endif
  
}


void keyboard_report_callback(hid_keyboard_report_t* report)
{
  processKeyboardReport(report);
}

void processKeyboardReport(hid_keyboard_report_t* report)
{
  // Check with last report to see if there is any changes
  if ( memcmp(&last_kbd_report, report, sizeof(hid_keyboard_report_t)) )
  {
    bool shifted = false;
    
    if ( report->modifier  )
    {
      if ( report->modifier & (KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_RIGHTCTRL) )
      {
        Serial.print("Ctrl ");
      }

      if ( report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT) )
      {
        Serial.print("Shift ");

        shifted = true;
      }

      if ( report->modifier & (KEYBOARD_MODIFIER_LEFTALT | KEYBOARD_MODIFIER_RIGHTALT) )
      {
        Serial.print("Alt ");
      }      
    }
    
    for(uint8_t i=0; i<6; i++)
    {
      uint8_t kc = report->keycode[i];
      char ch = 0;
      
      if ( kc < 128 )
      {
        ch = shifted ? hid_keycode_to_ascii[kc][1] : hid_keycode_to_ascii[kc][0];
      }else
      {
        // non-US keyboard !!??
      }

      // Printable
      if (ch)
      {
        Serial.print(ch);
      }
    }
  }

  // update last report
  memcpy(&last_kbd_report, report, sizeof(hid_keyboard_report_t));  
}
