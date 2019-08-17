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
#include "IEEE11073float.h"

/* Health Thermometer Service Definitions
 * Health Thermometer Service:  0x1809
 * Temperature Measurement Char: 0x2A1C
 */
BLEService        htms = BLEService(UUID16_SVC_HEALTH_THERMOMETER);
BLECharacteristic htmc = BLECharacteristic(UUID16_CHR_TEMPERATURE_MEASUREMENT);

BLEDis bledis;    // DIS (Device Information Service) helper class instance

double  tempvalue = 0;

// Advanced function prototypes
void startAdv(void);
void setupHTM(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Heath Thermometer Example");
  Serial.println("-------------------------------------\n");

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();;

  // Setup the Heath Thermometer service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Heath Thermometer Service");
  setupHTM();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HTM Service UUID
  Bluefruit.Advertising.addService(htms);

  // Include Name
  Bluefruit.Advertising.addName();
  
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

void setupHTM(void)
{
  // Configure the Health Thermometer service
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.health_thermometer.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Temperature Measurement      0x2A1C  Mandatory   Indicate
  //
  // Temperature Type             0x2A1D  Optional    Read                  <-- Not used here
  // Intermediate Temperature     0x2A1E  Optional    Read, Notify          <-- Not used here
  // Measurement Interval         0x2A21  Optional    Read, Write, Indicate <-- Not used here
  htms.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Temperature Measurement characteristic
  // See:https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_measurement.xml
  // Properties = Indicte
  // Min Len    = 6
  // Max Len    = 6
  //    B0      = UINT8  - Flag (MANDATORY)
  //      b3:7  = Reserved
  //      b2    = Temperature Type Flag (0 = Not present, 1 = Present)
  //      b1    = Timestamp Flag (0 = Not present, 1 = Present)
  //      b0    = Unit Flag (0 = Celsius, 1 = Fahrenheit)
  //    B4:1    = FLOAT  - IEEE-11073 32-bit FLOAT measurement value
  //    B5      = Temperature Type
  htmc.setProperties(CHR_PROPS_INDICATE);
  htmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  htmc.setFixedLen(6);
  htmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  htmc.begin();
  uint8_t htmdata[6] = { 0b00000101, 0, 0 ,0 ,0, 2 }; // Set the characteristic to use Fahrenheit, with type (body) but no timestamp field
  htmc.write(htmdata, sizeof(htmdata));                    // Use .write for init data

  // Temperature Type Value
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_type.xml
  //    B0      = UINT8 - Temperature Type
  //      0     = Reserved
  //      1     = Armpit
  //      2     = Body (general)
  //      3     = Ear (usually ear lobe)
  //      4     = Finger
  //      5     = Gastro-intestinal Tract
  //      6     = Mouth
  //      7     = Rectum
  //      8     = Toe
  //      9     = Tympanum (ear drum)
  //     10:255 = Reserved
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == htmc.uuid) {
        if (chr->indicateEnabled(conn_hdl)) {
            Serial.println("Temperature Measurement 'Indicate' enabled");
        } else {
            Serial.println("Temperature Measurement 'Indicate' disabled");
        }
    }
}

void loop()
{
  digitalToggle(LED_RED);
  
  if ( Bluefruit.connected() ) {
    uint8_t htmdata[6] = { 0b00000101, 0,0,0,0, 2 }; // Fahrenheit unit, temperature type = body (2)

    float2IEEE11073(tempvalue, &htmdata[1]);
    
    // Note: We use .indicate instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although indicate is not sent
    if ( htmc.indicate(htmdata, sizeof(htmdata)) ){
      Serial.print("Temperature Measurement updated to: "); Serial.println(tempvalue); 
    }else{
      Serial.println("ERROR: Indicate not set in the CCCD or not connected!");
    }

    tempvalue += 0.5F;
  }

  // Only send update once per second
  delay(1000);
}
