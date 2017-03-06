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

#define STATUS_LED  (17)
#define BLINKY_MS   (2000)

/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37
 * Body Sensor Location Char:   0x2A38
 */
BLEService        hrms = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmc = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic bslc = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

uint32_t blinkyms;
uint8_t  bps = 0;

// Advanced function prototypes
void setupAdv(void);
void setupHRM(void);
void connect_callback(void);
void disconnect_callback(uint8_t reason);
void cccd_callback(BLECharacteristic& chr, ble_gatts_evt_write_t* request);

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 HRM Example");
  Serial.println("-----------------------");

  // Setup LED pins and reset blinky counter
  pinMode(STATUS_LED, OUTPUT);
  blinkyms = millis();

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name to 'Feather52 HRM'");
  Bluefruit.setName("Feather52 HRM");

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.start();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.start();
  blebas.update(100);

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Heart Rate Monitor Service");
  setupHRM();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  setupAdv();

  // Start Advertising
  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(hrms);

  // There isn't enough room in the advertising packet for the
  // name so we'll place it on the secondary Scan Response packet
  Bluefruit.ScanResponse.addName();
}

void setupHRM(void)
{
  // Configure the Heart Rate Monitor service
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.heart_rate.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Heart Rate Measurement       0x2A37  Mandatory   Notify
  // Body Sensor Location         0x2A38  Optional    Read
  // Heart Rate Control Point     0x2A39  Conditional Write       <-- Not used here
  hrms.start();

  // Note: You must call .start() on the BLEService before calling .start() on
  // any characteristic(s) within that service definition.. Calling .start() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'start()'ed!

  // Configure the Heart Rate Measurement characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml
  // Permission = Notify
  // Min Len    = 1
  // Max Len    = 8
  //    B0      = UINT8  - Flag (MANDATORY)
  //      b5:7  = Reserved
  //      b4    = RR-Internal (0 = Not present, 1 = Present)
  //      b3    = Energy expended status (0 = Not present, 1 = Present)
  //      b1:2  = Sensor contact status (0+1 = Not supported, 2 = Supported but contact not detected, 3 = Supported and detected)
  //      b0    = Value format (0 = UINT8, 1 = UINT16)
  //    B1      = UINT8  - 8-bit heart rate measurement value in BPM
  //    B2:3    = UINT16 - 16-bit heart rate measurement value in BPM
  //    B4:5    = UINT16 - Energy expended in joules
  //    B6:7    = UINT16 - RR Internal (1/1024 second resolution)
  hrmc.setProperties(CHR_PROPS_NOTIFY);
  hrmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  hrmc.setFixedLen(2);
  hrmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  hrmc.start();
  uint8_t hrmdata[2] = { 0b00000110, 0x40 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  hrmc.notify(hrmdata, 2);                   // Use .notify instead of .write!

  // Configure the Body Sensor Location characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.body_sensor_location.xml
  // Permission = Read
  // Min Len    = 1
  // Max Len    = 1
  //    B0      = UINT8 - Body Sensor Location
  //      0     = Other
  //      1     = Chest
  //      2     = Wrist
  //      3     = Finger
  //      4     = Hand
  //      5     = Ear Lobe
  //      6     = Foot
  //      7:255 = Reserved
  bslc.setProperties(CHR_PROPS_READ);
  bslc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bslc.setFixedLen(1);
  bslc.start();
  bslc.write(2);    // Set the characteristic to 'Wrist' (2)
}

void connect_callback(void)
{
  Serial.println("Connected");
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");
}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr.uuid == hrmc.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Heart Rate Measurement 'Notify' enabled");
        } else {
            Serial.println("Heart Rate Measurement 'Notify' disabled");
        }
    }
}

void loop()
{
  // Blinky!
  if (blinkyms+BLINKY_MS < millis()) {
    blinkyms = millis();
    digitalToggle(STATUS_LED);

    if (Bluefruit.connected()) {
      uint8_t hrmdata[2] = { 0b00000110, bps++ };           // Sensor connected, increment BPS value
      err_t resp = hrmc.notify(hrmdata, sizeof(hrmdata));   // Note: We use .notify instead of .write!

      // This isn't strictly necessary, but you can check the result
      // of the .notify() attempt to see if it was successful or not
      switch (resp) {
        case ERROR_NONE:
          // Value was written correctly!
          Serial.print("Heart Rate Measurement updated to: "); Serial.println(bps);
          break;
        case NRF_ERROR_INVALID_PARAM:
          // Characteristic property not set to 'Notify'
          Serial.println("ERROR: Characteristic 'Property' not set to Notify!");
          break;
        case NRF_ERROR_INVALID_STATE:
          // Notify bit not set in the CCCD or not connected
          Serial.println("ERROR: Notify not set in the CCCD or not connected!");
          break;
        default:
          // Unhandled error code
          Serial.print("ERROR: Ox"); Serial.println(resp, HEX);
          break;
      }
    }
  }
}
