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

// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Bluefruit52"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

BLEDis bledis;
BLEMidi blemidi;

void setup()
{
  Serial.begin(115200);

  Serial.println("Bluefruit52 BLEMIDI Example");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.setHardwareRev("Rev E");
  bledis.setFirmwareRev(ARDUINO_BSP_VERSION);  // ARDUINO_BSP_VERSION is BSP Libraries version
  bledis.start();
  
  /* Start BLE MIDI
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEMidi::start() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blemidi.setWriteCallback(midi_write_callback);
  blemidi.start();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEMidi::start() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /* Bluefruit.setConnInterval(9, 12); */  

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  //Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(blemidi);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void midi_write_callback(uint32_t tstamp, uint8_t data[])
{
  Serial.printf("Time %d: ", tstamp);
  Serial.printBuffer(data, 3);
  Serial.println();
}

void loop()
{
  if ( Bluefruit.connected() && blemidi.notifyEnabled() )
  {
    static int current_note = 60;
    
    // send note on
    blemidi.send(0x90, current_note, 0x64);
    delay(500);

    ledToggle(LED_BUILTIN);

    // send note off
    blemidi.send(0x80, current_note, 0x64);
    delay(500);

    // increment note pitch
    current_note++;

    // only do one octave
    if(current_note > 72)
    {
      current_note = 60;
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
  
  Serial.println("Disconnected");
  Serial.println("Bluefruit will auto start advertising (default)");
}
