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
#include <MIDI.h>

// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Bluefruit52"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

BLEDis bledis;
BLEMidi blemidi;

// Create MIDI instance using ble service as physical layer
MIDI_CREATE_BLE_INSTANCE(blemidi);

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 BLEMIDI Example");

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this manually via PIN 19
  Bluefruit.autoConnLed(true);  

  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);
  

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
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

  // Initialize MIDI on channel 4
  MIDI.begin(4);

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

void midi_write_callback(void)
{
  MIDI.read();
}

void loop()
{
  if ( Bluefruit.connected() && blemidi.notifyEnabled() )
  {
    //if (MIDI.read())                    // If we have received a message
    {
        //ledOn(LED_BUILTIN);
        MIDI.sendNoteOn(42, 127, 1);    // Send a Note (pitch 42, velo 127 on channel 1)
        delay(1000);                // Wait for a second
        MIDI.sendNoteOff(42, 0, 1);     // Stop the note
        //ledOff(LED_BUILTIN);
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
