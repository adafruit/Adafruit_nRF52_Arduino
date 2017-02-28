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
  bledis.setFirmwareRev("0.1.0");
  bledis.start();
  
  /* Start BLE MIDI
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEMidi::start() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blemidi.start();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEMidi::start() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /* Bluefruit.setConnInterval(9, 12); */  

  // Set up Advertising Packet
  setupAdv();

  // Start Advertising
  Bluefruit.startAdvertising();
}

void setupAdv(void)
{
  //Bluefruit.addAdvTxPower();
  Bluefruit.addAdvFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.addAdvTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.addAdvService(blemidi);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.addScanRespName();
}

void loop()
{
  if ( Bluefruit.connected() && blemidi.configured() )
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
