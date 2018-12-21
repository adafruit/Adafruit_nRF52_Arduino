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
 * This sketch demonstrate the central API(). A additional bluefruit
 * that has bleuart as peripheral is required for the demo.
 */
#include <bluefruit.h>
#include <MIDI.h>

bool MIDILOG = false;

BLEClientDis  clientDis;
BLEClientMidi clientMidi;

MIDI_CREATE_BLE_INSTANCE(clientMidi);

int clockPin = 7;

void setup()
{
  pinMode(clockPin, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central BLEUART Example");
  Serial.println("-----------------------------------\n");
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  
  Bluefruit.setName("Bluefruit52 Central");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientMidi.begin();
  clientMidi.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds

  //try Midi Things
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientMidi) )
  {
    Serial.print("BLE MIDI service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }else
  {      
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
  Serial.println("Connected");

  Serial.print("Dicovering DIS ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }  

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientMidi.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientMidi.enableTXD();

    Serial.println("Ready to receive from peripheral");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
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
  
  Serial.println("Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientMidi
 */
void bleuart_rx_callback(BLEClientMidi& uart_svc)
{
  //Serial.print("[RX]: ");

 
  if ( uart_svc.available() && Serial.available())
  {
      //delay(2);
      MIDI.read();
//    Serial.print( (char) uart_svc.read() );
  }
//
//  Serial.println();
}

bool on16 = true;

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  if(MIDILOG){
    // Log when a note is pressed.
    Serial.printf("Note on: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
    Serial.println();
  }

  //make any note event on channel 16 a trig out
  if(channel == 16 && !on16){
    on16 = true;
    digitalWrite(LED_RED, HIGH);
    digitalWrite(clockPin, HIGH);
    delay(5);
    digitalWrite(LED_RED, LOW);
    digitalWrite(clockPin, LOW);
    
  }

}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  if(MIDILOG){
    // Log when a note is released.
    Serial.printf("Note off: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
    Serial.println();
  }

  if(channel == 16){
    on16 = false;
  }
}

void loop()
{
  if ( Bluefruit.Central.connected() )
  {
    MIDI.read();
    // Not discovered yet
    if ( clientMidi.discovered() )
    {
      // Discovered means in working state
      // Get Serial input and send to Peripheral
//      if ( Serial.available() )
//      {
//        delay(2); // delay a bit for all characters to arrive
//        
//        char str[20+1] = { 0 };
//        Serial.readBytes(str, 20);
//        
//        clientMidi.print( str );
//      }

      
    }
  }
}
