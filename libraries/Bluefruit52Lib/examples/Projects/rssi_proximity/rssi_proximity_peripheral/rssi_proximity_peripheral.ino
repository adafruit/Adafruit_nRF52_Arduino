/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
 
 Author: KTOWN (Kevin Townsend)
 Copyright (C) Adafruit Industries 2017
*********************************************************************/

/*  This example constantly advertises a custom 128-bit UUID, and is
 *  intended to be used in combination with a Central sketch that scans
 *  for this UUID, and then displays an alert message, sorting matching
 *  devices by their RSSI level which is an approximate indication of
 *  distance (although highly subject to environmental obstacles).
 *  
 *  By including a custom UUID in the advertising packet, we can easily
 *  filter the scan results on the Central device, rather than manually
 *  parsing the advertising packet(s) of every device in range.
 *  
 *  This example is intended to be run with the *_central.ino version
 *  of this application.
 */

#include <bluefruit.h>
#include <ble_gap.h>

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

// Custom UUID used to differentiate this device.
// Use any online UUID generator to generate a valid UUID.
// Note that the byte order is reversed ... CUSTOM_UUID
// below corresponds to the follow value:
// df67ff1a-718f-11e7-8cf7-a6006ad3dba0
const uint8_t CUSTOM_UUID[] =
{
    0xA0, 0xDB, 0xD3, 0x6A, 0x00, 0xA6, 0xF7, 0x8C,
    0xE7, 0x11, 0x8F, 0x71, 0x1A, 0xFF, 0x67, 0xDF
};

BLEUuid uuid = BLEUuid(CUSTOM_UUID);

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Peripheral Proximity Example");
  Serial.println("----------------------------------------\n");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

  if (!Bluefruit.begin())
  {
    Serial.println("Unable to init Bluefruit");
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }
  else
  {
    Serial.println("Bluefruit initialized (peripheral mode)");
  }

  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Set up and start advertising
  startAdv();

  Serial.println("Advertising started"); 
}

void startAdv(void)
{   
  // Note: The entire advertising packet is limited to 31 bytes!
  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Preferred Solution: Add a custom UUID to the advertising payload, which
  // we will look for on the Central side via Bluefruit.Scanner.filterUuid(uuid);
  // A valid 128-bit UUID can be generated online with almost no chance of conflict
  // with another device or etup
  Bluefruit.Advertising.addUuid(uuid);

  /*
  // Alternative Solution: Manufacturer Specific Data (MSD)
  // You could also send a custom MSD payload and filter for the 'Company ID'
  // via 'Bluefruit.Scanner.filterMSD(CID);', although this does require a
  // valid CID, which is why the UUID method above is more appropriate in
  // most situations. For a complete list of valid company IDs see:
  // https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
  // For test purposes, 0xFFFF CAN be used, but according to the Bluetooth SIG:
  // > "This value may be used in the internal and interoperability tests before a
  // >  Company ID has been assigned. This value shall not be used in shipping end
  // >  products."
  uint8_t msd_payload[4]; // Two bytes are required for the CID, so we have 2 bytes user data, expand as needed
  uint16_t msd_cid = 0xFFFF;
  memset(msd_payload, 0, sizeof(msd_payload));
  memcpy(msd_payload, (uint8_t*)&msd_cid, sizeof(msd_cid));
  msd_payload[2] = 0x11;
  msd_payload[3] = 0x22;
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, msd_payload, sizeof(msd_payload));
  */

  // Not enough room in the advertising packet for name
  // so store it in the Scan Response instead
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
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start();
}

void loop() 
{
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}

