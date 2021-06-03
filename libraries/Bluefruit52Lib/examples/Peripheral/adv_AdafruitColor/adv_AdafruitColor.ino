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

/* This sketch demonstrates the Bluefruit.Advertising API(). When powered up,
 * the Bluefruit module will start advertising a packet compatible with the
 * AdafruitColor class from CircuitPython:
 * https://github.com/adafruit/Adafruit_CircuitPython_BLE/blob/master/adafruit_ble/advertising/adafruit.py
 * In other words, `ble.start_scan(AdafruitColor)` in CircuitPython will hear
 * an advertisement containing the color 0x0F0033 (violet) from this device.
 */
 
#include <bluefruit.h>

#define ADV_TIMEOUT   0 // seconds. Set this higher to automatically stop advertising after a time

// The following code is for setting a name based on the actual device MAC address
// Where to go looking in memory for the MAC
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define MAC_ADDRESS_HIGH  (*(pREG32 (0x100000a8)))
#define MAC_ADDRESS_LOW   (*(pREG32 (0x100000a4)))

void byte_to_str(char* buff, uint8_t val) {  // convert an 8-bit byte to a string of 2 hexadecimal characters
  buff[0] = nibble_to_hex(val >> 4);
  buff[1] = nibble_to_hex(val);
}

char nibble_to_hex(uint8_t nibble) {  // convert a 4-bit nibble to a hexadecimal character
  nibble &= 0xF;
  return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}

void setup() 
{
//  Serial.begin(115200);
//  while ( !Serial ) delay(10);

  Serial.println("Bluefruit52 Color Advertising Example");
  Serial.println("----------------------------------------\n");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  char ble_name[14] = "BluefruitXXXX"; // Null-terminated string must be 1 longer than you set it, for the null

  // Replace the XXXX with the lowest two bytes of the MAC Address
  // The commented lines show you how to get the WHOLE MAC address
//  uint32_t addr_high = ((MAC_ADDRESS_HIGH) & 0x0000ffff) | 0x0000c000;
  uint32_t addr_low  = MAC_ADDRESS_LOW;
//  Serial.print("MAC Address: ");
//  Serial.print((addr_high >> 8) & 0xFF, HEX); Serial.print(":");
//  Serial.print((addr_high) & 0xFF, HEX); Serial.print(":");
//  Serial.print((addr_low >> 24) & 0xFF, HEX); Serial.print(":");
//  Serial.print((addr_low >> 16) & 0xFF, HEX); Serial.print(":");
//  Serial.print((addr_low >> 8) & 0xFF, HEX); Serial.print(":");
//  Serial.print((addr_low) & 0xFF, HEX); Serial.println("");

  // Fill in the XXXX in ble_name
  byte_to_str(&ble_name[9], (addr_low >> 8) & 0xFF);
  byte_to_str(&ble_name[11], addr_low & 0xFF);
  // Set the name we just made
  Bluefruit.setName(ble_name);

  // start advertising
  startAdv();
  Serial.print("Advertising is started: ");
  Serial.println(ble_name);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);

  // This is the data format that will match CircuitPython's AdafruitColor
  struct ATTR_PACKED
  {
    uint16_t manufacturer;
    uint8_t  color_len;
    uint16_t  color_type;
    uint8_t  color_info[4];
  } color_data =
  {
      .manufacturer = 0x0822,                   // Adafruit company ID
      .color_len  = sizeof(color_data) - 3,     // length of data to follow
      .color_type = 0x0000,                     // Type specifier of this field, which AdafruitColor defines as 0x0000
      .color_info = { 0x33, 0x00, 0x0F, 0x00 }, // { 0xBB, 0xGG, 0xRR, 0xIgnore}
  };
  // BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA is 0xFF
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &color_data, sizeof(color_data));

  // Tell the BLE device we want to send our name in a ScanResponse if asked.
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
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);      // Stop advertising entirely after ADV_TIMEOUT seconds 
}

void loop() 
{

}

/**
 * Callback invoked when advertising is stopped by timeout
 */
void adv_stop_callback(void)
{
  Serial.println("Advertising time passed, advertising will now stop.");
}
