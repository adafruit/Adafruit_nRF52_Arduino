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

/* This sketch remove the folder that contains the bonding information
 * used by Bluefruit which is "/adafruit/bond"
 */

#include <bluefruit.h>
#include <utility/bonding.h>

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Clear Bonds Example");
  Serial.println("-------------------------------\n");

  Bluefruit.begin();

  Serial.println();
  Serial.println("----- Before -----\n");
  bond_print_list(BLE_GAP_ROLE_PERIPH);
  bond_print_list(BLE_GAP_ROLE_CENTRAL);

  Bluefruit.clearBonds();
  Bluefruit.Central.clearBonds();

  Serial.println();
  Serial.println("----- After  -----\n");
  
  bond_print_list(BLE_GAP_ROLE_PERIPH);
  bond_print_list(BLE_GAP_ROLE_CENTRAL);
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_RED);

  delay(1000);
}

