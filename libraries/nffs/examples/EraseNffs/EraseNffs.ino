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

#include <Arduino.h>

/* This example erase the content of 28KB User Data starting from 0x6D000
 * (used by NFFS) using low level API sd_flash_page_erase() with SoftDevice
 * DISABLED. In other words, it performs low level formating of NFFS.
 */

#define STARTING_ADDR     0x06D000
#define SECTOR_SIZE       4096
#define ERASE_SIZE        (7*SECTOR_SIZE)
 
// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  Serial.println("Nffs Erasing Example\n");

  for (uint32_t addr = STARTING_ADDR; addr < STARTING_ADDR + ERASE_SIZE; addr += SECTOR_SIZE)
  {
    Serial.printf("Erasing 0x%06X ... ", addr);

    if ( NRF_SUCCESS == sd_flash_page_erase(addr/SECTOR_SIZE) )
    {
      Serial.println("OK");
    }else
    {
      Serial.println("Failed");
      Serial.println("Exit due to ERRORs");
      return;
    }
  }

  Serial.println("Erasing completed");
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_RED);
  delay(1000);
}

