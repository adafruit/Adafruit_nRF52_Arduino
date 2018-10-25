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

#include <FileIO.h>

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10); // for nrf52840 with native usb

  Serial.println("ExternalFS Format Example");
  Serial.println();

  // Wait for user input to run.
  Serial.println("This sketch will destroy all of your data in External Flash!");
  Serial.print("Enter any keys to continue:");
  while ( !Serial.available() )
  {
    delay(1);
  }
  Serial.println();
  Serial.println();

  // Initialize Internal File System
  ExternalFS.begin();

  Serial.print("Formating ...");
  delay(1);
  
  // Format without erase
  ExternalFS.format(true);

  Serial.println("Done");
  delay(1);
}

// the loop function runs over and over again forever
void loop() 
{
  // nothing to do
}
