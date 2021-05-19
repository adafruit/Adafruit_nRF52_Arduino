/*********************************************************************
 This is an example for our Feather Bluefruit modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch demonstrate how to use Hardware Serial1 along with
 * native USB Serial on Bluefruit nRF52840.
 * Note: Bluefruit nRF52832 does not support Serial1
 */

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Goodnight moon!");
 
  // set the data rate for the SoftwareSerial port
  //mySerial.begin(9600);
  //mySerial.println("Hello, world?");

  Serial1.begin(115200);
  Serial1.println("Hello, world?");
}

void loop() // run over and over//
{
  if (Serial1.available()) 
    Serial.write(Serial1.read());

  if (Serial.available())
    Serial1.write(Serial.read());
}
