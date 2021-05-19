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

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

const int baudrate = 115200;

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin (baudrate);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Serial Echo demo");
  Serial.print("Badurate : ");
  Serial.println(baudrate);
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{ 
  // From Serial monitor to All
  if ( Serial.available() )
  {
    Serial.write( Serial.read() );
  }
}
