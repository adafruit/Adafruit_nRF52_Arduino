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

//int pin = PIN_DFU;
int pin = LED_BLUE;

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin(115200);

  analogWrite(LED_RED, 50);
  PWM0.printInfo();

  PWM1.addPin(pin);
  PWM1.enable();
  PWM1.writePin(pin, 50);
  PWM1.start();
  PWM1.printInfo();
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{

}
