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
#include "RotaryEncoder.h"

RotaryEncoder encoder(A0, A1);

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 Hardware Encoder OneKnob Example");
  Serial.println("--------------------------------------------\n");

  // Initialize Encoder
  encoder.begin();

  // Start encoder
  encoder.start();
}

void loop()
{
  int value = encoder.read();

  if (value)
  {
    if ( value > 0 )
    {
      Serial.println("Left");
    }else
    {
      Serial.println("Right");
    }
  }
}
