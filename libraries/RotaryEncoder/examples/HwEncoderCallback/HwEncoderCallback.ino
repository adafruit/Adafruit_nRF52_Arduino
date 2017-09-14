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

#define PIN_A     A0
#define PIN_B     A1

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 Rotary Encoder Callback Example");
  Serial.println("-------------------------------------------\n");

  // Initialize Encoder
  RotaryEncoder.begin(PIN_A, PIN_B);

  // Set callback
  RotaryEncoder.setCallback(encoder_callback);

  // Start encoder
  RotaryEncoder.start();
}

void loop()
{
  // do nothing
}

void encoder_callback(int step)
{
  if ( step > 0 )
  {
    Serial.println("Left");
  }else
  {
    Serial.println("Right");
  }  
}

