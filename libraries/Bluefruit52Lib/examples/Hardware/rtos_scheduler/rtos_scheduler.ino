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
#include <Adafruit_TinyUSB.h> // for Serial

/*
 * Sketch demonstate mutli-task using Scheduler. Demo create loop2() that
 * run in 'parallel' with loop().
 * - loop() toggle LED_RED every 1 second
 * - loop2() toggle LED_BLUE every half of second
 */

void setup() 
{
  // LED_RED & LED_BLUE pin already initialized as an output.
  
  // Create loop2() using Scheduler to run in 'parallel' with loop()
  Scheduler.startLoop(loop2);
}

/**
 * Toggle led1 every 1 second
 */
void loop() 
{
  digitalToggle(LED_RED); // Toggle LED 
  delay(1000);            // wait for a second
}

/**
 * Toggle led1 every 0.5 second
 */
void loop2()
{
  digitalToggle(LED_BLUE); // Toggle LED 
  delay(500);              // wait for a half second  
}

