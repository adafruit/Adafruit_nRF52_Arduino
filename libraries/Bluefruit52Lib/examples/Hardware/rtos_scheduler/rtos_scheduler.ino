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

/*
 * Sketch demonstate mutli-task using Scheduler. Demo create loop2() that
 * run in 'parallel' with loop().
 * - loop() toggle LED1 every 1 second
 * - loop2() toggle LED2 every half of second
 */

int led1 = 17;
int led2 = 19;

void setup() 
{
  // initialize digital pin as an output.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

  // Create loop2() using Scheduler
  Scheduler.startLoop(loop2);
}

/**
 * Toggle led1 every 1 second
 */
void loop() 
{
  digitalToggle(led1); // Toggle LED 
  delay(1000);         // wait for a second
}

/**
 * Toggle led1 every 0.5 second
 */
void loop2()
{
  digitalToggle(led2); // Toggle LED 
  delay(500);          // wait for a half second  
}

