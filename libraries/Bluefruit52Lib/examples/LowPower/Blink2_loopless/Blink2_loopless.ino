/*
  Blink_loopless
  
  Created 2019-05-03
  by Pol Van Aubel
  
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example is based on the original Blink sketch. The Adafruit nRF52* boards
  are more advanced than "normal" Arduinos, however, and come with freeRTOS
  and proper task management. This blink sketch demonstrates the use of two
  SoftwareTimers to perform two repetitive delayed tasks at different frequencies
  without using the energy that a continuously running loop() function has.

  AdaFruit's nRF-boards have multiple on-board LEDs you can control. These are
  addressable using the LED_RED, LED_BLUE, LED_BUILTIN and LED_CONN definitions.
  These take care to use the correct LED pin regardless of which board is used.
  If you want to know what pin the on-board LED is connected to on your model,
  check the Technical Specs of your board on https://www.adafruit.com/ or the
  board's variant.h at
  https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/variants
  

  Based on the original Blink, the code of which is in the public domain.
  */

#include <Arduino.h>

// Create the two SoftwareTimers we're going to use.
SoftwareTimer bluetimer, redtimer;

void setup()
{
  // LED_RED & LED_BLUE are already initialized as outputs.

  // Initialize the timers at .75 seconds for blue, and 1 second for red.
  bluetimer.begin(750, bluetoggle);
  redtimer.begin(1000, redtoggle);
  bluetimer.start();
  redtimer.start();
  suspendLoop();
}

void loop(void) { }

/**
 * Toggle led1 every 1 second
 */
void redtoggle(TimerHandle_t _handle) 
{
  digitalToggle(LED_RED); // Toggle LED 
}

/**
 * Toggle led1 every 0.5 second
 */
void bluetoggle(TimerHandle_t _handle)
{
  digitalToggle(LED_BLUE); // Toggle LED 
}
