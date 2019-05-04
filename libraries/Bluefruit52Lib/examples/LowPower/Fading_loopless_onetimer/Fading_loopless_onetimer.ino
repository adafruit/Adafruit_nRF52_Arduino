/*
  Fading_loopless_onetimer

  Fades an LED in from 0 to max intensity, then out back to 0, repeatedly.

  This example is based on the original Fading sketch. The Adafruit nRF52* boards
  are more advanced than "normal" Arduinos, however, and come with freeRTOS
  and proper task management. This blink sketch demonstrates the use of a single
  SoftwareTimer to perform a repetitive delayed task without using the energy
  that a continuously running loop() function has.

  AdaFruit's nRF-boards have multiple on-board LEDs you can control. These are
  addressable using the LED_RED, LED_BLUE, LED_BUILTIN and LED_CONN definitions.
  These take care to use the correct LED pin regardless of which board is used.
  If you want to know what pin the on-board LED is connected to on your model,
  check the Technical Specs of your board on https://www.adafruit.com/ or the
  board's variant.h at
  https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/variants

  Created 2019-05-03
  by Pol Van Aubel


 Based on:

 
 Fading

 This example shows how to fade an LED using the analogWrite() function.

 The circuit:
 * LED attached from digital pin 9 to ground.

 Created 1 Nov 2008
 By David A. Mellis
 modified 30 Aug 2011
 By Tom Igoe

 http://www.arduino.cc/en/Tutorial/Fading

 This example code is in the public domain.

 */

// Create a SoftwareTimer that will resume our loop.
SoftwareTimer fadetimer;

int ledPin = LED_RED;    // Red LED connected to a PWM-capable pin.

void setup() {
  fadetimer.begin(30, fadetimer_callback);  // Configure our timer for intervals of 30 ms.
  fadetimer.start();                     // Start the timer.

  suspendLoop();  // Since loop() is empty, we should not run it to allow for huge powersaving.
}

void loop() { } // Nothing happens in loop. Don't forget to call suspendLoop()!

void fadetimer_callback(TimerHandle_t _handle) {
  static bool increasing = true; // Variable that remembers the direction over function calls.
  static int fadeValue = 0;      // Variable that remembers the intensity over function calls.
  
  if (increasing) {
    fadeValue += 5; // Increase intensity by 5.
  } else {
    fadeValue -= 5; // Decrease intensity by 5.
  }
  
  analogWrite(ledPin, fadeValue);  // Do the actual write 

  // Be careful: If you change the in/decrements above, make sure you actually still
  // hit the following values or change the comparisons.
  if (fadeValue == 0 || fadeValue == 255) {  // We've reached the end, so flip direction.
    increasing = !increasing;  // If increasing = true, this makes it false, and vice versa.
  }
}
