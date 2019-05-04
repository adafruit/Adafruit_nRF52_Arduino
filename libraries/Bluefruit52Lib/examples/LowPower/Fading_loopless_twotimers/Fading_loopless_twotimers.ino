/*
  Fading_twotimers

  Fades an LED in fast from 0 to max intensity, then slowly out back to 0, repeatedly.
  
  This example is based on the original Fading sketch. The Adafruit nRF52* boards
  are more advanced than "normal" Arduinos, however, and come with freeRTOS
  and proper task management. This blink sketch demonstrates the use of two
  SoftwareTimers to perform a repetitive delayed task without using the energy
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

// Create SoftwareTimers that will fade our LED.
SoftwareTimer increasetimer, decreasetimer;

int ledPin = LED_RED;    // Red LED connected to a PWM-capable pin.

static int fadeValue = 0;      // Variable that remembers the intensity over function calls.

void setup() {
  increasetimer.begin(10, increasetimer_callback);  // Configure our timers for intervals of 10ms for
  decreasetimer.begin(50, decreasetimer_callback);  // fade in and 50ms for fade out.
  increasetimer.start();                           // Start the increasing timer.

  suspendLoop();  // Since loop() is empty, we should not run it to allow for huge powersaving.
}

void loop() { } // Nothing happens in loop. Don't forget to call suspendLoop()!

void increasetimer_callback(TimerHandle_t _handle) {
  fadeValue += 5; // Increase intensity by 5.
  analogWrite(ledPin, fadeValue);  // Do the actual write 

  // Be careful: If you change the increment above, make sure you actually still
  // hit the following value or change the comparisons.
  if (fadeValue == 255) {      // We've reached the end, so flip direction by switching active timers.
    increasetimer.stop();
    decreasetimer.start();
  }
}

void decreasetimer_callback(TimerHandle_t _handle) {
  fadeValue -= 5; // Increase intensity by 5.
  analogWrite(ledPin, fadeValue);  // Do the actual write 

  // Be careful: If you change the decrement above, make sure you actually still
  // hit the following value or change the comparisons.
  if (fadeValue == 0) {      // We've reached the end, so flip direction by switching active timers.
    decreasetimer.stop();
    increasetimer.start();
  }
}
