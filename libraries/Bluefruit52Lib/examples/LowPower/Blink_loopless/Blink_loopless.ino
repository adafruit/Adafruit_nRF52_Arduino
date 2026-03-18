/*
  Blink_loopless
  
  Created 2019-05-03
  by Pol Van Aubel
  
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example is based on the original Blink sketch. The Adafruit nRF52* boards
  are more advanced than "normal" Arduinos, however, and come with freeRTOS
  and proper task management. This blink sketch demonstrates the use of
  SoftwareTimer to perform a repetitive delayed task without using the energy
  that a continuously running loop() function has.

  AdaFruit's nRF-boards have multiple on-board LEDs you can control. These are
  addressable using the LED_RED, LED_BLUE, LED_BUILTIN and LED_CONN definitions.
  These take care to use the correct LED pin regardless of which board is used.
  If you want to know what pin the on-board LED is connected to on your model,
  check the Technical Specs of your board on https://www.adafruit.com/ or the
  board's variant.h at
  https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/variants
  

  Based on the original Blink, the code of which is in the public domain.

*/

// Create a SoftwareTimer that will drive our LED.
SoftwareTimer led_timer;

// the setup function runs once when you press reset or power the board
void setup(void) {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_RED, OUTPUT);

  // Set up a repeating timer that fires every half second (500ms) to toggle the LED.
  led_timer.begin(500, timer_callback);
  led_timer.start();

  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
}

void timer_callback(TimerHandle_t _handle) {
  digitalToggle(LED_RED);   // toggle the red LED
}

// the loop function is empty and should never run.
void loop(void) { }
