/*
  Button_interrupt
  
  Created 2019-05-04
  by Pol Van Aubel
  
  Uses an interrupt to listen for a button input. On the nRF52840, it can listen
  on pin 7 to the UserSW. On other boards, an external switch needs to be provided.

  The Adafruit nRF52* boards are more advanced than "normal" Arduinos and come
  with freeRTOS, interrupts available on every pin, and proper task management.
  This button sketch demonstrates the use of interrupts to act on an input such
  as a switch, without having to poll it in a continuously scanning loop() function.
  Even though human button presses are slow, and therefore unlikely to be missed
  using a scanning loop, the interrupt technique allows the board to go into sleep
  mode while waiting for an input, thereby saving a lot of energy.

  AdaFruit's nRF-boards have multiple on-board LEDs you can control. These are
  addressable using the LED_RED, LED_BLUE, LED_BUILTIN and LED_CONN definitions.
  These take care to use the correct LED pin regardless of which board is used.
  If you want to know what pin the on-board LED is connected to on your model,
  check the Technical Specs of your board on https://www.adafruit.com/ or the
  board's variant.h at
  https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/variants
  

  This code is licensed under CC0, effectively putting it in the Public Domain
  where possible.

*/

// On the nRF52840, pin 7 is the UserSW/DFU switch.
int interruptPin = 7;

// the setup function runs once when you press reset or power the board
void setup(void) {
  pinMode(LED_RED, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);   // Configure the switch pin as an input with internal pull-up register enabled.
  
  // Configure it to call switch_callback if the switch pin transitions from HIGH to LOW, i.e. when it is pressed.
  // On the nRF52840, RISING and CHANGE are also valid options. Depending on your platform, LOW may also be available.
  attachInterrupt(interruptPin, switch_callback, FALLING);

  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
}

// the loop function is empty and should never run.
void loop(void) { }

// The switch_callback function is the function we attached to the interrupt on line 42.
// This is known as an Interrupt Service Routine, or ISR.
// It gets run every time the interrupt fires, in the interrupt context. Nothing else
// happens while this is running. Therefore, it should be fast and simple.
// In particular, it should never block, nor do Serial communication or use the Bluefruit API.
// There are a few freeRTOS functions specifically designed to be run from an ISR.
// If you need a longer ISR, other examples show how to do this.
void switch_callback(void) {
  digitalToggle(LED_RED);   // toggle the red LED

  // You may notice that the LED does not reliably turn on on one press of the switch,
  // then off on the next. This is likely due to a phenomenon physical switches exhibit called "bounce".
  // Effectively this means that the pin goes through multiple transitions of HIGH to LOW,
  // back to HIGH, to LOW again, for a single press of the button.
  // Some debouncing techniques using interrupts are shown in another example.
}
