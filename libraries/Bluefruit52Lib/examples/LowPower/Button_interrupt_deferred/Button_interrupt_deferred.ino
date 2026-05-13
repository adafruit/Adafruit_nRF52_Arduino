/*
  Button_interrupt_deferred
  
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

  However, because interrupts need to be fast, you must not do Serial communication
  or long tasks directly in an ISR. This is where deferred ISRs come in.

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
  Serial.begin(115200); // Not required on the nRF52840 with native USB.

  while (!Serial) {    // Stalls the nRF52840 until the USB serial console has been opened on the host PC.
    delay(10);
  }
  Serial.println("Press the button!");
  
  pinMode(LED_RED, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);   // Configure the switch pin as an input with internal pull-up register enabled.
  
  // Configure it to call switch_callback if the switch pin transitions from HIGH to LOW, i.e. when it is pressed.
  // On the nRF52840, RISING and CHANGE are also valid options. Depending on your platform, LOW may also be available.
  // The ISR_DEFERRED flag ensures that the interrupt is serviced by a scheduled operating system task.
  // What this means is that the ISR is allowed to perform slow functionality, including Serial communication,
  // using the Bluefruit API, and the entire freeRTOS API rather than just the *FromISR functions.
  // Use this if your interrupt servicing is not time-critical.
  attachInterrupt(interruptPin, switch_deferred, ISR_DEFERRED | FALLING);

  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
}

// the loop function is empty and should never run.
void loop(void) { }

// The switch_deferred function is the function we attached to the interrupt on line 56.
// This is known as an Interrupt Service Routine, or ISR.
// Because it is a deferred ISR, it is not guaranteed to run immediately after the interrupt
// happens. However, the advantage is you have the full range of Serial, Bluefruit,
// and freeRTOS API to use now.
void switch_deferred(void) {
  digitalToggle(LED_RED);
  Serial.println("Interrupt serviced!");
  Serial.flush();  // Because this sketch does nothing else, it will go to sleep rather than send the
                   // entire Serial buffer to the host PC. So, use Serial.flush() to make sure it does that
                   // before allowing it to sleep.

  // You may notice that you receive multiple messages per press of the switch,
  // This is likely due to a phenomenon physical switches exhibit called "bounce".
  // Effectively this means that the pin goes through multiple transitions of HIGH to LOW,
  // back to HIGH, to LOW again, for a single press of the button.
  // Some debouncing techniques using interrupts are shown in another example.
}
