/*
  Button_interrupt_ada_callback
  
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
  or long tasks directly in an ISR. This is where deferred ISRs come in. However,
  deferred ISRs cannot have a single time-critical component. By using
  ada_callback_fromISR, you can run a fast time-critical component in a normal
  ISR, and then queue up a callback to process the rest.

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

// Because this variable is modified inside a non-deferred ISR, and used outside it,
// it must be declared volatile to ensure that the most recently written value is
// always used.
volatile int numinterrupts;

// the setup function runs once when you press reset or power the board
void setup(void) {
  Serial.begin(115200); // Not required on the nRF52840 with native USB.

  while (!Serial) {    // Stalls the nRF52840 until the USB serial console has been opened on the host PC.
    delay(10);
  }
  Serial.println("Press the button!");
  Serial.flush();  // Because this sketch does nothing else, it will go to sleep rather than send the
                   // entire Serial buffer to the host PC. So, use Serial.flush() to make sure it does that
                   // before allowing it to sleep.
  
  pinMode(LED_RED, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);   // Configure the switch pin as an input with internal pull-up register enabled.
  
  // Configure it to call switch_callback if the switch pin transitions from HIGH to LOW, i.e. when it is pressed.
  // On the nRF52840, RISING and CHANGE are also valid options. Depending on your platform, LOW may also be available.
  attachInterrupt(interruptPin, switch_isr, FALLING);

  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
}

// the loop function is empty and should never run.
void loop(void) { }

// The switch_isr function is the function we attached to the interrupt on line 63.
// This is known as an Interrupt Service Routine, or ISR.
// It gets run every time the interrupt fires, in the interrupt context. Nothing else
// happens while this is running. Therefore, it should be fast and simple.
// In particular, it should never block, nor do Serial communication or use the Bluefruit API.
// There are a few freeRTOS functions specifically designed to be run from an ISR.
void switch_isr(void) {
  // Do short time-critical processing (setting flags, queueing up tasks, etc) here.
  digitalToggle(LED_RED);
  
  ++numinterrupts;
  ada_callback_fromISR(NULL, 0, switch_isr_callback); // Queue up a task with no extra variables and no arguments.
                                                      // Every single interrupt is serviced, because internally, a
                                                      // queue is used.
}


// The switch_isr_callback function is the function we set up as the ada_callback on line 84.
// It gets run once for every interrupt that switch_isr ran for, but with no guarantees about
// when that happens. However, the advantage is you have the full range of Serial, Bluefruit,
// and freeRTOS API to use now.
// Bear in mind, however, that if this function takes longer to run than the average time between
// interrupts, at some point your ada_callback_queue will overflow. For that scenario, consider
// batch-processing interrupts by using direct-to-task notifications.
void switch_isr_callback(void) {
  Serial.println("Interrupt serviced!");
  Serial.print("The number of interrupts received at this point was ");
  Serial.println(numinterrupts);
  Serial.flush();  // Because this sketch does nothing else, it will go to sleep rather than send the
                   // entire Serial buffer to the host PC. However, here, Serial.flush() doesn't seem
                   // to work either. Maybe something to do with the task context?
}
