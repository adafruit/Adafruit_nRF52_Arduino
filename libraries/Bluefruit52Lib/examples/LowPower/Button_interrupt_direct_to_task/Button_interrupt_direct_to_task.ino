/*
  Button_interrupt_direct_to_task
  
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
  deferred ISRs cannot have a single time-critical component. ada_callback_fromISR,
  queues up a task for every interrupt fired. Sometimes it's okay or even desirable
  to service multiple interrupts in a single run of the task. This sketch demonstrates
  how to take direct control of this, by using startLoop and the freeRTOS
  direct-to-task notification API.

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
// Best results for this sketch are achieved with a very bouncy button.
int interruptPin = 6;

TaskHandle_t switch_isr_task_handle;


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

  // Set up a task to act on the interrupts.
  // startLoop is a wrapper around freeRTOS's xTaskCreate that takes care of infinitely
  // looping inside the function being called, just like loop(). If you need to do setup
  // that should not be repeated, e.g. if you want to keep an incrementing counter inside
  // the task, use the freeRTOS API directly.
  //Scheduler.startLoop(switch_isr_task);
  // TODO Cannot use startLoop because we need the handle for vTaskNotifyGiveFromISR().


  // Set up a task to act on the interrupts. We schedule it so it only happens if there
  // is absolutely nothing else to do.
  if (xTaskCreate(switch_isr_task, "ISRprocessor", SCHEDULER_STACK_SIZE_DFLT, NULL, TASK_PRIO_LOW, &switch_isr_task_handle) != pdPASS) {
    Serial.println("Unable to create task for the ISR. This is going to crash!");
    Serial.flush();
    delay(1000);
  }
  
  // Configure it to call switch_callback if the switch pin transitions from HIGH to LOW, i.e. when it is pressed.
  // On the nRF52840, RISING and CHANGE are also valid options. Depending on your platform, LOW may also be available.
  attachInterrupt(interruptPin, switch_isr, FALLING);

  // Since loop() is empty, suspend its task so that the system never runs it
  // and can go to sleep properly.
  suspendLoop();
}

// the loop function is empty and should never run.
void loop(void) { } // We could actually use the loop task instead of switch_isr_task.

// The switch_isr function is the function we attached to the interrupt on line 61.
// This is known as an Interrupt Service Routine, or ISR.
// It gets run every time the interrupt fires, in the interrupt context. Nothing else
// happens while this is running. Therefore, it should be fast and simple.
// In particular, it should never block, nor do Serial communication or use the Bluefruit API.
// There are a few freeRTOS functions specifically designed to be run from an ISR.
void switch_isr(void) {
  // Do short time-critical processing (setting flags, queueing up tasks, etc) here.
  digitalToggle(LED_RED);

  vTaskNotifyGiveFromISR(switch_isr_task_handle, NULL);  // Increment the notification value of the task.
}


// The switch_isr_task function is the function we set up as the task on line 72.
// It gets run when the scheduler can schedule it, and then processes all interrupts in one go.
// You have the full range of Serial, Bluefruit, and freeRTOS API to use here.
void switch_isr_task(void* ignored) {
  uint32_t pulNotificationValue = 0;
  static uint32_t clear_mask = 0xFFFFFFFF;
  while (1) {
    xTaskNotifyWait(0, clear_mask, &pulNotificationValue, DELAY_FOREVER);  // Block the task running this function until it gets a notification.
  
    Serial.println("Interrupt serviced!");
    Serial.print("The number of interrupts serviced by this invocation was ");
    Serial.println(pulNotificationValue);
    Serial.flush();  // Because this sketch does nothing else, it will go to sleep rather than send the
                     // entire Serial buffer to the host PC. However, here, Serial.flush() doesn't seem
                     // to work either. Maybe something to do with the task context?
  }
}
