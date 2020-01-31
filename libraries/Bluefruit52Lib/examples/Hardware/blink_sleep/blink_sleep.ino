/*
  Blink_sleep
  Turns on an LED on for one second, then off for one second, until the time to go to sleep runs out.
  You can wake up the feather by pressing a key/switch connecting the WAKE_LOW_PIN to GND and the WAKE_HIGH_PIN to 3.3V (VCC)

  Based on the blinky arduino example 
  
  This example code is in the public domain.

  created 19 Jan 2010
  by Pierre Constantineau

*/
#include <bluefruit.h>

#define WAKE_LOW_PIN  PIN_A0
#define WAKE_HIGH_PIN PIN_A1

#define SLEEPING_DELAY 30000                                // sleep after 30 seconds of blinking

void gotoSleep(unsigned long time)
{
  // shutdown when time reaches SLEEPING_DELAY ms
  if ((time>SLEEPING_DELAY))
  {
    // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
    digitalWrite(LED_BUILTIN, LOW);                     

    // setup your wake-up pins.
    pinMode(WAKE_LOW_PIN,  INPUT_PULLUP_SENSE);    // this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
    pinMode(WAKE_HIGH_PIN, INPUT_PULLDOWN_SENSE);  // this pin (WAKE_HIGH_PIN) is pulled down and wakes up the feather when externally connected to 3.3v.
 
    // power down nrf52.
    sd_power_system_off();                              // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
  } 
}

// the setup function runs once when you press reset or power the board
void setup() {
  Bluefruit.begin();          // Sleep functions need the softdevice to be active.

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}
     
// the loop function runs over and over again forever
void loop() {
  digitalToggle(LED_BUILTIN); // turn the LED on (HIGH is the voltage level)
  gotoSleep(millis());        // call millis() and pass it to the sleep function.  On wake-up, millis will start at 0 again.
  delay(1000);                // wait for a second
}
