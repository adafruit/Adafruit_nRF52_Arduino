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

/* SoftwareTimer is a helper class that uses FreeRTOS software timer
 * to invoke callback. Its periodic timing is flexible as opposed to
 * hardware timer and cannot be faster than rtos's tick which is configured
 * at  ~1 ms interval.
 *
 * If you need an strict interval timing, or faster frequency, check out
 * the hw_systick sketch example that use hardware systick timer.
 *
 * http://www.freertos.org/RTOS-software-timer.html
 */
SoftwareTimer blinkTimer;


void setup()
{
  // Configure the timer with 1000 ms interval, with our callback
  blinkTimer.begin(1000, blink_timer_callback);

  // Start the timer
  blinkTimer.start();
}

void loop()
{
  // do nothing here
}


/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 *
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  // freeRTOS timer ID, ignored if not used
  (void) xTimerID;

  digitalToggle(LED_RED);
}
