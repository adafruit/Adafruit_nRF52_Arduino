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

// Interval between systick event
#define TICK_INTERVAL_MS    50

// Note: Extern "C" is required since all the IRQ hardware handler is
// declared as "C function" within the startup (assembly) file.
// Without it, our SysTick_Handler will be declared as "C++ function"
// which is not the same as the "C function" in startup even it has
// the same name.
extern "C"
{

/* This is hardware interupt service function exectuing in non-RTOS thread
 * Function implementation should be quick and short if possible.
 * 
 * WARNING: This function MUST NOT call any blocking FreeRTOS API 
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void SysTick_Handler(void)
{
  digitalToggle(LED_RED);
}

} // extern C

void setup()
{
  /* Input parameter is number of ticks between interrupts handler i.e SysTick_Handler
   * 1000 ms --> F_CPU            ticks
   * T    ms --> (F_CPU/1000)*T   ticks
   *
   * Note: Since systick is 24-bit timer, the max tick value is 0xFFFFFF, F_CPU = 64 Mhz
   * --> our Tmax = 0xFFFFFF/64000 ~ 262 ms
   */

  SysTick_Config( (F_CPU/1000)*TICK_INTERVAL_MS );
}

void loop()
{
  // do nothing here
}
