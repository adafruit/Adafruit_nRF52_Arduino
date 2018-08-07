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
  ledToggle(LED_RED);
}

} // extern C

void setup()
{
  // Set up systick to fire 10 times per second
  SysTick_Config(F_CPU/10);
}

void loop()
{
  // do nothing here
}
