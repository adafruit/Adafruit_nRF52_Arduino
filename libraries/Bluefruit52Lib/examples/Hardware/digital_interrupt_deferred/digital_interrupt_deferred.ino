/*********************************************************************
 This is an example for our Feather Bluefruit modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch demonstrate how to pass ISR_DEFFERED as additional parameter
 * to defer callback from ISR context with attachInterrupt
 */
#include <Arduino.h>

int interruptPin = A0;

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  pinMode(interruptPin, INPUT_PULLUP);

  // ISR_DEFERRED flag cause the callback to be deferred from ISR context
  // and invoked within a callback thread.
  // It is required to use ISR_DEFERRED if callback function take long time 
  // to run e.g Serial.print() or using any of Bluefruit API() which will
  // potentially call rtos API
  attachInterrupt(interruptPin, digital_callback, ISR_DEFERRED | CHANGE);
}

void loop()
{
  // nothing to do
}

void digital_callback(void)
{
  Serial.print("Pin value: ");
  Serial.println(digitalRead(interruptPin));
}
