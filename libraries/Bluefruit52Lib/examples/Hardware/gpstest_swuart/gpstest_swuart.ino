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

/* This example show how to use Software Serial on Bluefruit nRF52
 * to interact with GPS FeatherWing https://www.adafruit.com/product/3133
 * 
 * Hardware Set up
 * - Connect 3V and GND to GPS wing
 * - 
 */

#include <SoftwareSerial.h>

#define SW_RXD    A0
#define SW_TXD    A1

// Declare an Software Serial instance
SoftwareSerial mySerial(SW_RXD, SW_TXD);

void setup() {

  // Init hardware UART <-> Serial Monitor
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("GPS echo test");

  // Init Software Uart <-> GPS FeatherWing
  mySerial.begin(9600);      // default NMEA GPS baud
}

     
void loop() {

  // Pass data from Serial (HW uart) to GPS Wing (SW Uart)
  if (Serial.available()) {
    char c = Serial.read();
    mySerial.write(c);
  }

  // Pass data from GPS Wing (SW Uart) to Serial (HW uart)
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
}
