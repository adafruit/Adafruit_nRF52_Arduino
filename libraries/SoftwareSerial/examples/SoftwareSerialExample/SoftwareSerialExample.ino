/*
  Software serial multiple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin A0 (connect to TX of other device)
 * TX is digital pin A1 (connect to RX of other device)
 
 This example code is in the public domain.
 
 Software Serial will not work when Bluetooth Radio is active because the softdevice
 interrupts the library.

 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1); // RX, TX


void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Goodnight moon!");
 
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop() // run over and over//
{
  if (mySerial.available()) 
    Serial.write(mySerial.read());

  if (Serial.available())
    mySerial.write(Serial.read());
}
