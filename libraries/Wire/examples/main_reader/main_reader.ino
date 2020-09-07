// Wire Main Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI secondary device
// Refer to the "Wire Secondary Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for main)
  Serial.begin(9600);  // start serial for output
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
}

void loop()
{
  Wire.requestFrom(2, 6);    // request 6 bytes from secondary device #2

  while(Wire.available())    // secondary may send less than requested
  { 
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }

  delay(500);
}
