/*
  SendText.ino
  
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  This example for the Arduino Primo board shows how to use
  NFC library.
  It sets a text message specifying the language code, then
  starts the module, so that when a device with NFC is near
  to the board the message "Hello World!" will be sent.

  This example code is in the public domain.
  
*/

#include <NFC.h>

void setup() { 
  // set the NFC message as first parameter and the language code as second
  NFC.setTXTmessage("Hello World!", "en");
  // start the NFC module
  NFC.start();
}

void loop() {

}
