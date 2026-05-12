/*
  SendText.ino
  
  Written by Chiara Ruggeri (chiara@arduino.org)
  
  This example for the Arduino Primo board shows how to use
  NFC library.
  It sets a app message specifying the package name (for Android)
  and the ID application (for Windows phone), then starts the
  module, so that when a device with NFC is near to the board
  it will try to open the application (if present) or will
  look for the app in the store. Finally it register a callback
  function that will be called any time an NFC field is detected
  (it means that a device is near). 

  This example code is in the public domain.
  
*/

#include <NFC.h>

//specify the package name for windows and android phone and insert the EOL character at the end '\0'
static const char android_package_name[] = {'n', 'o', '.', 'n', 'o', 'r', 'd', 'i', 'c', 's',
                                               'e', 'm', 'i', '.', 'a', 'n', 'd', 'r', 'o', 'i',
                                               'd', '.', 'n', 'r', 'f', 't', 'o', 'o', 'l', 'b',
                                               'o', 'x', '\0'};

static const char windows_application_id[] = {'{', 'e', '1', '2', 'd', '2', 'd', 'a', '7', '-',
                                                 '4', '8', '8', '5', '-', '4', '0', '0', 'f', '-',
                                                 'b', 'c', 'd', '4', '-', '6', 'c', 'b', 'd', '5',
                                                 'b', '8', 'c', 'f', '6', '2', 'c', '}', '\0'};

void setup() {
  Serial.begin(9600);
  NFC.setAPPmessage(android_package_name, windows_application_id);
  NFC.start();
  NFC.registerCallback(myFunction);
}


void loop() {
}

void myFunction(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t dataLength){
    (void)context;

    switch (event)
    {
        case NFC_T2T_EVENT_FIELD_ON:
            Serial.println("******NFC_T2T_EVENT_FIELD_ON******");
            break;

        case NFC_T2T_EVENT_FIELD_OFF:
            Serial.println("------NFC_T2T_EVENT_FIELD_OFF------");
            break;

        default:
            break;
    }
}
