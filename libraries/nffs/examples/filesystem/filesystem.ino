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

#include <bluefruit.h>
#include <nffs_lib.h>

#define FILENAME    "/adafruit.txt"
#define CONTENTS    "Bluefruit Feather52's NFFS file contents"

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println("nffs example");

  Bluefruit.begin();
  nffs_pkg_init();
  
  struct fs_file *file;

  // file existed
  if ( 0 == fs_open(FILENAME, FS_ACCESS_READ, &file))
  {
    Serial.println(FILENAME " file existed");
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    fs_read(file, sizeof(buffer), buffer, &readlen);

    buffer[readlen] = 0;
    Serial.println(buffer);
  }else
  {
    Serial.print("Open " FILENAME " file to write ... ");

    if( 0 == fs_open(FILENAME, FS_ACCESS_WRITE, &file) )
    {
      Serial.println("OK");
      fs_write(file, CONTENTS, strlen(CONTENTS));
      fs_close(file);
    }else
    {
      Serial.println("Failed (hint: paht must start with '//') ");
    }
  }
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
