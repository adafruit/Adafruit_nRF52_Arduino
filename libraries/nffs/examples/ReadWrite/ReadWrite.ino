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
#include <NewtNffs.h>

#define FILENAME    "/adafruit.txt"
#define CONTENTS    "Bluefruit Feather52's NFFS file contents"

NffsFile file;

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  Serial.println("nffs example");

  Bluefruit.begin();
  Nffs.begin();

  file.open(FILENAME, FS_ACCESS_READ);

  // file existed
  if ( file.existed() )
  {
    Serial.println(FILENAME " file existed");
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    readlen = file.read(buffer, sizeof(buffer));

    buffer[readlen] = 0;
    Serial.println(buffer);
  }else
  {
    Serial.print("Open " FILENAME " file to write ... ");

    if( file.open(FILENAME, FS_ACCESS_WRITE) )
    {
      Serial.println("OK");
      file.write(CONTENTS, strlen(CONTENTS));
      file.close();
    }else
    {
      Serial.println("Failed (hint: path must start with '//') ");
    }
  }
}

// the loop function runs over and over again forever
void loop() 
{
  digitalToggle(LED_BUILTIN);
  delay(1000);
}
