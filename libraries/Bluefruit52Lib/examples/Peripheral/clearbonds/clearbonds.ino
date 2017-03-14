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
#include <ApacheNffs.h>

void setup() 
{
  Serial.begin(115200);
  Serial.println("Bluefruit52 Clear Bonds Example");

  Bluefruit.begin();

  Serial.println();
  Serial.println("----- Before -----\n");
  printDir("/adafruit/bond");

  Bluefruit.clearBonds();

  Serial.println();
  Serial.println("----- After  -----\n");
  printDir("/adafruit/bond");
}

void loop() 
{
  // Toggle both LEDs every 1 second
  digitalToggle(LED_BUILTIN);

  delay(1000);
}

void printDir(const char* cwd)
{
  // Open the input folder
  NffsDir dir(cwd);

  Serial.println(cwd);
 
  // File Entry Information which hold file attribute and name
  NffsDirEntry dirEntry;

  // Loop through the directory
  while( dir.read(&dirEntry) )
  {
    // Indentation
    Serial.print("|_ ");

    char eName[64];
    dirEntry.getName(eName, sizeof(eName));

    char fullpath[256];
    strcpy(fullpath, cwd);
    strcat(fullpath, "/");
    strcat(fullpath, eName);
    
    Serial.print( eName );

    if ( dirEntry.isDirectory() )
    {
      Serial.println("/");
    }else
    {
      // Print file size starting from position 50
      int pos = 3 + strlen(eName);

      // Print padding
      for (int i=pos; i<50; i++) Serial.print(' ');

      // Print at least one extra space in case current position > 50
      Serial.print(' ');

      NffsFile file(fullpath);

      Serial.print( file.size() );
      Serial.println( " Bytes");

      file.close();
    }
  }

  dir.close();
}
