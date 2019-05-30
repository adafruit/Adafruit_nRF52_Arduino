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

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

/* This example  print out Internal Flash contents up to
 * MAX_LEVEL level of directories (including root)
 * WARNING: This example uses recursive call to print out directory tree
 * be extra careful, high number of MAX_LEVEL can cause memory overflow
 */
#define MAX_LEVEL   2

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("InternalFS List Files Example");

  // Initialize Internal File System
  InternalFS.begin();

  // Print whole directory tree of root whose level is 0
  printTreeDir("/", 0);

  // Print prompt
  Serial.println();
  Serial.println("Enter anything to print directory tree:");
}

// the loop function runs over and over again forever
void loop() 
{
  if ( Serial.available() )
  {
    delay(10); // delay for all input arrived
    while( Serial.available() ) Serial.read();

    printTreeDir("/", 0);
    
    // Print prompt
    Serial.println();
    Serial.println("Enter anything to print directory tree:");
  }
}

/**************************************************************************/
/*!
    @brief  Print out whole directory tree of an folder
            until the level reach MAX_LEVEL

    @note   Recursive call
*/
/**************************************************************************/
void printTreeDir(const char* cwd, uint8_t level)
{
  // Open the input folder
  File dir(cwd, FILE_O_READ, InternalFS);

  // Print root
  if (level == 0) Serial.println("root");
 
  // File within folder
  File item(InternalFS);

  // Loop through the directory 
  while( (item = dir.openNextFile(FILE_O_READ)) )
  {
    // Indentation according to dir level
    for(int i=0; i<level; i++) Serial.print("|  ");

    Serial.print("|_ ");
    Serial.print( item.name() );

    if ( item.isDirectory() )
    {
      Serial.println("/");

      // ATTENTION recursive call to print sub folder with level+1 !!!!!!!!
      // High number of MAX_LEVEL can cause memory overflow
      if ( level < MAX_LEVEL )
      {
        char dpath[strlen(cwd) + strlen(item.name()) + 2 ];
        strcpy(dpath, cwd);
        strcat(dpath, "/");
        strcat(dpath, item.name());
        
        printTreeDir( dpath, level+1 );
      }
    }else
    {
      // Print file size starting from position 30
      int pos = level*3 + 3 + strlen(item.name());

      // Print padding
      for (int i=pos; i<30; i++) Serial.print(' ');

      // Print at least one extra space in case current position > 50
      Serial.print(' ');
      
      Serial.print( item.size() );
      Serial.println( " Bytes");
    }

    item.close();
  }

  dir.close();
}
