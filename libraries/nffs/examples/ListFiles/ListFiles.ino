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
#include <Nffs.h>

/* This example  print out NFFS content up to
 * MAX_LEVEL level of directories (including root)
 */

/*
 * WARNING: This example uses recursive call to print out directory tree
 * be extra careful, high number of MAX_LEVEL can cause memory overflow
 */
#define MAX_LEVEL   2

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  Serial.println("Nffs List Files Example");

  // Bluefruit module must be initialized for Nffs to work
  // Since Bluefruit's SOC event handling task is required for flash operation
  Bluefruit.begin();

  // Initialize Nffs
  Nffs.begin();

  // Print whole directory tree of root whose level is 0
  printTreeDir("/", 0);

  // Print prompt
  Serial.println();
  Serial.println("Enter anything to print directory tree (again):");
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
    Serial.println("Enter anything to print directory tree (again):");
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
  NffsDir dir(cwd);

  // Print root
  if (level == 0) Serial.println("root");
 
  // File Entry Information which hold file attribute and name
  NffsDirEntry dirEntry;

  // Loop through the directory
  while( dir.read(&dirEntry) )
  {
    // Indentation according to dir level
    for(int i=0; i<level; i++) Serial.print("|  ");

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

      // ATTENTION recursive call to print sub folder with level+1 !!!!!!!!
      // High number of MAX_LEVEL can cause memory overflow
      if ( level < MAX_LEVEL )
      {
        printTreeDir( fullpath, level+1 );
      }
    }else
    {
      // Print file size starting from position 50
      int pos = level*3 + 3 + strlen(eName);

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
