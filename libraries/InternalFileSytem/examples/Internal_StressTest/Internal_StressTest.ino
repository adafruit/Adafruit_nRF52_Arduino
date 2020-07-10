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

// hardware random generator
#include "nrf_rng.h"

using namespace Adafruit_LittleFS_Namespace;

/* This example perform a stress test on Little FileSystem on internal flash.
 * There are 4 different thread sharing same loop() code with different priority
 *    loop (low), normal, high, highest
 * Each will open and write a file of its name (+ .txt). Task takes turn writing until timeout
 * to print out the summary
 */

// timeout in seconds
#define TIME_OUT        60

// Limit number of writes since our internal flash is limited (28 KB in total)
#define WRITE_COUNT   2000

uint32_t writeCount = 0;

// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) yield();   // for nrf52840 with native usb

  Serial.println("Internal Stress Test Example");
  yield();

  // Initialize Internal File System
  InternalFS.begin();

  // Format
  Serial.print("Formatting ... "); Serial.flush();
  InternalFS.format();
  Serial.println("Done"); Serial.flush();

  // Create a folder for each thread
  InternalFS.mkdir("high");
  InternalFS.mkdir("n1");
  InternalFS.mkdir("n2");
  InternalFS.mkdir("n3");
  InternalFS.mkdir("loop");

  // Create thread with different priority
  // Although all the thread share loop() code, they are separated threads
  // and running with different priorities

  // Note: default loop() is running at LOW
  Scheduler.startLoop(loop, 1024, TASK_PRIO_HIGH  , "high");
  Scheduler.startLoop(loop, 1024, TASK_PRIO_NORMAL, "n1");
  Scheduler.startLoop(loop, 1024, TASK_PRIO_NORMAL, "n2");
  Scheduler.startLoop(loop, 1024, TASK_PRIO_NORMAL, "n3");
}

void write_files(const char * name)
{
  char fname[30] = { 0 };
  sprintf(fname, "%s/%s.txt", name, name); // each task has its own folder and file

  File file(InternalFS);

  if ( file.open(fname, FILE_O_WRITE) )
  {
    file.printf("%d\n", writeCount++);
    file.close();
  }else
  {
    Serial.printf("Failed to open %s\n", fname);
  }
}

void list_files(void)
{
  File root("/", FILE_O_READ, InternalFS);
  File subdir(InternalFS);
  File file(InternalFS);

  while( (subdir = root.openNextFile(FILE_O_READ)) )
  {
    if ( subdir.isDirectory() )
    {
      char fname[30];
      sprintf(fname, "%s/%s.txt", subdir.name(), subdir.name());

      if ( file.open(fname, FILE_O_READ) )
      {
        Serial.printf("--- %s ---\n", fname);

        while ( file.available() )
        {
          char buffer[64] = { 0 };
          file.read(buffer, sizeof(buffer)-1);

          Serial.print(buffer);
          delay(10);
        }
        file.close();

        Serial.println("---------------\n");
      }
    }

    subdir.close();
  }
}

// Use random generator (requires Softdevice disabled)
uint16_t get_rand(void)
{
  uint8_t bytes[2];

  for(int i=0; i<2; i++)
  {
    nrf_rng_event_clear(NRF_RNG, NRF_RNG_EVENT_VALRDY);
    nrf_rng_task_trigger(NRF_RNG, NRF_RNG_TASK_START);

    while ( !nrf_rng_event_check(NRF_RNG, NRF_RNG_EVENT_VALRDY) ) yield();

    bytes[i] = nrf_rng_random_value_get(NRF_RNG);
  }

  return (bytes[0] << 8) + bytes[1];
}

// the loop function runs over and over again forever
void loop() 
{
  TaskHandle_t th = xTaskGetCurrentTaskHandle();

  if ( (millis() > TIME_OUT*1000) || (writeCount > WRITE_COUNT) )
  {
    // low priority task print summary
    if (TASK_PRIO_LOW == uxTaskPriorityGet(th))
    {
      Serial.printf("Total write count = %d in %.2f seconds\n", writeCount, millis()/1000.0);
      list_files();
    }

    delay(1000);
    vTaskSuspend(NULL); // suspend task
    return;
  }

  const char* name = pcTaskGetName(th);
  Serial.printf( "Task %s writing ...\n", name );
  Serial.flush();

  // Write files
  write_files(name);

  // lower delay increase chance for high prio task preempt others.
  // delay is from 0-5000
  uint16_t ms = get_rand() % 5000;
  delay(ms);
}
