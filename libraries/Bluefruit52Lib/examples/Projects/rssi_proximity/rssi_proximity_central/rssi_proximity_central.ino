/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution

 Author: KTOWN (Kevin Townsend)
 Copyright (C) Adafruit Industries 2017
*********************************************************************/

/*  This example scans for advertising devices (peripherals) in range,
 *  looking for a specific UUID in the advertising packet. When this
 *  UUID is found, it will display an alert, sorting devices detected
 *  in range by their RSSI value, which is an approximate indicator of
 *  proximity (though highly dependent on environmental obstacles, etc.).
 *  
 *  A simple 'bubble sort' algorithm is used, along with a simple
 *  set of decisions about whether and where to insert new records
 *  in the record list.
 *  
 *  This example is intended to be used with multiple peripherals
 *  devices running the *_peripheral.ino version of this application.
 *  
 *  VERBOSE_OUTPUT
 *  --------------
 *  Verbose advertising packet analysis can be enabled for
 *  advertising packet contents to help debug issues with the 
 *  advertising payloads, with fully parsed data displayed in the 
 *  Serial Monitor.
 *  
 *  ARRAY_SIZE
 *  ----------
 *  The numbers of peripherals tracked and sorted can be set via the
 *  ARRAY_SIZE macro. Must be at least 2.
 *  
 *  TIMEOUT_MS
 *  ----------
 *  This value determines the number of milliseconds before a tracked
 *  peripheral has it's last sample 'invalidated', such as when a device
 *  is tracked and then goes out of range.
 *  
 *  ENABLE_TFT
 *  ----------
 *  An ILI9341 based TFT can optionally be used to display proximity 
 *  alerts. The Adafruit TFT Feather Wing is recommended and is the only 
 *  device tested with this functionality.
 *  
 *  ENABLE_OLED
 *  -----------
 *  An SSD1306 128x32 based OLED can optionally be used to display
 *  proximity alerts. The Adafruit OLED Feather Wing is recommended and
 *  is the only device tested with this functionality.
 */

#include <string.h>
#include <bluefruit.h>
#include <SPI.h>

#define VERBOSE_OUTPUT (0)    // Set this to 1 for verbose adv packet output to the serial monitor
#define ARRAY_SIZE     (4)    // The number of RSSI values to store and compare
#define TIMEOUT_MS     (2500) // Number of milliseconds before a record is invalidated in the list
#define ENABLE_TFT     (0)    // Set this to 1 to enable ILI9341 TFT display support
#define ENABLE_OLED    (0)    // Set this to 1 to enable SSD1306 128x32 OLED display support

#if (ARRAY_SIZE <= 1)
  #error "ARRAY_SIZE must be at least 2"
#endif
#if (ENABLE_TFT) && (ENABLE_OLED)
  #error "ENABLE_TFT and ENABLE_OLED can not both be set at the same time"
#endif

// Custom UUID used to differentiate this device.
// Use any online UUID generator to generate a valid UUID.
// Note that the byte order is reversed ... CUSTOM_UUID
// below corresponds to the follow value:
// df67ff1a-718f-11e7-8cf7-a6006ad3dba0
const uint8_t CUSTOM_UUID[] =
{
    0xA0, 0xDB, 0xD3, 0x6A, 0x00, 0xA6, 0xF7, 0x8C,
    0xE7, 0x11, 0x8F, 0x71, 0x1A, 0xFF, 0x67, 0xDF
};

BLEUuid uuid = BLEUuid(CUSTOM_UUID);

/* TFT setup if the TFT display is available */
#if (ENABLE_TFT)
  #include <Adafruit_GFX.h>
  #include <Adafruit_ILI9341.h>

   /* Pin setup for the TFT display over SPI */
  #ifdef ARDUINO_NRF52832_FEATHER
     #define TFT_DC   11
     #define TFT_CS   31
     #define STMPE_CS 30
     #define SD_CS    27
  #else
    #error "Unknown target. Please make sure you are using an nRF52 Feather and BSP 0.6.5 or higher!"
  #endif

  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
#endif

/* 128x32 OLED setup if the OLED display is available */
#if (ENABLE_OLED)
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  /* Pin setup for the OLED display */
  #ifdef ARDUINO_NRF52832_FEATHER
    #define BUTTON_A 31
    #define BUTTON_B 30
    #define BUTTON_C 27
  #else
    #error "Unknown target. Please make sure you are using an nRF52 Feather and BSP 0.6.5 or higher!"
  #endif

  Adafruit_SSD1306 oled = Adafruit_SSD1306();
#endif

/* This struct is used to track detected nodes */
typedef struct node_record_s
{
  uint8_t  addr[6];    // Six byte device address
  int8_t   rssi;       // RSSI value
  uint32_t timestamp;  // Timestamp for invalidation purposes
  int8_t   reserved;   // Padding for word alignment
} node_record_t;

node_record_t records[ARRAY_SIZE];

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central Proximity Example");
  Serial.println("-------------------------------------\n");

  /* Enable the TFT display if available */
  #if ENABLE_TFT
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(40, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("DUNKIN");
  #endif

  /* Enable the OLED display if available */
  #if ENABLE_OLED
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();
  #endif

  /* Clear the results list */
  memset(records, 0, sizeof(records));
  for (uint8_t i = 0; i<ARRAY_SIZE; i++)
  {
    // Set all RSSI values to lowest value for comparison purposes,
    // since 0 would be higher than any valid RSSI value
    records[i].rssi = -128;
  }

  /* Enable both peripheral and central modes */
  if ( !Bluefruit.begin(1, 1) )
  {
    Serial.println("Unable to init Bluefruit");
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }
  else
  {
    Serial.println("Bluefruit initialized (central mode)");
  }
  
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);            // Only invoke callback for devices with RSSI >= -80 dBm
  Bluefruit.Scanner.filterUuid(uuid);           // Only invoke callback if the target UUID was found
  //Bluefruit.Scanner.filterMSD(0xFFFF);          // Only invoke callback when MSD is present with the specified Company ID
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  Serial.println("Scanning ...");
}

/* This callback handler is fired every time a valid advertising packet is detected */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  node_record_t record;
  
  /* Prepare the record to try to insert it into the existing record list */
  memcpy(record.addr, report->peer_addr.addr, 6); /* Copy the 6-byte device ADDR */
  record.rssi = report->rssi;                     /* Copy the RSSI value */
  record.timestamp = millis();                    /* Set the timestamp (approximate) */

  /* Attempt to insert the record into the list */
  if (insertRecord(&record) == 1)                 /* Returns 1 if the list was updated */
  {
    printRecordList();                            /* The list was updated, print the new values */
    Serial.println("");
    
    /* Display the device list on the TFT if available */
    #if ENABLE_TFT
    renderResultsToTFT();
    #endif
  
    /* Display the device list on the OLED if available */
    #if ENABLE_OLED
    renderResultsToOLED();
    #endif
  }

/* Fully parse and display the advertising packet to the Serial Monitor
 * if verbose/debug output is requested */
#if VERBOSE_OUTPUT
  uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  /* Display the timestamp and device address */
  if (report->type.scan_response)
  {
    Serial.printf("[SR%10d] Packet received from ", millis());
  }
  else
  {
    Serial.printf("[ADV%9d] Packet received from ", millis());
  }
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.println("");
  
  /* Raw buffer contents */
  Serial.printf("%14s %d bytes\n", "PAYLOAD", report->data.len);
  if (report->data.len)
  {
    Serial.printf("%15s", " ");
    Serial.printBuffer(report->data.p_data, report->data.len, '-');
    Serial.println();
  }

  /* RSSI value */
  Serial.printf("%14s %d dBm\n", "RSSI", report->rssi);

  /* Adv Type */
  Serial.printf("%14s ", "ADV TYPE");
  if ( report->type.connectable )
  {
    Serial.print("Connectable ");
  }else
  {
    Serial.print("Non-connectable ");
  }

  if ( report->type.directed )
  {
    Serial.println("directed");
  }else
  {
    Serial.println("undirected");
  }

  /* Shortened Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s\n", "SHORT NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
  }

  /* Complete Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
  }

  /* TX Power Level */
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %i\n", "TX PWR LEVEL", buffer[0]);
    memset(buffer, 0, sizeof(buffer));
  }

  /* Check for UUID16 Complete List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid16List(buffer, len);
  }

  /* Check for UUID16 More Available List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid16List(buffer, len);
  }

  /* Check for UUID128 Complete List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid128List(buffer, len);
  }

  /* Check for UUID128 More Available List */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE, buffer, sizeof(buffer));
  if ( len )
  {
    printUuid128List(buffer, len);
  }  

  /* Check for BLE UART UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.printf("%14s %s\n", "BLE UART", "UUID Found!");
  }

  /* Check for DIS UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_DEVICE_INFORMATION) )
  {
    Serial.printf("%14s %s\n", "DIS", "UUID Found!");
  }

  /* Check for Manufacturer Specific Data */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (len)
  {
    Serial.printf("%14s ", "MAN SPEC DATA");
    Serial.printBuffer(buffer, len, '-');
    Serial.println();
    memset(buffer, 0, sizeof(buffer));
  }

  Serial.println();
#endif

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

#if ENABLE_TFT
void renderResultsToTFT(void)
{
  tft.fillRect(0, 0, 240, ARRAY_SIZE*8, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  
  for (uint8_t i=0; i<ARRAY_SIZE; i++)
  {
    if (records[i].addr[0] <= 0xF) tft.print("0");
    tft.print(records[i].addr[0], HEX);
    tft.print(":");
    if (records[i].addr[1] <= 0xF) tft.print("0");
    tft.print(records[i].addr[1], HEX);
    tft.print(":");
    if (records[i].addr[2] <= 0xF) tft.print("0");
    tft.print(records[i].addr[2], HEX);
    tft.print(":");
    if (records[i].addr[3] <= 0xF) tft.print("0");
    tft.print(records[i].addr[3], HEX);
    tft.print(":");
    if (records[i].addr[4] <= 0xF) tft.print("0");
    tft.print(records[i].addr[4], HEX);
    tft.print(":");
    if (records[i].addr[5] <= 0xF) tft.print("0");
    tft.print(records[i].addr[5], HEX);
    tft.print(" ");
    tft.print(records[i].rssi);
    tft.print(" [");
    tft.print(records[i].timestamp);
    tft.println("] ");
  }
}
#endif

#if ENABLE_OLED
void renderResultsToOLED(void)
{
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  for (uint8_t i=0; i<ARRAY_SIZE; i++)
  {
    if (i>=4)
    {
      /* We can only display four records on a 128x32 pixel display */
      oled.display();
      return;
    }
    if (records[i].rssi != -128)
    {
      oled.setCursor(0, i*8);
      if (records[i].addr[0] <= 0xF) oled.print("0");
      oled.print(records[i].addr[0], HEX);
      oled.print(":");
      if (records[i].addr[1] <= 0xF) oled.print("0");
      oled.print(records[i].addr[1], HEX);
      oled.print(":");
      if (records[i].addr[2] <= 0xF) oled.print("0");
      oled.print(records[i].addr[2], HEX);
      oled.print(":");
      if (records[i].addr[3] <= 0xF) oled.print("0");
      oled.print(records[i].addr[3], HEX);
      oled.print(":");
      if (records[i].addr[4] <= 0xF) oled.print("0");
      oled.print(records[i].addr[4], HEX);
      oled.print(":");
      if (records[i].addr[5] <= 0xF) oled.print("0");
      oled.print(records[i].addr[5], HEX);
      oled.print(" ");
      oled.println(records[i].rssi);
    }
  }

  oled.display();
}
#endif

/* Prints a UUID16 list to the Serial Monitor */
void printUuid16List(uint8_t* buffer, uint8_t len)
{
  Serial.printf("%14s %s", "16-Bit UUID");
  for(int i=0; i<len; i+=2)
  {
    uint16_t uuid16;
    memcpy(&uuid16, buffer+i, 2);
    Serial.printf("%04X ", uuid16);
  }
  Serial.println();
}

/* Prints a UUID128 list to the Serial Monitor */
void printUuid128List(uint8_t* buffer, uint8_t len)
{
  (void) len;
  Serial.printf("%14s %s", "128-Bit UUID");

  // Print reversed order
  for(int i=0; i<16; i++)
  {
    const char* fm = (i==4 || i==6 || i==8 || i==10) ? "-%02X" : "%02X";
    Serial.printf(fm, buffer[15-i]);
  }

  Serial.println();  
}

/* Prints the current record list to the Serial Monitor */
void printRecordList(void)
{
  for (uint8_t i = 0; i<ARRAY_SIZE; i++)
  {
    Serial.printf("[%i] ", i);
    Serial.printBuffer(records[i].addr, 6, ':');
    Serial.printf(" %i (%u ms)\n", records[i].rssi, records[i].timestamp);
  }
}

/* This function performs a simple bubble sort on the records array */
/* It's slow, but relatively easy to understand */
/* Sorts based on RSSI values, where the strongest signal appears highest in the list */
void bubbleSort(void)
{
  int inner, outer;
  node_record_t temp;

  for(outer=0; outer<ARRAY_SIZE-1; outer++)
  {
    for(inner=outer+1; inner<ARRAY_SIZE; inner++)
    {
      if(records[outer].rssi < records[inner].rssi)
      {
        memcpy((void *)&temp, (void *)&records[outer], sizeof(node_record_t));           // temp=records[outer];
        memcpy((void *)&records[outer], (void *)&records[inner], sizeof(node_record_t)); // records[outer] = records[inner];
        memcpy((void *)&records[inner], (void *)&temp, sizeof(node_record_t));           // records[inner] = temp;
      }
    }
  }
}

/*  This function will check if any records in the list
 *  have expired and need to be invalidated, such as when
 *  a device goes out of range.
 *  
 *  Returns the number of invalidated records, or 0 if
 *  nothing was changed.
 */
int invalidateRecords(void)
{
  uint8_t i;
  int match = 0;

  /* Not enough time has elapsed to avoid an underflow error */
  if (millis() <= TIMEOUT_MS)
  {
    return 0;
  }

  /* Check if any records have expired */
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (records[i].timestamp) // Ignore zero"ed records
    {
      if (millis() - records[i].timestamp >= TIMEOUT_MS)
      {
        /* Record has expired, zero it out */
        memset(&records[i], 0, sizeof(node_record_t));
        records[i].rssi = -128;
        match++;
      }
    }
  }

  /* Resort the list if something was zero'ed out */
  if (match)
  {
    // Serial.printf("Invalidated %i records!\n", match);
    bubbleSort();    
  }

  return match;
}

/* This function attempts to insert the record if it is larger than the smallest valid RSSI entry */
/* Returns 1 if a change was made, otherwise 0 */
int insertRecord(node_record_t *record)
{
  uint8_t i;
  
  /* Invalidate results older than n milliseconds */
  invalidateRecords();
  
  /*  Record Insertion Workflow:
   *  
   *            START
   *              |
   *             \ /
   *        +-------------+
   *  1.    | BUBBLE SORT |   // Put list in known state!
   *        +-------------+
   *              |
   *        _____\ /_____
   *       /    ENTRY    \    YES
   *  2. <  EXISTS W/THIS > ------------------+
   *       \   ADDRESS?  /                    |
   *         -----------                      |
   *              | NO                        |
   *              |                           |
   *       ______\ /______                    |
   *      /      IS       \   YES             |
   *  3. < THERE A ZERO'ED >------------------+
   *      \    RECORD?    /                   |
   *        -------------                     |
   *              | NO                        |
   *              |                           |
   *       ______\ /________                  |
   *     /     IS THE       \ YES             |
   *  4.<  RECORD'S RSSI >=  >----------------|
   *     \ THE LOWEST RSSI? /                 |
   *       ----------------                   |
   *              | NO                        |
   *              |                           |
   *             \ /                         \ /
   *      +---------------+           +----------------+
   *      | IGNORE RECORD |           | REPLACE RECORD |
   *      +---------------+           +----------------+
   *              |                           |
   *              |                          \ /
   *             \ /                  +----------------+
   *             EXIT  <------------- |   BUBBLE SORT  |
   *                                  +----------------+
   */  

  /* 1. Bubble Sort 
   *    This puts the lists in a known state where we can make
   *    certain assumptions about the last record in the array. */
  bubbleSort();

  /* 2. Check for a match on existing device address */
  /*    Replace it if a match is found, then sort */
  uint8_t match = 0;
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (memcmp(records[i].addr, record->addr, 6) == 0)
    {
      match = 1;
    }
    if (match)
    {
      memcpy(&records[i], record, sizeof(node_record_t));
      goto sort_then_exit;
    }
  }

  /* 3. Check for zero'ed records */
  /*    Insert if a zero record is found, then sort */
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (records[i].rssi == -128)
    {
      memcpy(&records[i], record, sizeof(node_record_t));
      goto sort_then_exit;
    }
  }

  /* 4. Check RSSI of the lowest record */
  /*    Replace if >=, then sort */
  if (records[ARRAY_SIZE-1].rssi <= record->rssi)
  {
      memcpy(&records[ARRAY_SIZE-1], record, sizeof(node_record_t));
      goto sort_then_exit;
  }

  /* Nothing to do ... RSSI is lower than the last value, exit and ignore */
  return 0;

sort_then_exit:
  /* Bubble sort */
  bubbleSort();
  return 1;
}

void loop() 
{
  /* Toggle red LED every second */
  digitalToggle(LED_RED);

  /* Invalidate old results once per second in addition
   * to the invalidation in the callback handler. */
  /* ToDo: Update to use a mutex or semaphore since this
   * can lead to list corruption as-is if the scann results
   * callback is fired in the middle of the invalidation
   * function. */
  if (invalidateRecords())
  {
    /* The list was updated, print the new values */
    printRecordList();
    Serial.println("");
    /* Display the device list on the TFT if available */
    #if ENABLE_TFT
    renderResultsToTFT();
    #endif
    /* Display the device list on the OLED if available */
    #if ENABLE_OLED
    renderResultsToOLED();
    #endif
  }
 
  delay(1000);
}
