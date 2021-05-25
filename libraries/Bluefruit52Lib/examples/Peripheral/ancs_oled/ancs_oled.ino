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

/*
 * This sketch is similar to 'ancs', but it also uses a Feather OLED
 * Wing to display incoming ANCS alerts:
 * https://www.adafruit.com/product/2900
 *
 * BUTTON A: Up or accept call
 * BUTTON B: Not used since it is hard to press
 * BUTTON C: Down or decline call
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <bluefruit.h>

/*------------- OLED and Buttons -------------*/
#if defined ARDUINO_NRF52832_FEATHER
  // Feather nRF52832
  #define BUTTON_A    31
  #define BUTTON_C    27

#else
  // Default for others
  #define BUTTON_A    9
  #define BUTTON_C    5

#endif

Adafruit_SSD1306 oled(-1);

/*------------- Notification List -------------*/
#define MAX_COUNT   20
#define BUFSIZE     64

typedef struct
{
  AncsNotification_t ntf;
  char title[BUFSIZE];
  char message[BUFSIZE];
  char app_name[BUFSIZE];
} MyNotif_t;

MyNotif_t myNotifs[MAX_COUNT] = { 0 };

// Number of notifications
int notifCount = 0;

/*------------- Display Management -------------*/
#define ONSCREEN_TIME 10000  // On-screen time for each notification

int  activeIndex  = 0;      // Index of currently displayed notification
int  displayIndex = -1;     // Index of notification about to display

uint32_t drawTime = 0;      // Last time oled display notification

/*------------- BLE Client Service-------------*/
BLEAncs       bleancs;

void setup()
{
  // Button configured
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // init with the I2C addr 0x3C (for the 128x32) and show splashscreen
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  oled.display();

  oled.setTextSize(1);// max is 4 line, 21 chars each
  oled.setTextColor(WHITE);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Set connection secured callback, invoked when connection is encrypted
  Bluefruit.Security.setSecuredCallback(connection_secured_callback);

  // Configure and Start Service
  bleancs.begin();
  bleancs.setNotificationCallback(ancs_notification_callback);

  // Set up and start advertising
  startAdv();

  // splash screen effect
  delay(100);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Not connected");
  oled.display();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include ANCS 128-bit uuid
  Bluefruit.Advertising.addService(bleancs);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{
  // This example only support 1 connection
  uint16_t const conn_handle = 0;
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  // connection exist, connected, and secured
  if ( !(conn && conn->connected() && conn->secured()) ) return;

  // If service is not yet discovered
  if ( !bleancs.discovered() ) return;

  // No notifications, do nothing
  if ( notifCount == 0 ) return;

  // Check buttons
  uint32_t presedButtons = readPressedButtons();

  if ( myNotifs[activeIndex].ntf.categoryID == ANCS_CAT_INCOMING_CALL )
  {
    /* Incoming call event
     * - Button A to accept call
     * - Button C to decline call
     */
    if ( presedButtons & bit(BUTTON_A) )
    {
      bleancs.actPositive(myNotifs[activeIndex].ntf.uid);
    }

    if ( presedButtons & bit(BUTTON_C) )
    {
      bleancs.actNegative(myNotifs[activeIndex].ntf.uid);
    }
  }
  else
  {
    /* Normal events navigation (wrap around)
     * - Button A to display previous notification
     * - Button C to display next notification
     *
     * When a notification is display ONSCREEN_TIME,
     * we will display the next one
     */
    if ( presedButtons & bit(BUTTON_A) )
    {
      displayIndex = (activeIndex != 0) ? (activeIndex-1) : (notifCount-1) ;
    }

    if ( presedButtons & bit(BUTTON_C) )
    {
      displayIndex = (activeIndex != (notifCount-1)) ? (activeIndex + 1) : 0;
    }

    // Display requested notification
    if ( displayIndex >= 0 )
    {
      activeIndex = displayIndex;
      displayIndex = -1;

      displayNotification(activeIndex);
      drawTime = millis(); // Save time we draw
    }
    // Display next notification if time is up
    else if ( drawTime + ONSCREEN_TIME < millis() )
    {
      activeIndex = (activeIndex+1)%notifCount;

      displayNotification(activeIndex);
      drawTime = millis(); // Save time we draw
    }
  }
}

/**
 * Display notification contents to oled screen
 * @param index index of notification
 */
void displayNotification(int index)
{
  // safeguard
  if ( index < 0 || (index >= notifCount) ) return;

  // let's Turn on and off RED LED when we draw to get attention
  digitalWrite(LED_RED, HIGH);

  /*------------- Display to OLED -------------*/
  MyNotif_t* myNtf = &myNotifs[index];

  oled.clearDisplay();
  oled.setCursor(0, 0);

  // Incoming call event, display a bit differently
  if ( myNtf->ntf.categoryID == ANCS_CAT_INCOMING_CALL )
  {
    oled.println(myNtf->title);
    oled.println("          is calling");
    oled.println("  Btn A to ACCEPT");
    oled.println("  Btn C to DECLINE");
  }else
  {
    // Text size = 1, max char is 21. Text size = 2, max char is 10
    char tempbuf[30];
    sprintf(tempbuf, "%-15s %02d/%02d", myNtf->app_name, index+1, notifCount);

    oled.println(tempbuf);
    oled.println(myNtf->title);

    oled.print("  ");
    oled.print(myNtf->message);
  }

  oled.display();

  digitalWrite(LED_RED, LOW);
}

/**
 * Connect Callback
 *  Perform ANCS discovering, request Pairing
 */
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Connected.");
  oled.print("Discovering ... ");
  oled.display();
  
  if ( bleancs.discover( conn_handle ) )
  {
    oled.println("OK");

    // ANCS requires pairing to work
    // request Pairing if not bonded
    oled.println("Paring      ... ");
    conn->requestPairing();
  }else
  {
    oled.println("Failed");
  }

  oled.display();
}

void connection_secured_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  if ( !conn->secured() )
  {
    // It is possible that connection is still not secured by this time.
    // This happens (central only) when we try to encrypt connection using stored bond keys
    // but peer reject it (probably it remove its stored key).
    // Therefore we will request an pairing again --> callback again when encrypted
    conn->requestPairing();
  }
  else
  {
    Serial.println("Secured");

    if ( bleancs.discovered() )
    {
      bleancs.enableNotification();
      oled.println("Ready to receive");
      oled.display();
    }
  }
}

/**
 * Notification callback
 * @param notif Notification from iDevice
 *
 * Save/Modify notification into myNotifs struct to display later
 */
void ancs_notification_callback(AncsNotification_t* notif)
{
  if (notif->eventID == ANCS_EVT_NOTIFICATION_ADDED )
  {
    myNotifs[ notifCount ].ntf = *notif;

    /*------------- Retrieve Title, Message, App Name -------------*/
    MyNotif_t* myNtf = &myNotifs[notifCount];
    uint32_t uid = myNtf->ntf.uid;

    // iDevice often include Unicode "Bidirection Text Control" in the Title.
    // Mostly are U+202D as beginning and U+202C as ending. Let's remove them
    if ( bleancs.getTitle  (uid, myNtf->title   , BUFSIZE) )
    {
      char u202D[3] = { 0xE2, 0x80, 0xAD }; // U+202D in UTF-8
      char u202C[3] = { 0xE2, 0x80, 0xAC }; // U+202C in UTF-8

      int len = strlen(myNtf->title);

      if ( 0 == memcmp(&myNtf->title[len-3], u202C, 3) )
      {
        len -= 3;
        myNtf->title[len] = 0; // chop ending U+202C
      }

      if ( 0 == memcmp(myNtf->title, u202D, 3) )
      {
        memmove(myNtf->title, myNtf->title+3, len-2); // move null-terminator as well
      }
    }

    bleancs.getMessage(uid, myNtf->message , BUFSIZE);
    bleancs.getAppName(uid, myNtf->app_name, BUFSIZE);

    displayIndex = notifCount++; // display new notification
  }else if (notif->eventID == ANCS_EVT_NOTIFICATION_REMOVED )
  {
    for(int i=0; i<notifCount; i++)
    {
      if ( notif->uid == myNotifs[i].ntf.uid )
      {
        // remove by swapping with the last one
        notifCount--;
        myNotifs[i] = myNotifs[notifCount];

        // Invalid removed data
        memset(&myNotifs[notifCount], 0, sizeof(MyNotif_t));

        if (activeIndex == notifCount)
        {
          // If remove the last notification, adjust display index
          displayIndex = notifCount-1;
        }else if (activeIndex == i)
        {
          // Re-draw if remove currently active one
          displayIndex = activeIndex;
        }

        break;
      }
    }
  }else
  {
    // Modification
    for(int i=0; i<notifCount; i++)
    {
      if ( notif->uid == myNotifs[i].ntf.uid )
      {
        // Display modification
        displayIndex = i;
        break;
      }
    }
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  // reset notification array
  notifCount   = 0;
  activeIndex  = 0;
  displayIndex = -1;

  memset(myNotifs, 0, sizeof(myNotifs));

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Not connected");
  oled.display();
}

/**
 * Check if button A,B,C state are pressed, include some software
 * debouncing.
 *
 * Note: Only set bit when Button is state change from
 * idle -> pressed. Press and hold only report 1 time, release
 * won't report as well
 *
 * @return Bitmask of pressed buttons e.g If BUTTON_A is pressed
 * bit 31 will be set.
 */
uint32_t readPressedButtons(void)
{
  // must be exponent of 2
  enum { MAX_CHECKS = 8, SAMPLE_TIME = 10 };

  /* Array that maintains bounce status/, which is sampled
   * 10 ms each. Debounced state is regconized if all the values
   * of a button has the same value (bit set or clear)
   */
  static uint32_t lastReadTime = 0;
  static uint32_t states[MAX_CHECKS] = { 0 };
  static uint32_t index = 0;

  // Last Debounced state, used to detect changed
  static uint32_t lastDebounced = 0;

  // Too soon, nothing to do
  if (millis() - lastReadTime < SAMPLE_TIME ) return 0;

  lastReadTime = millis();

  // Take current read and masked with BUTTONs
  // Note: Bitwise inverted since buttons are active (pressed) LOW
  uint32_t debounced = ~( (digitalRead(BUTTON_A) << BUTTON_A) | (digitalRead(BUTTON_C) << BUTTON_C) );

  // Copy current state into array
  states[ (index & (MAX_CHECKS-1)) ] = debounced;
  index++;

  // Bitwise And all the state in the array together to get the result
  // This means pin must stay at least MAX_CHECKS time to be realized as changed
  for(int i=0; i<MAX_CHECKS; i++)
  {
    debounced &= states[i];
  }

  // result is button changed and current debounced is set
  // Mean button is pressed (idle previously)
  uint32_t result = (debounced ^ lastDebounced) & debounced;

  lastDebounced = debounced;

  return result;
}

