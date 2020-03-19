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
 * This sketch is similar to 'ancs', but it uses TFT to display
 * incoming ANCS alerts. Supported boards are:
 * - CLUE https://www.adafruit.com/product/4500
 * - Circuit Playground Bluefruit + TFT Gizmo
 *   - https://www.adafruit.com/product/4333
 *   - https://www.adafruit.com/product/4367
 *
 * Button Left: Next or Answer call
 * Button Right: Previous or Reject call
 *
 * Note on CPB button A is RIGHT and button B is LEFT, this is due to
 * the TFT is on the back of the board.
 */
#include <Adafruit_Arcada.h>
#include <bluefruit.h>

Adafruit_Arcada arcada;
Adafruit_SPITFT* tft;

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
  arcada.arcadaBegin();
  arcada.displayBegin();
  arcada.setBacklight(255);

  tft = arcada.display;
  tft->setCursor(0, 0);
  tft->setTextWrap(true);
  tft->setTextSize(2);

  tft->println("Advertising...");

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

  arcada.readButtons();
  uint8_t const justReleased = arcada.justReleasedButtons();
  uint8_t buttonLeft = (justReleased & ARCADA_BUTTONMASK_LEFT);
  uint8_t buttonRight = (justReleased & ARCADA_BUTTONMASK_RIGHT);

#ifdef ARDUINO_NRF52840_CLUE
  // swap button left & right on CLUE since the screen is on opposite side
  uint8_t temp = buttonLeft;
  buttonLeft = buttonRight;
  buttonRight = temp;
#endif

  if ( myNotifs[activeIndex].ntf.categoryID == ANCS_CAT_INCOMING_CALL )
  {
    /* Incoming call event
     * - Button A to accept call
     * - Button B to decline call
     */
    if ( buttonLeft  ) bleancs.actPositive(myNotifs[activeIndex].ntf.uid);
    if ( buttonRight ) bleancs.actNegative(myNotifs[activeIndex].ntf.uid);
  }
  else
  {
    /* Normal events navigation (wrap around)
     * - Button A to display previous notification
     * - Button B to display next notification
     *
     * When a notification is display ONSCREEN_TIME,
     * we will display the next one
     */
    if ( buttonLeft )
    {
      displayIndex = (activeIndex != 0) ? (activeIndex-1) : (notifCount-1) ;
    }

    if ( buttonRight )
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
  digitalWrite(LED_BUILTIN, HIGH);

  /*------------- Display to OLED -------------*/
  MyNotif_t* myNtf = &myNotifs[index];

  tft->fillScreen(ARCADA_BLACK);
  tft->setCursor(0, 0);

  tft->setTextSize(3);
  tft->setTextColor(ARCADA_GREEN);
  tft->println(myNtf->app_name);
  tft->println();

  // Incoming call event, display a bit differently
  if ( myNtf->ntf.categoryID == ANCS_CAT_INCOMING_CALL )
  {
    tft->setTextColor(ARCADA_YELLOW);
    tft->println(myNtf->title);
    tft->println();

    tft->println("calling ...");
    tft->println();

    tft->setTextSize(2);
    tft->setTextColor(ARCADA_GREEN);
    tft->print("< Answer");
    tft->setTextColor(ARCADA_RED);
    tft->println("    Reject >");
  }else
  {
    tft->setTextSize(2);
    tft->setTextColor(ARCADA_GREENYELLOW);
    tft->printf("%02d/%02d\n", index+1, notifCount);
    tft->println();

    tft->setTextColor(ARCADA_YELLOW);
    tft->println(myNtf->title);
    tft->println();

    tft->setTextColor(ARCADA_WHITE);
    tft->println(myNtf->message);
    tft->println();
  }

  digitalWrite(LED_BUILTIN, LOW);
}

/**
 * Connect Callback
 *  Perform ANCS discovering, request Pairing
 */
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);

  tft->println("Connected");
  tft->print("Discovering ... ");

  if ( bleancs.discover( conn_handle ) )
  {
    tft->println("OK");

    // ANCS requires secured connection
    // request Pairing if not bonded
    tft->print("Paring ... ");
    conn->requestPairing();
  }else
  {
    // disconnect if couldn't find ancs service
    conn->disconnect();
    tft->println("Failed");
  }
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

    tft->println("Secured");

    if ( bleancs.discovered() )
    {
      bleancs.enableNotification();
      tft->println("Ready to receive");
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
    if ( bleancs.getTitle(uid, myNtf->title, BUFSIZE) )
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

  tft->fillScreen(ARCADA_BLACK);
  tft->setCursor(0, 0);
  tft->println("Not connected");
}
