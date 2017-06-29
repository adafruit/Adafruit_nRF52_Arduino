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
#define BUTTON_A    31
#define BUTTON_B    30
#define BUTTON_C    27

#define OLED_RESET 4
Adafruit_SSD1306 oled(OLED_RESET);


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
#define ONSCREEN_TIME 5000  // On-screen time for each notification

int  activeIndex  = 0;      // Index of currently displayed notification
int  displayIndex = -1;     // Index of notification about to display

uint32_t drawTime = 0;      // Last time oled display notification

/*------------- BLE Client Service-------------*/
BLEAncs       bleancs;

void setup()
{
  // Button configured
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // init with the I2C addr 0x3C (for the 128x32) and show splashscreen
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();

  oled.setTextSize(1);// max is 4 line, 21 chars each
  oled.setTextColor(WHITE);

  Serial.begin(115200);

  Bluefruit.setName("Bluefruit52"); // set name first
  Bluefruit.begin();
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Service
  bleancs.begin();
  bleancs.setNotificationCallback(ancs_notification_callback);

  // Set up Advertising Packet
  setupAdv();

  // splash screen effect
  delay(100);

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Not connected");
  oled.display();

  // Start Advertising
  Bluefruit.Advertising.start();
}

void setupAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include ANCS 128-bit uuid
  Bluefruit.Advertising.addService(bleancs);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void loop()
{
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
    char tempbuf[22];
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
void connect_callback(void)
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Connected.");
  oled.print("Discovering ... ");
  oled.display();

  if ( bleancs.discover( Bluefruit.connHandle() ) )
  {
    oled.println("OK");

    // ANCS requires pairing to work
    oled.print("Paring      ... ");

    oled.display();

    if ( Bluefruit.requestPairing() )
    {
      oled.println("OK");

      bleancs.enableNotification();
      oled.println("Receiving   ...");
    }else
    {
      oled.println("Failed");
    }
  }else
  {
    oled.println("Failed");
  }

  oled.display();
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

void disconnect_callback(uint8_t reason)
{
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
  uint32_t debounced = ~(*portInputRegister(0));
  debounced &= (bit(BUTTON_A) | bit(BUTTON_B) | bit(BUTTON_C));

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
