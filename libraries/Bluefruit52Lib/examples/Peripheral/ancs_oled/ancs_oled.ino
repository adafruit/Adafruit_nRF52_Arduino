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
 * This sketch is similar to 'ancs' and use Feather OLED Wing 
 * https://www.adafruit.com/product/2900
 * as the notification display
 */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <bluefruit.h>

#define OLED_RESET 4
Adafruit_SSD1306 oled(OLED_RESET);

#define BUTTON_A    31
#define BUTTON_B    30
#define BUTTON_C    27

#define MAX_COUNT   20
#define BUFSIZE     32

typedef struct
{
  AncsNotification_t ntf;
  char app_name[BUFSIZE];
  char title[BUFSIZE];
  char message[BUFSIZE];
} MyNotif_t;

MyNotif_t myNotifs[MAX_COUNT] = { 0 };

int notifCount  = 0;
int activeIndex = -1;
int nextIndex   = -1;

// BLE Client Service
char deviceModel[BUFSIZE] = { 0 };

BLEClientDis  bleClientDis;
BLEAncs       bleancs;


// General purpose buffer
char buffer[128];

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

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");
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
  // Not connected, wait for a connection
  if ( !Bluefruit.connected() ) return;

  // If service is not yet discovered
  if ( !bleancs.discovered() ) return;

  // Check buttons
  uint32_t presedButtons = readPressedButtons();

//  PRINT_HEX(presedButtons);

  if ( presedButtons & bit(BUTTON_A) )
  {
    if (activeIndex != 0)
    {
      nextIndex = activeIndex - 1;
    }
  }

  if ( presedButtons & bit(BUTTON_B) )
  {
    if (activeIndex != (notifCount-1))
    {
      nextIndex = activeIndex + 1;
    }
  }

  // Only display when index is different
  if ( activeIndex != nextIndex )
  {
    activeIndex = nextIndex;
    displayNotification(activeIndex);
  }
}

/**
 * Display notification contents
 * @param index index of notification
 */
void displayNotification(int index)
{
  // safeguard
  if ( index < 0 || (index >= notifCount) ) return;

  /*-------------Extract data if needed -------------*/
  if ( !strlen(myNotifs[index].app_name) ||
       !strlen(myNotifs[index].title   ) ||
       !strlen(myNotifs[index].message ) )
  {
    uint32_t uid = myNotifs[index].ntf.uid;

    // App Name, Title, Message
    bleancs.getAppName(uid, myNotifs[index].app_name, BUFSIZE);
    bleancs.getTitle  (uid, myNotifs[index].title   , BUFSIZE);
    bleancs.getMessage(uid, myNotifs[index].message , BUFSIZE);
  }

  /*------------- Display to OLED -------------*/
  oled.clearDisplay();
  oled.setCursor(0, 0);

  char tempbuf[32];
  sprintf(tempbuf, "%02d/%02d %15s", index+1, notifCount, deviceModel);

  oled.println(tempbuf);
  oled.println(myNotifs[index].app_name);
  oled.println(myNotifs[index].title);
  oled.println(myNotifs[index].message);

  oled.display();
}

void connect_callback(void)
{
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Connected.");
  oled.print("Discovering ... ");
  oled.display();

  // Discover DIS to read Device Model
  if ( bleClientDis.discover( Bluefruit.connHandle()) )
  {
    bleClientDis.getModel(deviceModel, BUFSIZE);
  }
  
  if ( bleancs.discover( Bluefruit.connHandle() ) )
  {
    oled.println("OK");

    // ANCS requires pairing to work
    oled.print("Paring    ... ");
    oled.display();

    if ( Bluefruit.requestPairing() )
    {
      oled.println("OK");

      bleancs.enableNotification();
      oled.println("Getting Notification");
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

void ancs_notification_callback(AncsNotification_t* notif)
{
  // Clear screen if it is the first notification
  if ( notifCount == 0 )
  {
    // will cause loop() to display
    activeIndex = -1;
    nextIndex   = 0;
  }
  
  if (notif->eventID == ANCS_EVT_NOTIFICATION_ADDED )
  {
    myNotifs[ notifCount++ ].ntf = *notif;
  }else if (notif->eventID == ANCS_EVT_NOTIFICATION_REMOVED )
  {
    for(int i=0; i<notifCount; i++)
    {
      if ( notif->uid == myNotifs[i].ntf.uid )
      {
        // remove by swapping with the last one
        notifCount--;
        myNotifs[i] = myNotifs[notifCount];
        break;
      }
    }
  }else
  {
    // modification: do nothing now !!
  }
  
  oled.display();

  // Automatically accept incoming call using perform Action
//  if ( notif->categoryID == ANCS_CAT_INCOMING_CALL && notif->eventID == ANCS_EVT_NOTIFICATION_ADDED)
//  {
//    Serial.println("Incoming call accepted");
//    bleancs.performAction(notif->uid, ANCS_ACTION_POSITIVE);
//  }
}

void disconnect_callback(uint8_t reason)
{
  (void) reason;

  // reset notification array
  notifCount = 0;
  activeIndex = nextIndex = -1;

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Not connected");
  oled.display();
}
