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

/* This sketches demontrate Bluefruit.Advertising API(). When power up
 * Bluefruit will advertising for ADV_TIMEOUT seconds (30 seconds in fast mode, 
 * the lesft in slow mode) and then stopped completely. Bluefruit only start
 * advertising again if PIN_ADV is grounded.
 */
#include <bluefruit.h>

#define PIN_ADV       11
#define ADV_TIMEOUT   60 // seconds

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

void setup() 
{
  // configure pin as input
  pinMode(PIN_ADV, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println("Bluefruit52 Blinky Example");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

  Bluefruit.begin();
  Bluefruit.setName("Bluefruit52");

  // Set up Advertising Packet
  setupAdv();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertising forever
   */
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  Bluefruit.Advertising.startIfDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);      // Stop advertising after ADV_TIMEOUT seconds

  Serial.println("Advertising is started");
}

void setupAdv(void)
{  
  //Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
}

void loop() 
{
  // Only check pin only when we stopped already
  if ( !Bluefruit.Advertising.isAdvertising() )
  {
    // Check if Pin is grounded
    if ( digitalRead(PIN_ADV) == 0 )
    {
      Bluefruit.Advertising.start(ADV_TIMEOUT);
      Serial.println("Advertising is started");
    }
  }
}

/**
 * Callback invoked when advertising is stopped by timeout
 */
void adv_stop_callback(void)
{
  Serial.println("Advertising timeout and stopped");
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}

