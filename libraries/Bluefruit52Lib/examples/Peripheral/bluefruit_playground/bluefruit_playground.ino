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

/* This sketch demonstrate the BLE Adafruit Service that is used with
 * "Adafruit Bluefruit Playground" app. Supported boards are
 *  - Circuit Playground Bluefruit (https://www.adafruit.com/product/4333)
 *  - CLUE nRF52840 (https://www.adafruit.com/product/4500)
 */

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <bluefruit.h>
#include <BLEAdafruitService.h>

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

// Adafruit Service: ADAFxx-C332-42A8-93BD-25E905756CB8
BLEAdafruitTemperature  bleTemp;
BLEAdafruitAccel        bleAccel;
BLEAdafruitLightSensor  bleLight;
BLEAdafruitButton       bleButton;
BLEAdafruitTone         bleTone;

BLEAdafruitAddressablePixel     blePixel;

// Circuit Playground Bluefruit
#if defined ARDUINO_NRF52840_CIRCUITPLAY
  #include <Adafruit_CircuitPlayground.h>

  #define DEVICE_NAME       "CPlay"
  #define NEOPIXEL_COUNT    10


#elif defined ARDUINO_NRF52840_CLUE
  #include <Adafruit_APDS9960.h>
  #include <Adafruit_LSM6DS33.h>

  #define DEVICE_NAME       "CLUE"
  #define NEOPIXEL_COUNT    1

  Adafruit_APDS9960 apds9960;
  Adafruit_LSM6DS33 lsm6ds33;
#else
  #error "Board is not supported"
#endif

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint16_t measure_button(uint8_t* buf, uint16_t bufsize)
{
  uint32_t button = 0;

#if defined ARDUINO_NRF52840_CIRCUITPLAY
  button |= ( CircuitPlayground.slideSwitch() ? 0x01 : 0x00 );
  button |= ( CircuitPlayground.leftButton()  ? 0x02 : 0x00 );
  button |= ( CircuitPlayground.rightButton() ? 0x04 : 0x00 );

#else
  // Button is active LOW on most board except CPlay

  // No slide switch
  button |= ( digitalRead(PIN_BUTTON1) ? 0x00 : 0x02 );
  button |= ( digitalRead(PIN_BUTTON2) ? 0x00 : 0x04 );
#endif

  memcpy(buf, &button, 4);
  return 4;
}

uint16_t measure_temperature(uint8_t* buf, uint16_t bufsize)
{
#if defined ARDUINO_NRF52840_CIRCUITPLAY
  float temp = CircuitPlayground.temperature();
  memcpy(buf, &temp, 4);
#endif
  return 4;
}

uint16_t measure_light_sensor(uint8_t* buf, uint16_t bufsize)
{ 
  float lux;

#if defined ARDUINO_NRF52840_CIRCUITPLAY
  lux = CircuitPlayground.lightSensor();
#else
  uint16_t r, g, b, c;
  apds9960.getColorData(&r, &g, &b, &c);

  lux = c;
#endif

  memcpy(buf, &lux, 4);
  return 4;
}

uint16_t measure_accel(uint8_t* buf, uint16_t bufsize)
{  
  float* accel_buf = (float*) buf;

#if defined ARDUINO_NRF52840_CIRCUITPLAY
  accel_buf[0] = CircuitPlayground.motionX();
  accel_buf[1] = CircuitPlayground.motionY();
  accel_buf[2] = CircuitPlayground.motionZ();
#else

  sensors_event_t accel, gyro, temp;
  (void) gyro; (void) temp;

  lsm6ds33.getEvent(&accel, &gyro, &temp);

  accel_buf[0] = accel.acceleration.x;
  accel_buf[1] = accel.acceleration.y;
  accel_buf[2] = accel.acceleration.z;
#endif

  return 3*sizeof(float); // 12
}

void setup()
{
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

#if defined ARDUINO_NRF52840_CIRCUITPLAY
  CircuitPlayground.begin();
#else

  // Button
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);

  // Buzzer Speaker
  pinMode(PIN_BUZZER, OUTPUT);

  // Light sensor
  apds9960.begin();
  apds9960.enableColor(true);

  // Accelerometer
  lsm6ds33.begin_I2C();
#endif



  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(false);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName(DEVICE_NAME);
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Adafruit Service
  bleTemp.begin();
  bleTemp.setMeasureCallback(measure_temperature);
  
  bleAccel.begin();
  bleAccel.setMeasureCallback(measure_accel);
  
  bleLight.begin();
  bleLight.setMeasureCallback(measure_light_sensor);
    
  bleButton.begin();
  bleButton.setMeasureCallback(measure_button);
  bleButton.setPeriod(0); // only notify if there is changes with buttons

  bleTone.begin(PIN_BUZZER);

  strip.begin();
  blePixel.begin(&strip);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertising with only board ID
  struct ATTR_PACKED {
    uint16_t mfr_id;
    
    uint8_t  field_len;
    uint16_t field_key;
    uint16_t field_value;
  } mfr_adv;

  mfr_adv.mfr_id = UUID16_COMPANY_ID_ADAFRUIT;
  mfr_adv.field_len = 4;
  mfr_adv.field_key = 1; // board id
  mfr_adv.field_value = USB_PID;

  Bluefruit.Advertising.addManufacturerData(&mfr_adv, sizeof(mfr_adv));

  // Add name to advertising, since there is enough room
  Bluefruit.Advertising.addName();
  
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

}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
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

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
