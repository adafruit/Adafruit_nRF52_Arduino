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

/* This sketch implement Arduino Science Journal firmware using Adafruit Bluefruit library
 * https://www.arduino.cc/education/science-journal.
 * Supported boards are:
 *  - Circuit Playground Bluefruit : https://www.adafruit.com/product/4333
 *  - CLUE nRF52840 : https://www.adafruit.com/product/4500
 *  - Feather Sense : https://www.adafruit.com/product/4516
 */

#include <bluefruit.h>
#include <PDM.h>

const int VERSION = 0x00000001;
const float TEMPERATURE_CALIBRATION = -5.0;

// 555a0002-val-467a-9538-01f0652c74e8"
#define SCIENCE_KIT_UUID(val) \
  (const uint8_t[]) { \
    0xe8, 0x74, 0x2c, 0x65, 0xf0, 0x01, 0x38, 0x95, \
    0x7a, 0x46, (uint8_t) (val & 0xff), (uint8_t) (val >> 8), 0x02, 0x00, 0x5a, 0x55  \
  }

BLEService        service                    (SCIENCE_KIT_UUID(0x0000));
BLECharacteristic versionCharacteristic      (SCIENCE_KIT_UUID(0x0001));
BLECharacteristic accelerationCharacteristic (SCIENCE_KIT_UUID(0x0011));
BLECharacteristic gyroscopeCharacteristic    (SCIENCE_KIT_UUID(0x0012));
BLECharacteristic magneticFieldCharacteristic(SCIENCE_KIT_UUID(0x0013));
BLECharacteristic temperatureCharacteristic  (SCIENCE_KIT_UUID(0x0014));
BLECharacteristic pressureCharacteristic     (SCIENCE_KIT_UUID(0x0015));
BLECharacteristic humidityCharacteristic     (SCIENCE_KIT_UUID(0x0016));
BLECharacteristic proximityCharacteristic    (SCIENCE_KIT_UUID(0x0017));
BLECharacteristic colorCharacteristic        (SCIENCE_KIT_UUID(0x0018));
BLECharacteristic soundPressureCharacteristic(SCIENCE_KIT_UUID(0x0019));


#if defined(ARDUINO_NRF52840_CIRCUITPLAY)

#include <Adafruit_CircuitPlayground.h>

#elif defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>

Adafruit_LSM6DS33 lsm6ds33; // Gyro and Accel
Adafruit_LIS3MDL  lis3mdl;  // Magnetometer
Adafruit_APDS9960 apds9960; // Proximity, Light, Gesture, Color
Adafruit_BMP280   bmp280;   // Temperature, Barometric
Adafruit_SHT31    sht30;    // Humid

#else

#error "Board is not supported"

#endif

Adafruit_Sensor* accel_sensor;

short soundSampleBuffer[256];

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(soundSampleBuffer, bytesAvailable);
}

uint16_t getSoundAverage() {
  uint32_t avg = 0;
  for (uint32_t i = 0; i < sizeof(soundSampleBuffer)/sizeof(soundSampleBuffer[0]); i++) {
    avg += soundSampleBuffer[i]*soundSampleBuffer[i];
  }
  return sqrt(avg);
}

void setupSensors(void)
{
#if defined ARDUINO_NRF52840_CIRCUITPLAY
  CircuitPlayground.begin();
  accel_sensor = &CircuitPlayground.lis;

#elif defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)

  #ifdef ARDUINO_NRF52840_CLUE
  // White LEDs for color sensing
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, LOW);
  #endif

  apds9960.begin();
  bmp280.begin();
  sht30.begin(0x44);
  lsm6ds33.begin_I2C();
  lis3mdl.begin_I2C();

  // set lowest range
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  // set slightly above refresh rate
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Increase I2C speed to 400 Khz
  Wire.setClock(400000);

  accel_sensor = lsm6ds33.getAccelerometerSensor();
#endif

  // 1 channel (mono mode) with 16 kHz sample rate
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);
}

void setupBLEScience(void)
{
  service.begin();

  versionCharacteristic.setProperties(CHR_PROPS_READ);
  versionCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  versionCharacteristic.setFixedLen(4);
  versionCharacteristic.begin();
  versionCharacteristic.write32(VERSION);

  accelerationCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  accelerationCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelerationCharacteristic.setFixedLen(3 * sizeof(float));
  accelerationCharacteristic.begin();

#if defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  gyroscopeCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  gyroscopeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gyroscopeCharacteristic.setFixedLen(3 * sizeof(float));
  gyroscopeCharacteristic.begin();

  magneticFieldCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  magneticFieldCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  magneticFieldCharacteristic.setFixedLen(3 * sizeof(float));
  magneticFieldCharacteristic.begin();

  pressureCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  pressureCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pressureCharacteristic.setFixedLen(sizeof(float));
  pressureCharacteristic.begin();

  humidityCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  humidityCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humidityCharacteristic.setFixedLen(sizeof(float));
  humidityCharacteristic.begin();

  proximityCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  proximityCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  proximityCharacteristic.setFixedLen(sizeof(unsigned int));
  proximityCharacteristic.setCccdWriteCallback(science_notify_callback);
  proximityCharacteristic.begin();
#endif

  temperatureCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  temperatureCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  temperatureCharacteristic.setFixedLen(sizeof(float));
  temperatureCharacteristic.begin();

  colorCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  colorCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  colorCharacteristic.setFixedLen(4 * sizeof(int));
  colorCharacteristic.setCccdWriteCallback(science_notify_callback);
  colorCharacteristic.begin();

  soundPressureCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  soundPressureCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  soundPressureCharacteristic.setFixedLen(sizeof(unsigned short));
  soundPressureCharacteristic.begin();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Bluefruit Science Journal Example");
  Serial.println("---------------------------------\n");

  setupSensors();

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configUuid128Count(15);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  setupBLEScience();

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(service);

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
  if ( Bluefruit.connected() )
  {
    updateSubscribedCharacteristics();
    delay(1000);
  }
}

void updateSubscribedCharacteristics(void)
{
  sensors_event_t event;

  if ( accelerationCharacteristic.notifyEnabled() )
  {
    accel_sensor->getEvent(&event);
    accelerationCharacteristic.notify(event.data, accelerationCharacteristic.getMaxLen());
  }

#ifdef ARDUINO_NRF52840_CIRCUITPLAY
  if ( temperatureCharacteristic.notifyEnabled() )
  {
    float temperature = CircuitPlayground.temperature();
    temperatureCharacteristic.notify32(temperature);
  }

  if ( colorCharacteristic.notifyEnabled() )
  {
    int color[4] = { 0 };
    color[3] = CircuitPlayground.lightSensor();
    colorCharacteristic.notify(color, colorCharacteristic.getMaxLen());
  }

#elif defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  if ( gyroscopeCharacteristic.notifyEnabled() )
  {
    lsm6ds33.getGyroSensor()->getEvent(&event);

    // Convert gyro from Rad/s to Degree/s
    event.gyro.x *= SENSORS_RADS_TO_DPS;
    event.gyro.y *= SENSORS_RADS_TO_DPS;
    event.gyro.z *= SENSORS_RADS_TO_DPS;

    gyroscopeCharacteristic.notify(event.data, gyroscopeCharacteristic.getMaxLen());
  }

  if ( magneticFieldCharacteristic.notifyEnabled() )
  {
    lis3mdl.getEvent(&event);
    magneticFieldCharacteristic.notify(event.data, magneticFieldCharacteristic.getMaxLen());
  }

  if ( proximityCharacteristic.notifyEnabled() )
  {
    uint32_t proximity = 255 - apds9960.readProximity();
    proximityCharacteristic.notify32(proximity);
  }

  if ( temperatureCharacteristic.notifyEnabled() ||  humidityCharacteristic.notifyEnabled() )
  {
    float temperature = bmp280.readTemperature();
    float temperatureCalibrated = temperature + TEMPERATURE_CALIBRATION;

    if ( temperatureCharacteristic.notifyEnabled() )
    {
      temperatureCharacteristic.notify32(temperatureCalibrated);
    }

    if ( humidityCharacteristic.notifyEnabled() )
    {
      float humidity = sht30.readHumidity();
      float dp = temperature - ((100.0 - humidity) / 5.0);
      float humidityCalibrated = 100.0 - (5.0 * (temperatureCalibrated - dp));
      humidityCharacteristic.notify32(humidityCalibrated);
    }
  }

  if ( pressureCharacteristic.notifyEnabled() )
  {
    float pressure = bmp280.readPressure() / 1000.0; // kilo pascal
    pressureCharacteristic.notify32(pressure);
  }

  if ( colorCharacteristic.notifyEnabled() )
  {
    int color[4] = { 0 };
    apds9960.getColorData((uint16_t*) &color[0], (uint16_t*) &color[1], (uint16_t*) &color[2], (uint16_t*) &color[3]);
    colorCharacteristic.notify(color, colorCharacteristic.getMaxLen());
  }
#endif

  if ( soundPressureCharacteristic.notifyEnabled() )
  {
    uint16_t sound = getSoundAverage();
    soundPressureCharacteristic.notify16(sound);
  }
}

void science_notify_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t value)
{
  (void) conn_hdl;

  bool enabled = (value == 0x0001);
  (void) enabled;

#if defined(ARDUINO_NRF52840_CLUE) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  if ( chr == &colorCharacteristic )
  {
    apds9960.enableColor(enabled);

#ifdef ARDUINO_NRF52840_CLUE
    digitalWrite(PIN_LED2, enabled);
#endif
  }

  if ( chr == &proximityCharacteristic)
  {
    apds9960.enableProximity(enabled);
  }
#endif
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
