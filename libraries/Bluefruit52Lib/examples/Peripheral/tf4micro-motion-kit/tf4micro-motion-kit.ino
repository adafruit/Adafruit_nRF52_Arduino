/* Copyright 2021 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

/*
* @author Rikard Lindstrom <rlindsrom@google.com>
*/

// Ported to Adafruit_nRF52_Arduino core by hathach for Adafruit from
// https://github.com/googlecreativelab/tiny-motion-trainer
//
// Boards equipped with IMU sensor such as Clue and Feather nRF52840 Sense are supported
// out of the box. If you use other board, or sensor please edit the 'data_provider.cpp' file
// to include driver libraries for yours.

// For how to run this example check out following
// - https://github.com/googlecreativelab/tf4micro-motion-kit
// - https://experiments.withgoogle.com/collection/tfliteformicrocontrollers

#define VERSION 5
#define FLOAT_BYTE_SIZE 4

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>

#include "ble_file_transfer.h"
#include "model_tester.h"
#include "data_provider.h"

/************************************************************************
* State
************************************************************************/

enum State
{
  IDLE_DISCONNECTED,          // Arduino was just turned on
  IDLE_CONNECTED,             // BLE was connected
  FILE_TRANSFER,              // File transfer mode
  INFERENCE,                  // Inference is happening and published
  IMU_DATA_PROVIDER,          // Send IMU data over BLE for IMU Trainer
  ERROR_STATE,                // Something went wrong,
  CALIBRATION,                // Calibrate Magnetometer position
  INFERENCE_AND_DATA_PROVIDER // both inference and IMU Data
};

State currentState = IDLE_DISCONNECTED;
State prevState = IDLE_DISCONNECTED;

enum FileTransferType
{
  MODEL_FILE,
  TEST_FILE
};

FileTransferType fileTransferType = MODEL_FILE;

/************************************************************************
* Globals / General
************************************************************************/
bool useMagnetometer = false; // Can be toggled with BLE (disableMagnetometerRx)

Adafruit_NeoPixel neopixels = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

/************************************************************************
* BLE Characteristic / Service UUIDs
************************************************************************/

#define LOCAL_NAME "TF4Micro - Motion Kit"

#define UUID_GEN(val) ("81c30e5c-" val "-4f7d-a886-de3e90749161")

BLEService          service                   (UUID_GEN("0000"));

BLECharacteristic   dataProviderTxChar        (UUID_GEN("1001") , BLERead | BLENotify , 9 * FLOAT_BYTE_SIZE);
BLECharacteristic   dataProviderLabelsTxChar  (UUID_GEN("1002") , BLERead             , 128);
BLECharacteristic   versionTxChar             (UUID_GEN("1003") , BLERead             , 1 , true);
BLECharacteristic   inferenceTxChar           (UUID_GEN("1004") , BLERead | BLENotify , 3 , true);

BLECharacteristic   numClassesRxChar          (UUID_GEN("2001") , BLEWrite            , 1 , true);
BLECharacteristic   numSamplesRxChar          (UUID_GEN("2002") , BLEWrite            , 4 , true);
BLECharacteristic   captureDelayRxChar        (UUID_GEN("2003") , BLEWrite            , 4 , true);
BLECharacteristic   thresholdRxChar           (UUID_GEN("2004") , BLEWrite            , 4 , true);
BLECharacteristic   disableMagnetometerRx     (UUID_GEN("2005") , BLEWrite            , 1 , true);

BLECharacteristic   stateRxChar               (UUID_GEN("3001") , BLEWrite            , 1 , true);
BLECharacteristic   stateTxChar               (UUID_GEN("3002") , BLERead | BLENotify , 1 , true);
BLECharacteristic   fileTransferTypeRxChar    (UUID_GEN("3003") , BLEWrite            , 1 , true);
BLECharacteristic   hasModelTxChar            (UUID_GEN("3004") , BLERead | BLENotify , 1 , true);

// Meta is for future-proofing, we can use it to store and read any 64 bytes
BLECharacteristic  metaRxChar                (UUID_GEN("4001")  , BLEWrite            , 64);
BLECharacteristic  metaTxChar                (UUID_GEN("4002")  , BLERead             , 64);

/************************************************************************
* Model file transfer
************************************************************************/

uint8_t *newModelFileData = nullptr;
int newModelFileLength = 0;

/************************************************************************
* Labels for local display
************************************************************************/

// Currently there is no set Labels, this used actual data matched with 2 examples
// "Air Snare" and "Finger User Interface" to display TFT locally

// from ARCADA color
#define COLOR_BLACK 0x0000       ///<   0,   0,   0
#define COLOR_BLUE 0x001F        ///<   0,   0, 255
#define COLOR_GREEN 0x07E0       ///<   0, 255,   0
#define COLOR_CYAN 0x07FF        ///<   0, 255, 255
#define COLOR_RED 0xF800         ///< 255,   0,   0
#define COLOR_MAGENTA 0xF81F     ///< 255,   0, 255
#define COLOR_YELLOW 0xFFE0      ///< 255, 255,   0
#define COLOR_WHITE 0xFFFF       ///< 255, 255, 255
#define COLOR_ORANGE 0xFD20      ///< 255, 165,   0
#define COLOR_PINK 0xFC18        ///< 255, 130, 198

const char* labelAirSnare[] = {
  "Side", "Down", "Up"
};

const char* labelFingerUserInterface[] = {
  "Left", "Right", "Poke", "Twirl", "Pluck"
};

const char** modelLabel = NULL;

// detect model label
const char** detectModelLabel(int file_length) {
  uint8_t const numClasses = numClassesRxChar.read8();

  if ( (numClasses == 3) && ( (file_length / 1000) == 17 ) ) {
    return labelAirSnare;
  }

  if ( (numClasses == 5) && ( (file_length / 1000) == 35 ) ) {
    return labelFingerUserInterface;
  }

  return NULL;
}

uint16_t const color_pallete[] = {
  COLOR_BLUE,
  COLOR_GREEN, // Green
  COLOR_YELLOW, // Yellow
  COLOR_CYAN, // Cyan
  COLOR_PINK, // Pink
  COLOR_MAGENTA, // MAGENTA
  COLOR_ORANGE, // Orange
  COLOR_RED, // Red
};

#if defined(ARDUINO_NRF52840_CLUE)

// use arcada for display
#include <Adafruit_Arcada.h>

Adafruit_Arcada arcada;

void displayInit(void) {
  arcada.displayBegin();
  arcada.setBacklight(255);

  Adafruit_SPITFT* tft = arcada.display;

  tft->fillScreen(COLOR_BLACK);
  tft->setRotation(3);
}

void displayText(const char* text, uint8_t size, uint16_t color) {
  Adafruit_SPITFT* tft = arcada.display;

  tft->fillScreen(COLOR_BLACK);
  tft->setTextColor(color);
  tft->setTextSize(size);
  tft->setCursor(60, 100);
  tft->print(text);
}

#else

// stub for no-tft board
void displayText(const char* text, uint8_t size, uint16_t color) { (void) text; (void) size; (void) color; }
void displayInit(void) { }

#endif

/************************************************************************
* LED / State status functions
************************************************************************/

void rgbLedOff()
{
  neopixels.setPixelColor(0, 0, 0, 0);
  neopixels.show();
}

void rgbLedYellow()
{
  neopixels.setPixelColor(0, 0xff, 0xff, 0);
  neopixels.show();
}

void rgbLedRed()
{
  neopixels.setPixelColor(0, 0xff, 0, 0);
  neopixels.show();
}

void rgbLedGreen()
{
  neopixels.setPixelColor(0, 0, 0xff, 0);
  neopixels.show();
}

void rgbLedBlue()
{
  neopixels.setPixelColor(0, 0, 0, 0xff);
  neopixels.show();
}

void showErrorLed()
{
  // blink red
  millis() % 1000 > 500 ? rgbLedOff() : rgbLedRed();
  yield();
}

void updateLed()
{
  switch (currentState)
  {

  case FILE_TRANSFER:
    if (ble_file_transfer::isTransfering())
    {
      // Rapid blink while transfering is in progress
      millis() % 100 > 50 ? rgbLedOff() : rgbLedYellow();
    }
    else
    {
      rgbLedYellow();
    }
    break;

  case INFERENCE:
    rgbLedGreen();
    break;

  case IMU_DATA_PROVIDER:
    rgbLedBlue();
    break;

  case INFERENCE_AND_DATA_PROVIDER:
    millis() % 800 > 400 ? rgbLedBlue() : rgbLedGreen();
    break;

  case ERROR_STATE:
    showErrorLed();
    break;

  case CALIBRATION:
    millis() % 100 > 50 ? rgbLedOff() : rgbLedGreen();
    break;

  case IDLE_DISCONNECTED:
  case IDLE_CONNECTED:
  default:
    if (Bluefruit.connected())
    {
      rgbLedBlue();
    }
    else
    {
      // Blink yellow at a .5 second interval
      const int now = millis();
      if (now % 1000 > 800)
      {
        rgbLedOff();
      }
      else
      {
        int blinkStep = floor(now % 3000);
        switch (blinkStep)
        {
        case 0:
          rgbLedRed();
          break;
        case 1000:
          rgbLedGreen();
          break;
        case 2000:
          rgbLedBlue();
          break;
        }
      }
    }
    break;
  }
}

void setState(State state)
{
  if (state != prevState && state != currentState)
  {
    prevState = currentState;
  }
  currentState = state;
  stateTxChar.notify8((unsigned char)state);
  switch (currentState)
  {
  case IDLE_DISCONNECTED:
    Serial.println("state is now IDLE_DISCONNECTED");
    break;
  case IDLE_CONNECTED:
    Serial.println("state is now IDLE_CONNECTED");
    break;
  case FILE_TRANSFER:
    Serial.println("state is now FILE_TRANSFER");
    displayText("Dwnloading", 3, COLOR_ORANGE);
    break;
  case INFERENCE:
    Serial.println("state is now INFERENCE");
    displayText("Ready", 3, COLOR_GREEN);
    break;
  case IMU_DATA_PROVIDER:
    Serial.println("state is now IMU_DATA_PROVIDER");
    break;
  case ERROR_STATE:
    Serial.println("state is now ERROR_STATE");
    break;
  case CALIBRATION:
    data_provider::calibrate();
    Serial.println("state is now CALIBRATION");
    break;
  case INFERENCE_AND_DATA_PROVIDER:
    Serial.println("state is now INFERENCE_AND_DATA_PROVIDER");
    break;
  default:
    Serial.println("Error: Unknown state");
  }
}

/************************************************************************
* BLE Event handlers
************************************************************************/

void handleNumSamplesRxWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const int value = (int) numSamplesRxChar.read32();
  model_tester::setNumSamples(value);
  Serial.print("Received numSamples: ");
  Serial.println(value);
}

void handleThresholdRxWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const float value = thresholdRxChar.readFloat();
  model_tester::setThreshold(value);
  Serial.print("Received threshold: ");
  Serial.println(value, 4);
}

void handleCaptureDelayRxWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const int value = captureDelayRxChar.read32();
  model_tester::setCaptureDelay(value);
  Serial.print("Received delay: ");
  Serial.println(value);
}

void handleNumClassesRxWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const unsigned char value = (unsigned char) numClassesRxChar.read8();
  model_tester::setNumClasses(value);
  Serial.print("Received numClasses: ");
  Serial.println(value);
}

void handleDisableMagnetometerRxWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const bool val = (bool) disableMagnetometerRx.read8();
  model_tester::setDisableMagnetometer(val);

  useMagnetometer = !val;
  
  Serial.print("Received disableMagnetometer: ");
  Serial.println(val);
}

void handleStateWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  const uint8_t value = stateRxChar.read8();

  setState((State) value);
  Serial.print("Received state: ");
  Serial.println(value);
}

void handleMetaWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  // Meta is just a 64 byte storage for anything, just publish it
  metaTxChar.write(data, len);
}

void handleFileTransferTypeWritten(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl; (void) chr; (void) data; (void) len;

  fileTransferType = (FileTransferType) fileTransferTypeRxChar.read8();
  Serial.print("Received fileTransferType: ");
  Serial.println(fileTransferType);
}

/************************************************************************
* Callbacks
************************************************************************/

// called on inference (gesture detected)
void model_tester_onInference(unsigned char classIndex, unsigned char score, unsigned char velocity)
{
  const byte buffer[]{classIndex, score, velocity};
  inferenceTxChar.notify(buffer, 3);
  Serial.print("Inference - class: ");
  Serial.print(classIndex);
  Serial.print(" score: ");
  Serial.println(score);

  if ( modelLabel ) {
    // display label on TFT use index as color id
    displayText(modelLabel[classIndex], 5, color_pallete[classIndex]);
  }
}

// called when calibration completes
void data_provider_calibrationComplete(){
  setState(prevState);
}

// called on file transfer complete
void onBLEFileReceived(uint8_t *file_data, int file_length)
{
  Serial.print("file length ");
  Serial.println(file_length);

  // detect label of model for local display
  modelLabel = detectModelLabel(file_length);

  switch (fileTransferType)
  {
    case MODEL_FILE:
      // Queue up the model swap
      newModelFileData = file_data;
      newModelFileLength = file_length;
      break;
    case TEST_FILE:
    {
      int floatLength = file_length / 4;
      float buffer[floatLength];
      for (int i = 0; i < file_length; i += 4)
      {
        union u_tag
        {
          byte b[4];
          float fval;
        } u;
  
        u.b[0] = file_data[i + 0];
        u.b[1] = file_data[i + 1];
        u.b[2] = file_data[i + 2];
        u.b[3] = file_data[i + 3];
  
        buffer[i / 4] = u.fval;
      }
      // set state to inference so we can capture result
      setState(INFERENCE);
      model_tester::runTest(buffer, floatLength);
    }
    break;
    default:
      Serial.println("Error: unkown file type");
      setState(ERROR_STATE);
      while (0) updateLed();
  }
}

/************************************************************************
* Main / Lifecycle
************************************************************************/

void setup()
{
  
  Serial.begin(9600);
  const int startTime = millis();
  
  // Give serial port 2 second to connect.
  while (!Serial && millis() - startTime < 2000)
    yield();

  // Prepare LED pins.
  pinMode(LED_BUILTIN, OUTPUT);
  neopixels.begin();
  neopixels.setBrightness(0x20);
  displayInit();

  // Start IMU / Data provider.
  if (!data_provider::setup())
  {
    Serial.println("Failed to initialize IMU!");
    while (1) showErrorLed();
  }

  Bluefruit.autoConnLed(true);
  Bluefruit.configUuid128Count(25);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);

  service.begin();

  versionTxChar.begin();
  dataProviderTxChar.begin();
  dataProviderLabelsTxChar.begin();
  inferenceTxChar.begin();

  numClassesRxChar.begin();
  numSamplesRxChar.begin();
  captureDelayRxChar.begin();
  thresholdRxChar.begin();
  disableMagnetometerRx.begin();

  stateRxChar.begin();
  stateTxChar.begin();
  fileTransferTypeRxChar.begin();
  hasModelTxChar.begin();
  
  metaRxChar.begin();
  metaTxChar.begin();

  // Event driven reads.
  numClassesRxChar.setWriteCallback(handleNumClassesRxWritten);
  numSamplesRxChar.setWriteCallback(handleNumSamplesRxWritten);
  thresholdRxChar.setWriteCallback(handleThresholdRxWritten);
  captureDelayRxChar.setWriteCallback(handleCaptureDelayRxWritten);
  stateRxChar.setWriteCallback(handleStateWritten);
  fileTransferTypeRxChar.setWriteCallback(handleFileTransferTypeWritten);
  metaRxChar.setWriteCallback(handleMetaWritten);
  disableMagnetometerRx.setWriteCallback(handleDisableMagnetometerRxWritten);


  uint8_t mac[6];
  char mac_str[20];
  Bluefruit.getAddr(mac);
  sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

  String address(mac_str);

  // Output BLE settings over Serial.
  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  static String deviceName = LOCAL_NAME;
  deviceName += " - ";
  deviceName += address[address.length() - 5];
  deviceName += address[address.length() - 4];
  deviceName += address[address.length() - 2];
  deviceName += address[address.length() - 1];

  Serial.print("deviceName = ");
  Serial.println(deviceName);

  Serial.print("localName = ");
  Serial.println(deviceName);

  // Set up properties for the whole service.
  Bluefruit.setName(deviceName.c_str());

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(service);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

  ble_file_transfer::setupBLEFileTransfer(service);

  // Print out full UUID and MAC address.
  Serial.println("Peripheral advertising info: ");
  Serial.print("Name: ");
  Serial.println(LOCAL_NAME);
  Serial.print("MAC: ");
  Serial.println(mac_str);
  Serial.print("Service UUID: ");
  Serial.println(service.uuid.toString());

  Serial.println("Bluetooth device active, waiting for connections...");

  // Broadcast sketch version
  versionTxChar.write8(VERSION);

  // Used for Tiny Motion Trainer to label / filter values
  dataProviderLabelsTxChar.write("acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, max.zl");
}

inline void updateIMU()
{
  const char bufferSize = useMagnetometer ? 9 : 6;
  float buffer[bufferSize];
  while(data_provider::dataAvailable()){

    // Collect the IMU data
    data_provider::update(buffer, useMagnetometer);
 
    if(currentState == INFERENCE || currentState == INFERENCE_AND_DATA_PROVIDER){
      // if we have a model, do inference
      if(model_tester::isModelLoaded){
        model_tester::update(buffer);  
      }
    }

    if(currentState == IMU_DATA_PROVIDER || currentState == INFERENCE_AND_DATA_PROVIDER){
      // provide data to IMU trainer
      dataProviderTxChar.notify(buffer, bufferSize * FLOAT_BYTE_SIZE);
    }

  }
}

inline void updateFileTransfer()
{
  // Update file transfer state
  ble_file_transfer::updateBLEFileTransfer();

  // Check if we should load a new model
  if (newModelFileData != nullptr)
  {
    Serial.println("reloading model");
    model_tester::loadModel(newModelFileData);
    Serial.println("done reloading model");
    newModelFileData = nullptr;

    hasModelTxChar.notify8(true);
    
    // We have a new model, always enter INFERENCE mode
    setState(INFERENCE);
  }
}

void loop()
{
  // Make sure we're connected and not busy file-transfering
  if (Bluefruit.connected())
  {
    switch (currentState)
    {
    case FILE_TRANSFER:
      updateFileTransfer();
      break;

    case CALIBRATION:
    case IMU_DATA_PROVIDER:
    case INFERENCE_AND_DATA_PROVIDER:
    case INFERENCE:
      if(!ble_file_transfer::isTransfering()){
        updateIMU();
      }
      break;

    default:
      break;
    }
  } else if(currentState != IDLE_DISCONNECTED){
    setState(IDLE_DISCONNECTED);
  }

  // Update led based on state
  updateLed();
}
