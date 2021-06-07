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

#define VERSION 5
#define FLOAT_BYTE_SIZE 4

//#include <ArduinoBLE.h>

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>

//#include "ble_file_transfer.h"
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

BLEService                      service                   (UUID_GEN("0000"));

//BLECharacteristic               dataProviderTxChar        (UUID_GEN("1001"), BLERead | BLENotify, 9 * FLOAT_BYTE_SIZE);
//BLECharacteristic               dataProviderLabelsTxChar  (UUID_GEN("1002"), BLERead, 128);
//BLEUnsignedCharCharacteristic   versionTxChar             (UUID_GEN("1003"), BLERead);
//BLECharacteristic               inferenceTxChar           (UUID_GEN("1004"), BLERead | BLENotify, 3);
//
//BLEUnsignedCharCharacteristic   numClassesRxChar          (UUID_GEN("2001"), BLEWrite);
//BLEIntCharacteristic            numSamplesRxChar          (UUID_GEN("2002"), BLEWrite);
//BLEIntCharacteristic            captureDelayRxChar        (UUID_GEN("2003"), BLEWrite);
//BLEFloatCharacteristic          thresholdRxChar           (UUID_GEN("2004"), BLEWrite);
//BLEBoolCharacteristic           disableMagnetometerRx     (UUID_GEN("2005"), BLEWrite);
//
//BLEUnsignedCharCharacteristic   stateRxChar               (UUID_GEN("3001"), BLEWrite);
//BLEUnsignedCharCharacteristic   stateTxChar               (UUID_GEN("3002"), BLERead | BLENotify);
//BLEUnsignedCharCharacteristic   fileTransferTypeRxChar    (UUID_GEN("3003"), BLEWrite);
//BLEBoolCharacteristic           hasModelTxChar            (UUID_GEN("3004"), BLERead | BLENotify);
//
//// Meta is for future-proofing, we can use it to store and read any 64 bytes
//BLECharacteristic               metaRxChar                (UUID_GEN("4001"), BLEWrite, 64);
//BLECharacteristic               metaTxChar                (UUID_GEN("4002"), BLERead, 64);

/************************************************************************
* Model file transfer
************************************************************************/

uint8_t *newModelFileData = nullptr;
int newModelFileLength = 0;

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
//  digitalWrite(LEDR, LOW);
//  digitalWrite(LEDG, LOW);
//  digitalWrite(LEDB, HIGH);
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
  delay(500);
}

void updateLed()
{
  switch (currentState)
  {

  case FILE_TRANSFER:
#if 0
    if (ble_file_transfer::isTransfering())
#else
    if (0)
#endif
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
#if 0
    if (BLE.connected())
#else
     if (1)
#endif
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
#if 0
  stateTxChar.writeValue((unsigned char)state);
#endif
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

#if 0
void handleNumSamplesRxWritten(BLEDevice central, BLECharacteristic characteristic)
{
  model_tester::setNumSamples(numSamplesRxChar.value());
  Serial.print("Received numSamples: ");
  Serial.println(numSamplesRxChar.value());
}

void handleThresholdRxWritten(BLEDevice central, BLECharacteristic characteristic)
{
  model_tester::setThreshold(thresholdRxChar.value());
  Serial.print("Received threshold: ");
  Serial.println(thresholdRxChar.value(), 4);
}

void handleCaptureDelayRxWritten(BLEDevice central, BLECharacteristic characteristic)
{
  model_tester::setCaptureDelay(captureDelayRxChar.value());
  Serial.print("Received delay: ");
  Serial.println(captureDelayRxChar.value());
}

void handleNumClassesRxWritten(BLEDevice central, BLECharacteristic characteristic)
{
  model_tester::setNumClasses(numClassesRxChar.value());
  Serial.print("Received numClasses: ");
  Serial.println(numClassesRxChar.value());
}

void handleDisableMagnetometerRxWritten(BLEDevice central, BLECharacteristic characteristic)
{
  bool val = disableMagnetometerRx.value();
  model_tester::setDisableMagnetometer(val);

  useMagnetometer = !val;
  
  Serial.print("Received disableMagnetometer: ");
  Serial.println(disableMagnetometerRx.value());
}

void handleStateWritten(BLEDevice central, BLECharacteristic characteristic)
{
  setState((State)stateRxChar.value());
  Serial.print("Received state: ");
  Serial.println(stateRxChar.value());
}

void handleMetaWritten(BLEDevice central, BLECharacteristic characteristic)
{
  // Meta is just a 64 byte storage for anything, just publish it
  byte values[64];
  metaRxChar.readValue(values, 64);
  metaTxChar.writeValue(values, 64);
}

void handleFileTransferTypeWritten(BLEDevice central, BLECharacteristic characteristic)
{
  fileTransferType = (FileTransferType)fileTransferTypeRxChar.value();
  Serial.print("Received fileTransferType: ");
  Serial.println(fileTransferType);
}
#endif

/************************************************************************
* Callbacks
************************************************************************/

// called on inference (gesture detected)
void model_tester_onInference(unsigned char classIndex, unsigned char score, unsigned char velocity)
{
  const byte buffer[]{classIndex, score, velocity};
#if 0
  inferenceTxChar.setValue(buffer, 3);
#endif
  Serial.print("Inference - class: ");
  Serial.print(classIndex);
  Serial.print(" score: ");
  Serial.println(score);
}

// called when calibration completes
void data_provider_calibrationComplete(){
  setState(prevState);
}

// called on file transfer complete
void onBLEFileReceived(uint8_t *file_data, int file_length)
{
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

  Serial.println("Bluefruit52 Example");
  Serial.println("-------------------\n");

  // Prepare LED pins.
  pinMode(LED_BUILTIN, OUTPUT);
  neopixels.begin();

  // Start IMU / Data provider.
//  if (!data_provider::setup())
//  {
//    Serial.println("Failed to initialize IMU!");
//    while (1) showErrorLed();
//  }

  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);

  service.begin();

#if 0
  service.addCharacteristic(versionTxChar);
  service.addCharacteristic(dataProviderTxChar);
  service.addCharacteristic(dataProviderLabelsTxChar);
  service.addCharacteristic(inferenceTxChar);

  service.addCharacteristic(numClassesRxChar);
  service.addCharacteristic(numSamplesRxChar);
  service.addCharacteristic(captureDelayRxChar);
  service.addCharacteristic(thresholdRxChar);
  service.addCharacteristic(disableMagnetometerRx);

  service.addCharacteristic(stateRxChar);
  service.addCharacteristic(stateTxChar);
  service.addCharacteristic(fileTransferTypeRxChar);
  service.addCharacteristic(hasModelTxChar);
  
  service.addCharacteristic(metaRxChar);
  service.addCharacteristic(metaTxChar);

  // Event driven reads.
  numClassesRxChar.setEventHandler(BLEWritten, handleNumClassesRxWritten);
  numSamplesRxChar.setEventHandler(BLEWritten, handleNumSamplesRxWritten);
  thresholdRxChar.setEventHandler(BLEWritten, handleThresholdRxWritten);
  captureDelayRxChar.setEventHandler(BLEWritten, handleCaptureDelayRxWritten);
  stateRxChar.setEventHandler(BLEWritten, handleStateWritten);
  fileTransferTypeRxChar.setEventHandler(BLEWritten, handleFileTransferTypeWritten);
  metaRxChar.setEventHandler(BLEWritten, handleMetaWritten);
  disableMagnetometerRx.setEventHandler(BLEWritten, handleDisableMagnetometerRxWritten);

  // Start the core BLE engine.
  if (!BLE.begin())
  {
    Serial.println("Failed to initialized BLE!");
    setState(ERROR_STATE);
    while (1) showErrorLed();
  }
#endif

#if 0
  String address = BLE.address();

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
  BLE.setLocalName(deviceName.c_str());
  BLE.setDeviceName(deviceName.c_str());
  BLE.setAdvertisedService(service);
#endif

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(service);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

#if 0
  ble_file_transfer::setupBLEFileTransfer(service);

  // Print out full UUID and MAC address.
  Serial.println("Peripheral advertising info: ");
  Serial.print("Name: ");
  Serial.println(LOCAL_NAME);
  Serial.print("MAC: ");
  Serial.println(BLE.address());
  Serial.print("Service UUID: ");
  Serial.println(service.uuid());

  // Start up the service itself.
  BLE.addService(service);
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // Broadcast sketch version
  versionTxChar.writeValue(VERSION);

  // Used for Tiny Motion Trainer to label / filter values
  dataProviderLabelsTxChar.writeValue("acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, max.zl");
#endif
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
#if 0
    if(currentState == IMU_DATA_PROVIDER || currentState == INFERENCE_AND_DATA_PROVIDER){
      // provide data to IMU trainer
      dataProviderTxChar.writeValue(buffer, bufferSize * FLOAT_BYTE_SIZE);
    }
#endif
  }
}

inline void updateFileTransfer()
{
#if 0
  // Update file transfer state
  ble_file_transfer::updateBLEFileTransfer();

  // Check if we should load a new model
  if (newModelFileData != nullptr)
  {
    Serial.println("reloading model");
    model_tester::loadModel(newModelFileData);
    Serial.println("done reloading model");
    newModelFileData = nullptr;

    hasModelTxChar.writeValue(true);
    
    // We have a new model, always enter INFERENCE mode
    setState(INFERENCE);
  }
#endif
}

void loop()
{
#if 0
  // Make sure we're connected and not busy file-transfering
  if (BLE.connected())
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
#endif

  // Update led based on state
  updateLed();
}
