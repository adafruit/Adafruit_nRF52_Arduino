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
 *  Namespaced methods for loading a TF model and running inference
 *  @author Rikard Lindstrom <rlindsrom@google.com>
*/
#include "model_tester.h"
#include <Arduino.h>
//#include <ArduinoBLE.h>

namespace model_tester
{

/************************************************************************
* Capture settings variables
************************************************************************/

  int numSamples = 10;
  int samplesRead = numSamples;
  float accelerationThreshold = 0.167;
  int captureDelay = 125;
  unsigned char numClasses = 3;
  bool disableMagnetometer = false;
  
  float maxVelocity = 0.;
  int lastCaptureTimestamp = 0;
  
/************************************************************************
* Model loading / inference variables
************************************************************************/

  bool isModelLoaded = false;
  uint8_t interpreterBuffer[sizeof(tflite::MicroInterpreter)];

  constexpr int tensorArenaSize = 8 * 1024;
  byte tensorArena[tensorArenaSize];

  // global variables used for TensorFlow Lite (Micro)
  tflite::MicroErrorReporter tflErrorReporter;

  // pull in all the TFLM ops, you can remove this line and
  // only pull in the TFLM ops you need, if would like to reduce
  // the compiled size of the sketch.
  tflite::AllOpsResolver tflOpsResolver;

  const tflite::Model *tflModel = nullptr;
  tflite::MicroInterpreter *tflInterpreter = nullptr;
  TfLiteTensor *tflInputTensor = nullptr;
  TfLiteTensor *tflOutputTensor = nullptr;


/************************************************************************
* Setters
************************************************************************/

  void setCaptureDelay(int val){
    captureDelay = val;
  }

  void setNumSamples(int val){
    numSamples = val;
    samplesRead = val;
  }
  
  void setThreshold(float val){
    accelerationThreshold = val;
  }

  void setNumClasses(unsigned char val){
    numClasses = val;
  }

   void setDisableMagnetometer(bool val){
    disableMagnetometer = val;
  }

/************************************************************************
* Model loading
************************************************************************/

  void loadModel(unsigned char model[])
  {

    // get the TFL representation of the model byte array
    tflModel = tflite::GetModel(model);
    if (tflModel->version() != TFLITE_SCHEMA_VERSION)
    {
      Serial.println("Model schema mismatch!");
      while (1)
        delay(1);
    }

    // Create an interpreter to run the model
    tflInterpreter = new (interpreterBuffer) tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

    // Allocate memory for the model's input and output tensors
    tflInterpreter->AllocateTensors();

    // Get pointers for the model's input and output tensors
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);

    isModelLoaded = true;
  }
  
/************************************************************************
* Life cycle
************************************************************************/

 // #define DEBUG - uncomment to log out each capture as CSV

  void update(float buffer[])
  {

   
    int now = millis();
    
    if (samplesRead == numSamples)
    {
    
      // Honor captureDelay setting
      if(now - lastCaptureTimestamp < captureDelay){
        return;  
      }
      
      const float aSum = (fabs(buffer[0]) + fabs(buffer[1]) + fabs(buffer[2]) + fabs(buffer[3]) + fabs(buffer[4]) + fabs(buffer[5])) / 6.0;

      // check if it's above the threshold
      if (aSum >= accelerationThreshold)
      {
        #ifdef DEBUG
        Serial.println("Capture started:");
        Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
        #endif
        
        // reset the sample read count
        samplesRead = 0;
        maxVelocity = 0.;
      }
    }

    if (samplesRead < numSamples)
    {  
      const int dataLen = disableMagnetometer ? 6 : 9;

      #ifdef DEBUG
      for(int j = 0; j < dataLen; j++){
        Serial.print(buffer[j], 16);
        if(j < dataLen - 1){
          Serial.print(",");
        }
      }
      Serial.println();
      #endif
      
      const float velocity = (fabs(buffer[0]) + fabs(buffer[1]) + fabs(buffer[2])) / 3.0; 
      maxVelocity = max(maxVelocity, velocity);

      tflInputTensor->data.f[samplesRead * dataLen + 0] = buffer[0];
      tflInputTensor->data.f[samplesRead * dataLen + 1] = buffer[1];
      tflInputTensor->data.f[samplesRead * dataLen + 2] = buffer[2];
      tflInputTensor->data.f[samplesRead * dataLen + 3] = buffer[3];
      tflInputTensor->data.f[samplesRead * dataLen + 4] = buffer[4];
      tflInputTensor->data.f[samplesRead * dataLen + 5] = buffer[5];
      if(!disableMagnetometer){
        tflInputTensor->data.f[samplesRead * dataLen + 6] = buffer[6];
        tflInputTensor->data.f[samplesRead * dataLen + 7] = buffer[7];
        tflInputTensor->data.f[samplesRead * dataLen + 8] = buffer[8];
      }

      samplesRead++;

      if (samplesRead == numSamples)
      {
        // Run inferencing
        TfLiteStatus invokeStatus = tflInterpreter->Invoke();
        if (invokeStatus != kTfLiteOk)
        {
          Serial.println("Invoke failed!");
          while (1)
            delay(1);
          return;
        }
        #ifdef DEBUG
        Serial.println();
        Serial.println("-----------------------------------------");
        #endif

        // Loop through the output tensor values from the model
        unsigned char maxIndex = 0;
        float maxValue = 0;
        for (int i = 0; i < numClasses; i++)
        {
          float _value = tflOutputTensor->data.f[i];
          Serial.print("class: ");
          Serial.println(i);
          Serial.print(" score: ");
          Serial.println(_value, 6);
          
          if (_value > maxValue)
          {
            maxValue = _value;
            maxIndex = i;
          }
        }

        // callback
        unsigned char velocity = (unsigned char )(((maxVelocity - accelerationThreshold) / (1.0 - accelerationThreshold)) * 255.999);
        unsigned char score = (unsigned char)(maxValue * 255.999);
        
        model_tester_onInference(maxIndex, score, velocity);

        Serial.print("Winner: ");
        Serial.print(maxIndex);
        
        // timestamp to follow the capture delay setting
        lastCaptureTimestamp = now;
      }
    }
  }

  void runTest(float *data, int len){
    const int numTests = len / numSamples;
    const int dataLen = disableMagnetometer ? 6 : 9;
    samplesRead = numSamples;
    const int tmpCaptureDelay = captureDelay;
    captureDelay = 0;
    for(int i = 0; i < len; i+= dataLen){
      if(disableMagnetometer){
        float buffer[] = {
          data[i + 0],
          data[i + 1],
          data[i + 2],
          data[i + 3],
          data[i + 4],
          data[i + 5]
        };
        update(buffer);
      } else {
        float buffer[] = {
          data[i + 0],
          data[i + 1],
          data[i + 2],
          data[i + 3],
          data[i + 4],
          data[i + 5],
          data[i + 6],
          data[i + 7],
          data[i + 8]
        };
              
        //update(buffer);
      }
    }

    captureDelay = tmpCaptureDelay;
  }
}
