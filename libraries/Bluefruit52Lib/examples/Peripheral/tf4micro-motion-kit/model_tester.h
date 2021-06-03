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

#ifndef MODEL_TESTER_CPP
#define MODEL_TESTER_CPP

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

// Forward declare the function that will be called when data has been delivered to us.
void model_tester_onInference(unsigned char classIndex, unsigned char score, unsigned char velocity);

namespace model_tester {
  void loadModel(unsigned char model[]);
  void update(float buffer[]);
  extern bool isModelLoaded;

  // Setters
  void setCaptureDelay(int val);
  void setNumSamples(int val);
  void setThreshold(float val);
  void setNumClasses(unsigned char val);
  void setDisableMagnetometer(bool val);
  void runTest(float *testData, int len);
}

#endif
