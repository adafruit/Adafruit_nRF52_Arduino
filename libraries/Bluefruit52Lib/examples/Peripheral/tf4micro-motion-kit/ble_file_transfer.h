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

#ifndef BLE_FILE_TRANSFER_CPP
#define BLE_FILE_TRANSFER_CPP

//#include <ArduinoBLE.h>
#include <bluefruit.h>

// Forward declare the function that will be called when data has been delivered to us.
void onBLEFileReceived(uint8_t* file_data, int file_length);

namespace ble_file_transfer{
  void updateBLEFileTransfer();
  bool isTransfering();
  void setupBLEFileTransfer(BLEService service);
}

#endif // BLE_FILE_TRANSFER_CPP
