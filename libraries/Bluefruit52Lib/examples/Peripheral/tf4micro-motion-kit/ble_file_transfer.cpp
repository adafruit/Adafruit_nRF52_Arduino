/* Copyright 2021 The TensorFlow Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

/*
 * Original here: https://github.com/petewarden/ble_file_transfer
 * Modified from original:
 *   - added name to namespace
 *   - moved to separate file with a header file
 *   - moved BLEService outside of namespace
 */
 
#include "ble_file_transfer.h"

// This is part of a simple demonstration of how to transfer small (tens of
// kilobytes) files over BLE onto an Arduino Nano BLE Sense board. Most of this
// sketch is internal implementation details of the protocol, but if you just
// want to use it you can look at the bottom of this file.
// The API is that you call setupBLEFileTransfer() in your setup() function to
// open up communication with any clients that want to send you files, and then
// onBLEFileReceived() is called when a file has been downloaded.


// Comment this macro back in to log received data to the serial UART.
//#define ENABLE_LOGGING

namespace ble_file_transfer {

// Controls how large a file the board can receive. We double-buffer the files
// as they come in, so you'll need twice this amount of RAM. The default is set
// to 50KB.
constexpr int32_t file_maximum_byte_count = (50 * 1024);

// Macro based on a master UUID that can be modified for each characteristic.
#define FILE_TRANSFER_UUID(val) ("bf88b656-" val "-4a61-86e0-769c741026c0")

// How big each transfer block can be. In theory this could be up to 512 bytes, but
// in practice I've found that going over 128 affects reliability of the connection.
constexpr int32_t file_block_byte_count = 128;

// Where each data block is written to during the transfer.
BLECharacteristic file_block_characteristic(FILE_TRANSFER_UUID("3000"), BLEWrite, file_block_byte_count);

// Write the expected total length of the file in bytes to this characteristic
// before sending the command to transfer a file.
BLECharacteristic file_length_characteristic(FILE_TRANSFER_UUID("3001"), BLERead | BLEWrite, sizeof(uint32_t));

// Read-only attribute that defines how large a file the sketch can handle.
BLECharacteristic file_maximum_length_characteristic(FILE_TRANSFER_UUID("3002"), BLERead, sizeof(uint32_t));

// Write the checksum that you expect for the file here before you trigger the transfer.
BLECharacteristic file_checksum_characteristic(FILE_TRANSFER_UUID("3003"), BLERead | BLEWrite, sizeof(uint32_t));

// Writing a command of 1 starts a file transfer (the length and checksum characteristics should already have been set).
// A command of 2 tries to cancel any pending file transfers. All other commands are undefined.
BLECharacteristic command_characteristic(FILE_TRANSFER_UUID("3004"), BLEWrite, sizeof(uint32_t));

// A status set to 0 means a file transfer succeeded, 1 means there was an error, and 2 means a file transfer is
// in progress.
BLECharacteristic transfer_status_characteristic(FILE_TRANSFER_UUID("3005"), BLERead | BLENotify, sizeof(uint32_t));

// Informative text describing the most recent error, for user interface purposes.
constexpr int32_t error_message_byte_count = 128;
BLECharacteristic error_message_characteristic(FILE_TRANSFER_UUID("3006"), BLERead | BLENotify, error_message_byte_count);

// Internal globals used for transferring the file.
uint8_t file_buffers[2][file_maximum_byte_count];
int finished_file_buffer_index = -1;
uint8_t* finished_file_buffer = nullptr;
int32_t finished_file_buffer_byte_count = 0;

uint8_t* in_progress_file_buffer = nullptr;
int32_t in_progress_bytes_received = 0;
int32_t in_progress_bytes_expected = 0;
uint32_t in_progress_checksum = 0;

void notifyError(const String& error_message) {
  Serial.println(error_message);
  constexpr int32_t error_status_code = 1;
  transfer_status_characteristic.notify32((int) error_status_code);
 
  const char* error_message_bytes = error_message.c_str();
  uint8_t error_message_buffer[error_message_byte_count];
  bool at_string_end = false;
  for (int i = 0; i < error_message_byte_count; ++i) {
    const bool at_last_byte = (i == (error_message_byte_count - 1));
    if (!at_string_end && !at_last_byte) {
      const char current_char = error_message_bytes[i];
      if (current_char == 0) {
        at_string_end = true;
      } else {
        error_message_buffer[i] = current_char;
      }
    }

    if (at_string_end || at_last_byte) {
      error_message_buffer[i] = 0;
    }
  }
  error_message_characteristic.notify(error_message_buffer, error_message_byte_count);
}

void notifySuccess() {
  constexpr int32_t success_status_code = 0;
  transfer_status_characteristic.notify32( (int) success_status_code);
}

void notifyInProgress() {
  constexpr int32_t in_progress_status_code = 2;
  transfer_status_characteristic.notify32( (int) in_progress_status_code);
}

// See http://home.thep.lu.se/~bjorn/crc/ for more information on simple CRC32 calculations.
uint32_t crc32_for_byte(uint32_t r) {
  for (int j = 0; j < 8; ++j) {
    r = (r & 1? 0: (uint32_t)0xedb88320L) ^ r >> 1;
  }
  return r ^ (uint32_t)0xff000000L;
}

uint32_t crc32(const uint8_t* data, size_t data_length) {
  constexpr int table_size = 256;
  static uint32_t table[table_size];
  static bool is_table_initialized = false;
  if (!is_table_initialized) {
    for(size_t i = 0; i < table_size; ++i) {
      table[i] = crc32_for_byte(i);
    }
    is_table_initialized = true;
  }
  uint32_t crc = 0;
  for (size_t i = 0; i < data_length; ++i) {
    const uint8_t crc_low_byte = static_cast<uint8_t>(crc);
    const uint8_t data_byte = data[i];
    const uint8_t table_index = crc_low_byte ^ data_byte;
    crc = table[table_index] ^ (crc >> 8);
  }
  return crc;
}

// This is a small test function for the CRC32 implementation, not normally called but left in
// for debugging purposes. We know the expected CRC32 of [97, 98, 99, 100, 101] is 2240272485,
// or 0x8587d865, so if anything else is output we know there's an error in the implementation.
void testCrc32() {
  constexpr int test_array_length = 5;
  const uint8_t test_array[test_array_length] = {97, 98, 99, 100, 101};
  const uint32_t test_array_crc32 = crc32(test_array, test_array_length);
  Serial.println(String("CRC32 for [97, 98, 99, 100, 101] is 0x") + String(test_array_crc32, 16) + 
    String(" (") + String(test_array_crc32) + String(")"));
}

void onFileTransferComplete() {
  uint32_t computed_checksum = crc32(in_progress_file_buffer, in_progress_bytes_expected);;
  if (in_progress_checksum != computed_checksum) {
    notifyError(String("File transfer failed: Expected checksum 0x") + String(in_progress_checksum, 16) + 
      String(" but received 0x") + String(computed_checksum, 16));
    in_progress_file_buffer = nullptr;
    return;
  }

  if (finished_file_buffer_index == 0) {
    finished_file_buffer_index = 1;
  } else {
    finished_file_buffer_index = 0;
  }
  finished_file_buffer = &file_buffers[finished_file_buffer_index][0];;
  finished_file_buffer_byte_count = in_progress_bytes_expected;

  in_progress_file_buffer = nullptr;
  in_progress_bytes_received = 0;
  in_progress_bytes_expected = 0;

  notifySuccess();

  onBLEFileReceived(finished_file_buffer, finished_file_buffer_byte_count);
}

void onFileBlockWritten(uint16_t conn_hdl, BLECharacteristic* characteristic, uint8_t* data, uint16_t len) {
  if (in_progress_file_buffer == nullptr) {
    notifyError("File block sent while no valid command is active");
    return;
  }
  
  const int32_t file_block_length = len;
  if (file_block_length > file_block_byte_count) {
    notifyError(String("Too many bytes in block: Expected ") + String(file_block_byte_count) + 
      String(" but received ") + String(file_block_length));
    in_progress_file_buffer = nullptr;
    return;
  }
  
  const int32_t bytes_received_after_block = in_progress_bytes_received + file_block_length;
  if ((bytes_received_after_block > in_progress_bytes_expected) ||
    (bytes_received_after_block > file_maximum_byte_count)) {
    notifyError(String("Too many bytes: Expected ") + String(in_progress_bytes_expected) + 
      String(" but received ") + String(bytes_received_after_block));
    in_progress_file_buffer = nullptr;
    return;
  }

  uint8_t* file_block_buffer = in_progress_file_buffer + in_progress_bytes_received;
  memcpy(file_block_buffer, data, file_block_length);

// Enable this macro to show the data in the serial log.
#ifdef ENABLE_LOGGING
  Serial.print("Data received: length = ");
  Serial.println(file_block_length);

  char string_buffer[file_block_byte_count + 1];
  for (int i = 0; i < file_block_byte_count; ++i) {
    unsigned char value = file_block_buffer[i];
    if (i < file_block_length) {
      string_buffer[i] = value;
    } else {
      string_buffer[i] = 0;
    }
  }
  string_buffer[file_block_byte_count] = 0;
  Serial.println(String(string_buffer));
#endif  // ENABLE_LOGGING

  if (bytes_received_after_block == in_progress_bytes_expected) {
    onFileTransferComplete();
  } else {
    in_progress_bytes_received = bytes_received_after_block;    
  }
}

void startFileTransfer() {
  if (in_progress_file_buffer != nullptr) {
    notifyError("File transfer command received while previous transfer is still in progress");
    return;
  }

  int32_t file_length_value = (int32_t) file_length_characteristic.read32();
  if (file_length_value > file_maximum_byte_count) {
    notifyError(
       String("File too large: Maximum is ") + String(file_maximum_byte_count) + 
       String(" bytes but request is ") + String(file_length_value) + String(" bytes"));
    return;
  }

  in_progress_checksum = file_checksum_characteristic.read32();

  int in_progress_file_buffer_index;
  if (finished_file_buffer_index == 0) {
    in_progress_file_buffer_index = 1;
  } else {
    in_progress_file_buffer_index = 0;
  }
  
  in_progress_file_buffer = &file_buffers[in_progress_file_buffer_index][0];
  in_progress_bytes_received = 0;
  in_progress_bytes_expected = file_length_value;

  notifyInProgress();
}

void cancelFileTransfer() {
  if (in_progress_file_buffer != nullptr) {
    notifyError("File transfer cancelled");
    in_progress_file_buffer = nullptr;
  }
}

void onCommandWritten(uint16_t conn_hdl, BLECharacteristic* characteristic, uint8_t* data, uint16_t len) {
  int32_t command_value = (int32_t) characteristic->read32();

  if ((command_value != 1) && (command_value != 2)) {
    notifyError(String("Bad command value: Expected 1 or 2 but received ") + String(command_value));
    return;
  }

  if (command_value == 1) {
    startFileTransfer();
  } else if (command_value == 2) {
    cancelFileTransfer();
  }

}

// Starts the BLE handling you need to support the file transfer.
void setupBLEFileTransfer(BLEService service) {
 

  // Add in the characteristics we'll be making available.
  file_block_characteristic.setWriteCallback(onFileBlockWritten);
  file_block_characteristic.begin();

  file_length_characteristic.begin();

  file_maximum_length_characteristic.begin();
  file_maximum_length_characteristic.write32((int) file_maximum_byte_count);

  file_checksum_characteristic.begin();

  command_characteristic.setWriteCallback(onCommandWritten);
  command_characteristic.begin();

  transfer_status_characteristic.begin();
  error_message_characteristic.begin();

  Serial.println("BLE setup and advertising");
}


// Called in your loop function to handle BLE housekeeping.
void updateBLEFileTransfer() {
//  BLEDevice central = BLE.central();
//  static bool was_connected_last = false;
//  if (central && !was_connected_last) {
//    Serial.print("Connected to central: ");
//    Serial.println(central.address());
//  }
//  was_connected_last = central;
}


bool isTransfering(){
  return in_progress_file_buffer != nullptr;
}

} // namespace
