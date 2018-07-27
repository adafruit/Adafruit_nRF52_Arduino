/**************************************************************************/
/*!
    @file     nrfbintool.c
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define SIGNATURE_EXTENSION "_signature.bin"
#define DFU_INIT_EXTENSION  "_init.dat"

typedef enum
{
    BANK_VALID_APP   = 0x01,
    BANK_VALID_SD    = 0xA5,
    BANK_VALID_BOOT  = 0xAA,
    BANK_ERASED      = 0xFE,
    BANK_INVALID_APP = 0xFF,
} bootloader_bank_code_t;

/**@brief Structure holding bootloader settings for application and bank data.
 */
typedef struct
{
    uint32_t bank_0;          /**< Variable to store if bank 0 contains a valid application. */
    uint32_t bank_0_crc;      /**< If bank is valid, this field will contain a valid CRC of the total image. */
    uint32_t bank_1;          /**< Variable to store if bank 1 has been erased/prepared for new image. Bank 1 is only used in Banked Update scenario. */
    uint32_t bank_0_size;     /**< Size of active image in bank0 if present, otherwise 0. */
    uint32_t sd_image_size;   /**< Size of SoftDevice image in bank0 if bank_0 code is BANK_VALID_SD. */
    uint32_t bl_image_size;   /**< Size of Bootloader image in bank0 if bank_0 code is BANK_VALID_SD. */
    uint32_t app_image_size;  /**< Size of Application image in bank0 if bank_0 code is BANK_VALID_SD. */
    uint32_t sd_image_start;  /**< Location in flash where SoftDevice image is stored for SoftDevice update. */
} bootloader_settings_t;

/**
 * crc16 - compute the CRC-CCITT (0xFFFF) for the data buffer
 * @buffer:	data pointer
 * @len:	number of bytes in the buffer
 *
 * Returns the updated CRC value.
 */
uint16_t crc16(uint8_t const *buffer, size_t len)
{
  uint16_t crc = 0xffff;

  int i;
  for (i = 0; i < len; i++)
  {
    crc  = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= buffer[i];
    crc ^= (unsigned char)(crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
  }

	return crc;
}

uint32_t get_filesize(char* filename)
{
  uint32_t filesize;

  FILE* fin = fopen(filename, "rb");
  assert(fin != NULL);

  // get file size
  fseek(fin, 0, SEEK_END);
  filesize = ftell(fin);
  fseek(fin, 0, SEEK_SET);

  fclose(fin);

  return filesize;
}

uint16_t crc16_of_file(char *filename)
{
  uint32_t filesize = get_filesize(filename);
  assert(filesize > 0);

  // open input binary file
  FILE* fin = fopen(filename, "rb");
  assert(fin != NULL);

  //malloc and compute crc
  uint8_t* buf = (uint8_t*) malloc(filesize);
  (void) fread(buf, 1, filesize, fin);

  uint16_t crc = crc16(buf, filesize);

  free(buf);
  fclose(fin);

  return crc;
}

void create_app_signature(char *filename)
{
  // this signature structure depends on bootloader_settings_t
  bootloader_settings_t signature =
  {
      .bank_0      = BANK_VALID_APP,
      .bank_0_crc  = crc16_of_file(filename),
      .bank_1      = BANK_ERASED,
      .bank_0_size = get_filesize(filename)
  } ;

  // output name = input_name (no extension) + _signature .bin
  char output_name[256] = { 0 };
  memcpy(output_name, filename, strlen(filename) - 4); // dont copy extension
  strcat(output_name, SIGNATURE_EXTENSION);

  // Create output binary file
  FILE* fout = fopen(output_name, "wb");
  assert(fout != NULL );

//  fwrite(&signature, 4, sizeof(signature)/4, fout);
  fwrite(&signature, 1, sizeof(signature), fout);
  fclose(fout);
}

#define U32_FROM_U16(high, low)     ((uint32_t) (((high) << 16) | (low)))

#define DFU_SOFTDEVICE_ANY                  ((uint16_t)0xFFFE)

void create_app_dfu_init_packet(char* filename)
{
  typedef struct
  {
    uint16_t device_type;    /**< Device type (2 bytes), for example Heart Rate. This number must be defined by the customer before production. It can be located in UICR or FICR. */
    uint16_t device_rev;     /**< Device revision (2 bytes), for example major revision 1, minor revision 0. This number must be defined by the customer before production. It can be located in UICR or FICR. */
    uint32_t app_version;    /**< Application version for the image software. This field allows for additional checking, for example ensuring that a downgrade is not allowed. */
    uint16_t softdevice_len; /**< Number of different SoftDevice revisions compatible with this application. The list of SoftDevice firmware IDs is defined in @ref softdevice. */
    uint16_t softdevice[1];  /**< Variable length array of SoftDevices compatible with this application. The length of the array is specified in the length field. SoftDevice firmware id 0xFFFE indicates any SoftDevice. */
    uint16_t app_crc16;
  } dfu_init_packet_t;

  dfu_init_packet_t init_packet =
  {
      .device_type    = 0xffff,
      .device_rev     = 0xffff,
      .app_version    = U32_FROM_U16(0, 0),
      .softdevice_len = 1,
      .softdevice     = { DFU_SOFTDEVICE_ANY  },
      .app_crc16      = crc16_of_file(filename)
  };

  // output name = input_name (no extension) + _signature .bin
  char output_name[256] = { 0 };
  memcpy(output_name, filename, strlen(filename) - 4); // dont copy extension
  strcat(output_name, DFU_INIT_EXTENSION);

  // Create output binary file
  FILE* fout = fopen(output_name, "wb");
  assert(fout != NULL );

  fwrite(&init_packet, 1, sizeof(init_packet), fout);
  fclose(fout);
}

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    printf("%s <filename>", argv[0]);
    return 0;
  }

  char *filename = argv[1];

  create_app_signature(filename);
  create_app_dfu_init_packet(filename);

  return 0;
}
