# Python nRF Binary Tool
# Generate a signature and DFU init packet for an nRF51822 binary.
# Note, can run unit tests by executing inside the same directory as this file:
#   python -m unittest pynrfbintool 
# Author: Tony DiCola
import argparse
import os
import struct
import unittest

# Constants and other config values.
SIGNATURE_EXTENSION = '_signature.bin'
BANK_VALID_APP      = 0x01
BANK_VALID_SD       = 0xB7
BANK_VALID_BOOT     = 0xAA
BANK_ERASED         = 0xFE
BANK_INVALID_APP    = 0xFF
DFU_SOFTDEVICE_ANY  = 0xFFFE

# Define the C structures used for the signature file format.
# typedef struct
# {
#     uint32_t bank_0;          /**< Variable to store if bank 0 contains a valid application. */
#     uint32_t bank_0_crc;      /**< If bank is valid, this field will contain a valid CRC of the total image. */
#     uint32_t bank_1;          /**< Variable to store if bank 1 has been erased/prepared for new image. Bank 1 is only used in Banked Update scenario. */
#     uint32_t bank_0_size;     /**< Size of active image in bank0 if present, otherwise 0. */
#     uint32_t sd_image_size;   /**< Size of SoftDevice image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t bl_image_size;   /**< Size of Bootloader image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t app_image_size;  /**< Size of Application image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t sd_image_start;  /**< Location in flash where SoftDevice image is stored for SoftDevice update. */
# } bootloader_settings_t;
bootloader_settings = struct.Struct('<IIIIIIII')

def crc16(data):
    """Generate the 16-bit CRC of the provided input string and return it."""
    crc = 0xFFFF
    for c in data:
        crc = (((crc >> 8) & 0xFF) | (crc << 8)) & 0xFFFF
        crc = (crc ^ (c if isinstance(c, int) else ord(c))) & 0xFFFF
        crc = (crc ^ ((crc & 0xFF) >> 4)) & 0xFFFF
        crc = (crc ^ ((crc << 8) << 4)) & 0xFFFF
        crc = (crc ^ (((crc & 0xFF) << 4) << 1)) & 0xFFFF
    return crc


def create_app_signature(data):
    """Generate a signature for the provided input data and return it.  Input
    should be a string of bytes with the input data.
    """
    settings = bootloader_settings.pack(BANK_VALID_APP,  # Bank 0
                                        crc16(data),     # Bank 0 CRC16
                                        BANK_ERASED,     # Bank 1
                                        len(data),       # Bank 0 length
                                        0,               # SD image size
                                        0,               # BL image size
                                        0,               # App image size
                                        0)               # SD image start
    return settings

if __name__ == '__main__':
    # Parse command line arguments.
    parser = argparse.ArgumentParser(description='nRF51822 binary signature tool')
    parser.add_argument('input',
                        action='store',
                        help='input binary filename')
    parser.add_argument('--signature',
                        action='store',
                        help='filename for the generated signature.  Defaults to <input file>_signature.bin',
                        metavar='FILENAME')
    parser.add_argument('-q', '--quiet',
                        action='store_true',
                        help='disable all console output unless an error occurs')
    args = parser.parse_args()

    # Make sure input filename exists.
    if not os.path.isfile(args.input):
        raise RuntimeError('Input file does not exist!')

    # Set default output filenames if not set.
    prefix = os.path.splitext(args.input)[0]
    signature = args.signature
    if signature is None:
        signature = prefix + SIGNATURE_EXTENSION

    # Read in all the input data.
    with open(args.input, 'rb') as in_file:
        data = in_file.read()
    if data is None or len(data) == 0:
        raise RuntimeError('Input file has no data!')

    # Generate and write signature.
    if not args.quiet:
        print('Writing signature to:     {0}'.format(signature))
    with open(signature, 'wb') as out_file:
        out_file.write(create_app_signature(data))

    # Done!
    if not args.quiet:
        print('Done!')


# Unit Tests.
class UnitTests(unittest.TestCase):

    def test_crc16(self):
        self.assertEqual(crc16('\x00'),  0xE1F0)
        self.assertEqual(crc16('Hello'), 0xDADA)

    def test_signature(self):
        expected = '\x01\x00\x00\x00\xDA\xDA\x00\x00\xFE\x00\x00\x00\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.assertEqual(create_app_signature('Hello'), expected)

