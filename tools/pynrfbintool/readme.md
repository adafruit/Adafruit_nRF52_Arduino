# pynrfbintool

pynrfbintool.py takes a binary (.bin) application file and generates two main outputs:

### 1. Bootloader Application Config Settings (for manual updates)

The bootloader uses the last page of flash memory (0x3FC00) to store various bootloader config settings, including the user application size and a CRC to to verify user application integrity at start up. The config setting structure is show below (based on the `bootloader_settings_t` struct):

	[0x7f000] = Application state (1 is valid)
	[0x7f004] = Application CRC16
	[0x7f008] = Bank 1 state (should be set to 0xFE)
	[0x7f00c] = Application size

When we flash a new user application to an nRF51822 module that uses the DFU bootloader, we also have to update the bootloader config settings with these values.  If these values aren't present, the bootloader will reject the user application code as invalid.

This script takes the application .bin file, calculates the CRC16 of it, then writes all the above info to a new .bin file with the `_signature.bin` suffix. The `_signature.bin` file can then be converted to a `_signature.hex` (in Intel Hex format) with an offset of 0x3FC00, which can be flashed to the nRF51822 at the appropriate address, causing the bootloader to treat the firmware as a valid image.

### 2. Init File (for OTA updates)

If you wish to use the over the air (OTA) features of the bootloader to update your nRF51822 via an Android or iOS device, you need a specially formatted `_init.dat` file.  This file provides meta-data about the firmware you are trying to transfer to the target device, such as the target chipset, SD version, etc.  The main goal of this file is to prevent flashing an inappropriate image for your device.

In addition to calculating the application config settings when manually flashing application code (as described above), pynrfbintool will also generate an appropriate `_init.dat` file for OTA updates. 

# Syntax

The pynrfbintool script has the following syntax, where the `input` .bin file is the only mandatory parameter:

```
usage: pynrfbintool.py [-h] [--signature FILENAME] [--dfu-init FILENAME] [-q]
                       input

nRF51822 binary signature tool

positional arguments:
  input                 input binary filename

optional arguments:
  -h, --help            show this help message and exit
  --signature FILENAME  filename for the generated signature. Defaults to
                        <input file>_signature.bin
  --dfu-init FILENAME   filename for the generated DFU init data. Defaults to
                        <input file>_init.dat
  -q, --quiet           disable all console output unless an error occurs
```

# nrfbintool.c

nrfbintool.c is provided for reference only if a binary implementation or pynrfbintool is ever required.  pynrfbintool.py should be used in any production environment, though, as it is cross-platform and doesn't require any additional build steps to function.
