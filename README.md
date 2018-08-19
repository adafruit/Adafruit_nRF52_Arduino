# Arduino Core for Adafruit Bluefruit nRF52 Boards

This repository contains the Arduino BSP for Adafruit nRF52 series:

- [Bluefruit Feather nRF52832](https://www.adafruit.com/product/3406)

## BSP Installation

There are two methods that you can use to install this BSP. We highly recommend the first option unless you wish to participate in active development of this codebase via Github.

### Recommended: Adafruit nRF52 BSP via the Arduino Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add https://www.adafruit.com/package_adafruit_index.json as an 'Additional Board Manager URL'
 5. Restart the Arduino IDE
 6. Open the Boards Manager from the Tools -> Board menu and install 'Adafruit nRF52 by Adafruit'
 7. Once the BSP is installed, select 'Adafruit Bluefruit nRF52 Feather' from the Tools -> Board menu, which will update your system config to use the right compiler and settings for the nRF52.

### Optional (Core Development): Adafruit nRF52 BSP via git

 1. Install BSP via Board Manager as above to install compiler & tools.
 2. Delete the core folder `nrf52` installed by Board Manager in Adruino15, depending on your OS. It could be
  * OS X   : `~/Library/Arduino15/packages/adafruit/hardware/nrf52`
  * Linux  : `~/.arduino15/packages/adafruit/hardware/nrf52`
  * Windows: `%APPDATA%\Local\Arduino15\packages\adafruit\hardware\nrf52`
 3. `cd <SKETCHBOOK>`, where `<SKETCHBOOK>` is your Arduino Sketch folder:
  * OS X   : `~/Documents/Arduino`
  * Linux  : `~/Arduino`
  * Windows: `~/Documents/Arduino`
 4. Create a folder named `hardware/Adafruit`, if it does not exist, and change directories to it
 5. Clone this repo: `git clone https://github.com/adafruit/Adafruit_nRF52_Arduino.git`
 6. Restart the Arduino IDE
 7. Once the BSP is installed, select 'Adafruit Bluefruit nRF52 Feather' from the Tools -> Board menu, which will update your system config to use the right compiler and settings for the nRF52.

### Adafruit's nrfutil tools

[adafruit-nrfutil](https://github.com/adafruit/Adafruit_nRF52_nrfutil) (derived from Nordic pc-nrfutil) is needed to upload sketch via serial port.

- For Windows and macOS, pre-built executable binaries are included in the BSP at `tools/adafruit-nrfutil/`. It should work out of the box.
- Linux user need to run follow command to install it from PyPi

    ```
    $ pip3 install adafruit-nrfutil --user
	```

## Arduino BLE Application Support

This Arduino core contains basic BLE peripheral mode helper classes and an initial peripheral mode
API. These helper classes and APIs aim to make it easier to work with the Nordic SoftDevice that
contains Nordic's official Bluetooth Low Energy stack. You are also free to use the Nordic SDK to
generate your own example code, since all of the SoftDevice header files are included in your
projects by default.

To see a list of example sketches that make use of these helper classes, select the appropriate
board from the `Tools > Board` menu item, and then in the `Examples` menu look for the list of
examples sketched for the selected board.

## Bootloader Support

### Upgrade existiting Bootloader

Bluefruit's Bootloader is self-upgradable, you could upgrade to the latest Bootloader + Softdevice using the serial port within Arduino IDE.

- Select `Tools > Board > Adafruit Bluefruit Feather52`
- Select `Tools > Programmer > Bootloader DFU for Bluefruit nRF52`
- Select `Tools > Burn Bootloader`
- **WAIT** until the process complete ~30 seconds

Note: close the Serial Monitor before you click "Burn Bootloader". Afterwards, you shouldn't close the Arduino IDE, unplug the Feather, launch Serial Monitor etc ... to abort the process. There is a high chance it will brick your device! Do this with care and caution.

### Burnning new Bootloader

To burn the bootloader from within the Arduino IDE, you will need the following tools installed
on your system and available in the system path:

- Segger [JLink Software and Documentation Pack](https://www.segger.com/downloads/jlink)
- Nordic [nRF5x Command Line Tools](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_installation.html)

Check to make sure you can run `nrfjprog` from your terminal/command prompt

**OS X Note** At present, you will need to create a symlink in `/usr/local/bin` to the
`nrfjprog` tool wherever you have added it. You can run the following command, for example:

```
$ ln -s $HOME/prog/nordic/nrfjprog/nrfjprog /usr/local/bin/nrfjprog
```

Once the tools above have been installed and added to your system path, from the Arduino IDE:

- Select `Tools > Board > Adafruit Bluefruit Feather52`
- Select `Tools > Programmer > J-Link for Feather52`
- Select `Tools > Burn Bootloader` with the board and J-Link connected

#### Manually Burning the Bootloader via nrfjprog

The bootloader hex file can be found at `bin/bootloader` run the command as follows:

```
$ nrfjprog -e -f nrf52
$ nrfjprog --program feather_nrf52832_bootloader.hex -f nrf52
$ nrfjprog --reset -f nrf52
```

## Misc Notes

#### nRF52DK Jlink Issue on OS X

If developping with the nRF52DK on OS X, there is a bug where only 64 bytes can be sent
over the USB CDC interface, which will prevent you from using the serial bootloader from
the Arduino IDE with an error like this:

```
Upgrading target on /dev/cu.usbmodem1421 with DFU package /private/var/folders/86/hb2vp14n5_5_yvdz_z8w9x_c0000gn/T/arduino_build_267869/nRF51Blinky.ino.zip. Flow control is disabled.


Timed out waiting for acknowledgement from device.

Failed to upgrade target. Error is: No data received on serial port. Not able to proceed.

Possible causes:
- bootloader, SoftDevice or application on target does not match the requirements in the DFU package.
- baud rate or flow control is not the same as in the target bootloader.
- target is not in DFU mode. If using the SDK examples, press Button 4 and RESET and release both to enter DFU mode.
```

To resolve this and enable 512 byte packets over USB serial, you must disable the
Mass Storage Device interface on the JLink-OB, which will free up two of the 512 byte
USB end points. (For details see [this article](https://wiki.segger.com/index.php?title=J-Link-OB_SAM3U).) 

You can do so by running `JLinkExe` from the command line, and then entering the
`MSDDisable` command, and power cycling your nRF52DK. To re-enable MSD support, do the same
but enter the `MSDEnable` command.

## Credits

This core is based on [Arduino-nRF5](https://github.com/sandeepmistry/arduino-nRF5) by Sandeep Mistry,
which in turn is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd).

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * adafruit-nrfutil is based on Nordic Semiconductor ASA's [pc-nrfutil](https://github.com/NordicSemiconductor/pc-nrfutil)
