# Arduino Core for Nordic Semiconductor nRF5 based boards

## BSP Installation
  
### Arduino nRF5x BSP via Board Manager

**NOTE:** For now, you need to install this BSP to gain access to the compiler. In the future, only the Adafruit nRF52 BSP will be required.

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://sandeepmistry.github.io/arduino-nRF5/package_nRF5_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Nordic Semiconductor nRF5 Boards"
 6. Select your nRF5 board from the Tools -> Board menu

### Adafruit nRF52 BSP via git (for core development)

 1. Follow steps from Board Manager section above
 2. ```cd <SKETCHBOOK>```, where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
  * OS X: ```~/Documents/Arduino```
  * Linux: ```~/Arduino```
  * Windows: ```~/Documents/Arduino```
 3. Create a folder named ```hardware/Adafruit```, if it does not exist, and change directories to it
 4. Clone this repo: `git clone git@github.com:adafruit/Adafruit_nRF52_Arduino.git`
 5. Restart the Arduino IDE

### Third Party Tools

#### nrfutil

The Adafruit nRF52 BSP includes a [python wrapper](https://github.com/NordicSemiconductor/pc-nrfutil) for Nordic's `nrfutil`, which is used to flash boards. Go into the BSP folder (`hardware/Adafruit/Adafruit_nRF52_Arduino/tools/nrfutil-0.5.2`), and run the following to make this available to the Arduino IDE:

```
$ cd tools/nrfutil-0.5.2
$ sudo pip install -r requirements.txt
$ sudo python setup.py install
```

**Notes** : Don't install nrfutil from the pip package (ex. `sudo pip install nrfutil`). The latest nrfutil does not support DFU via Serial, and you should install the local copy of 0.5.2 included with the BSP via the `python setup.py install` command above.

## Arduino BLE Application Support

This Arduino Core does **not** contain any Arduino style API's for BLE functionality. All the relevant Nordic SoftDevice (S110, S130, S132) header files are included in the build path when a SoftDevice is selected via the `Tools` menu, and you can use the Nordic SDK to generate your own example code, or use one of the following libraries:

### Recommend BLE Libraries

  * [Adafruit_nRF52_Arduino_Helpers](https://github.com/adafruit/Adafruit_nRF52_Arduino_Helpers)
    * Recommended BLE library to use with the Adafruit nRF52 Feather board
    * Contains base classes for BLE services and characteristics, and helper classes for BLE UART, DIS (Device Information Service) and OTA DFU.
  * [BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral)
   * v0.3.0 and greater, available via the Arduino IDE's library manager.
   * Supports peripheral mode only.

## Bootloader Support

### Third Party Tools

To burn the bootloader from within the Arduino IDE, you will need the following tools installed on your system and available in the system path:

#### Jlink Driver and Tools

Download and install the [JLink Software and Documentation Pack](https://www.segger.com/downloads/jlink) from Segger, which will also install a set of command line tools.

#### pynrfjprog

In order to burn the bootloader with a J-Link, you will need `pynrfjprog` from Nordic:

```
$ sudo pip install pynrfjprog
```

### Burning the Bootloader

Once the tools above have been installed and added to your system path, from the Arduino IDE:

- Select `Tools > Board > Adafruit Bluefruit Feather52`
- Select `Tools > Programmer > J-Link for Feather52`
- Select `Tools > Burn Bootloader` with the board and J-Link connected

#### Manually Burning the Bootloader via nrfjprog

You can also manually burn the bootloader from the command line, you will need `nrfjprog` from Nordic: 

- Download [nRF5x-Command-Line-Tools](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832#Downloads) for OSX/Linux/Win32
- Extract the downloaded file and add the extracted path to your environment `PATH` variable
- Check to make sure you can run `nrfjprog` from your terminal/command prompt

**OS X Note** At present, you will need to create a symlink in `/usr/local/bin` to the `nrfjprog` tool wherever you have added it. You can run the following command, for example:

```
$ ln -s $HOME/prog/nordic/nrfjprog/nrfjprog /usr/local/bin/nrfjprog
```

Then run the command as follows:

```
$ nrfjprog -e -f nrf52
$ nrfjprog --program bootloader_with_s132.hex -f nrf52
$ nrfjprog --reset -f nrf52
```

## Misc Notes

#### nRF52DK Jlink Issue on OS X

If developping with the nRF52DK on OS X, there is a bug where only 64 bytes can be sent over the USB CDC interface, which will prevent you from using the serial bootloader from the Arduino IDE with an error like this:

```
Upgrading target on /dev/cu.usbmodem1421 with DFU package /private/var/folders/86/hb2vp14n5_5_yvdz_z8w9x_c0000gn/T/arduino_build_267869/nRF51Blinky.ino.zip. Flow control is disabled.


Timed out waiting for acknowledgement from device.

Failed to upgrade target. Error is: No data received on serial port. Not able to proceed.

Possible causes:
- bootloader, SoftDevice or application on target does not match the requirements in the DFU package.
- baud rate or flow control is not the same as in the target bootloader.
- target is not in DFU mode. If using the SDK examples, press Button 4 and RESET and release both to enter DFU mode.
```

To resolve this and enable 512 byte packets over USB serial, you must disable the Mass Storage Device interface on the JLink-OB, which will free up two of the 512 byte USB end points. (For details see [this article](https://wiki.segger.com/index.php?title=J-Link-OB_SAM3U).) 

You can do so by running `JLinkExe` from the command line, and then entering the `MSDDisable` command, and power cycling your nRF52DK. To re-enable MSD support, do the same but enter the `MSDEnable` command.

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd) and licensed under the same [GPL License](LICENSE)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
