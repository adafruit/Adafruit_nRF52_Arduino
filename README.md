# Arduino Core for Nordic Semiconductor nRF5 based boards

## Installing

### Tools

#### Jlink
	
    Download and install jlink from segger

#### nrfutil
  
 - cd tools/nrfutil-0.5.2
 - sudo pip install nrfutil

#### nrfjprog

For burning bootloader with jlink, you will nrfjprog. Not required if you only upload sketch.

- download nRF5x-Command-Line-Tools-OSX/Linux/Win32 https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832#Downloads
- Extract downloaded file and add extracted path to Environment Path
- Check to make sure you could run nrfjprog from your terminal/command prompt
  
### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (At least v1.6.12)
 2. Start the Arduino IDE
 3. Go into Preferences
 4. Add ```https://sandeepmistry.github.io/arduino-nRF5/package_nRF5_boards_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Nordic Semiconductor nRF5 Boards"
 6. Select your nRF5 board from the Tools -> Board menu

### From git (for core development)

 1. Follow steps from Board Manager section above
 2. ```cd <SKETCHBOOK>```, where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
  * OS X: ```~/Documents/Arduino```
  * Linux: ```~/Arduino```
  * Windows: ```~/Documents/Arduino```
 3. Create a folder named ```hardware/Adafruit```, if it does not exist, and change directories to it
 4. Clone this repo: `git clone git@github.com:adafruit/Adafruit_nRF52_Arduino.git`
 5. Restart the Arduino IDE

## BLE

This Arduino Core does **not** contain any Arduino style API's for BLE functionality. All the relevant Nordic SoftDevice (S110, S130, S132) header files are included build path when a SoftDevice is selected via the `Tools` menu.

### Recommend BLE Libraries

 * [BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral)
   * v0.3.0 and greater, available via the Arduino IDE's library manager.
   * Supports peripheral mode only.

## Credits

This core is based on the [Arduino SAMD Core](https://github.com/arduino/ArduinoCore-samd) and licensed under the same [GPL License](LICENSE)

The following tools are used:

 * [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) as the compiler
 * A [forked](https://github.com/sandeepmistry/openocd-code-nrf5) version of [OpenOCD](http://openocd.org) to flash sketches
