# Adafruit nRF52 Arduino Core Changelog

## 0.19.0 - 2020.03.12

- Add BLECharacteristic::isFixedLen()
- Enhance and add new Playground sensor services
  - Quaternion
  - Sound
  - Color
  - Proximity
  - Gesture
- Fix sporadic hangup when stopping notifies, thanks to @FariSoftware PR #444
- Rename image_upload sketch to image_transfer
- Update bluefruit_playground sketch to work with CPB, CLUE and Sense
- Upgrade SystemView to v3.10, thanks to @henrygab PR #437
- Fix D13 LED_BUILTIN for Feather Sense
- Skip waiting for Serial in some examples, force wait for Serial when Debug is enabled
- Update bootloader binary from 0.3.0 to 0.3.2

## 0.18.5 - 2020.02.19

- Add macro `SPI_32MHZ_INTERFACE` to variant to select SPI or SPI1 to use 32mhz SPIM3
- Add PIN_BUZZER to variants with built-in speaker
- Enhance Particle Xenon support, PR #435 thanks to @outlandnish and @jaswope
- Rename cplay_ble.ino to bluefruit_playground.ino
- Use USB_PRODUCT string for default bledis model
- Increase attr table size for 840 to from 0xC00 to 0x1000
- Add more Adafruit sensor service: Gyro, Magento, Humid, Baro
- Upadte `image_upload.ino` to support CLUE with built-in TFT

## 0.18.0 - 2020.02.03

- Add EXTERNAL_FLASH_USE_QSPI to all variants that has on-board flash

## 0.17.0 - 2020.02.01

### Core

- Add board support for Adafruit CLUE, Feather nRF52840 Sense
- Add board support Raytac MDBT50Q_RX dongle, thanks to @pyro9 PR #328
- Update bootloader binary to 0.3.0
- Addd PIN_BUTTON for variants
- Add SoftwareTimer getID/setID
- Add **INPUT_PULLUP/DOWN_SENSE** mode, PR #427, thanks to @jpconstantineau
- Support Serial event parity SERIAL_8E1, PR #369, thanks to @prjh
- Support Serial2, migrate uart driver to UARTE, PR #315, thanks to @ogatatsu
- Fix various bugs

### BLE

- Add **image_eink_upload8* example sketch
- Add Nordic Led Button service example **nrf_blinky** sketch
- Increase SD RAM for nRF52832
- Fix Discovery bug, PR #413, thanks to @ogatatsu

## 0.16.0 - 2020.01.10

- Fix Little FileSystem (LFS) reentrance issue, by serialize access to lfs_* API(). Huge thanks to @hentrygab for spending lots of his time on PR #397
- Fix SPIM transfer with length > 64 KB bytes
- Fix #352 PDM issue
- SchedulerRTOS::startLoop() support stack size and task priority parameters

## 0.15.1 - 2019.12.31

- Print::availableForWrite() return int instead of size_t
- Migrate CI from travis to actions

## 0.15.0 - 2019.12.30

### Core

- Add Clue variant
- Clean up warnings, thanks to @henrygab
- Enhance Software Timer

### BLE
 
- Increase sd attribute table size from 0x800 to 0xC00, increase linker memory for SD 840 from 3400 to 6000
- Add Adafruit BLE Service library (used by Circuit Playground Bluefruit App): Temperature, Addressable Pixel, Accel, Button
  - Add `cplay_ble.ino` example sketch
- Change BLEUuid begin's return type to bool
- BLECharacteristic allow user to set buffer

### USB

- Moved TinyUSB core into submodule at https://github.com/adafruit/Adafruit_TinyUSB_ArduinoCore
- Added USBD detach/attach API
- Synced TinyUSB with upstream

## 0.14.6 - 2019.10.30

- Added power switch pin for Circuit Playground Bluefruit
- Make min/max templates

## 0.14.5 - 2019.10.21

- Added Itsy nRF52840 Express support
- Replace legacy SPI by SPIM3 with maximum 32Mhz for nRF52840
- Added support for 2nd I2C interface aka Wire1
- Macro defines clean up
  - remove ARDUINO_FEATHER52
  - remove ARDUINO_NRF52_FEATHER use either ARDUINO_NRF52832_FEATHER or ARDUINO_NRF52840_FEATHER
  - use ARDUINO_NRF52840_CIRCUITPLAY for Circuit Playground Bluefruit
- ARDUINO_NRF52840_CIRCUITPLAY defined for circuit playground bluefruit
- Updated nrfx module to 1.7.2
- Fixed more warnings. better describe flash caching, PR #347 thanks to @henrygab
- Improve tinyUSB thanks to @kaysievers
  - Fixed warnings
  - Allows configuration of power setting
  - Allows to set USB Manufacturer/product ID
  - Allow to set configuration descriptor buffer
- Fixed missing bootloader binaries

## 0.14.0 - 2019.09.27

- Core
  - Ada Callback task dynamically resize its queue size on demand. Also invoke function immediately if callback failed to allocate memory for deferring.
  - Changde stack size for following task
    - Task loop     : from 256*6 to 256*4
    - Task Callback : from 256*4 to 256*3
    - Task USBD     : from 150   to 200
    - Task BLE      : from 256*6 to 256*5
  - Added _sbkr() to handle heap overflowed
- BLEUart
  - Added setRxOverflowCallback()
  - Added deferred option for setRxCallback()
- Update `image_upload` example to work with both nRF52832 & nRF52840 with maximum throughput, also support 16 or 24-bit color
- Update bootloader binary to to 0.2.13 version (upgrade is optional)
- Enhance BLEDis with new characteristic : system id, reg cert list, pnp id. PR #336 Thanks to @elral
- Fixed SPI definition for circuitplayground Bluefruit

## 0.13.0 - 2019.08.22

- Update bootloader binary to to 0.2.12 version (upgrade is optional)
- Added Circuit Playground Blueffuit nRF52840 support
- Added variant support for Particle Xenon, PR #317
- Added PDM support
- Switch compiler optimization from -Os to -Ofast
- Improve BLEConnection throughput
  - Added requestPHY() to initate PHY switching to 2 MB, 1MB
  - Added requestDataLengthUpdate() to initate Data Length Update process
  - Added requestMtuExchange() to iniate MTU exchange procoess
  - Added requestConnectionParameter() to initate connection parameter process
- Updated throughput and added central_throughput sketch (WIP)
- Added image_upload example
- Enhance mutex lock/unlock for fifo used by bleuart
- Fixed multiples warning with tinyUSB descriptor template, PR #322
- Fixed typo, PR #326

## 0.12.0 - 2019.08.05

- Update tinyusb core to support webUSB & vendor class
- Added a couple delete operators to make std=gnu++14/17, PR #312 thanks to @kevinfrei
- Fix Serial.read() return clean 8-bit, PR #308 thanks to @pyro9
- Added availableForWrite(), PR #311 thanks to @pyro9

## 0.11.1 - 2019.07.10

- Update tinyusb core to support USB MIDI
- Refactor Ada Callback, use ISCR to detect isr context. Use function instead of macro
- Implement #240 run travis test with all example sketches
- Fixed auto-start of advertising when central is connected, thanks to @ogatatsu PR #268
- Added Tone()/noTone() functions
- Travis-ci builds all sketches when commit code
- Fixed setAppearance/getAppearance() typo, thanks to @paulmand3l PR #292
- Fixed rssi_proximity_peripheral sketch, thanks to @dicobrazz PR #295
- Fixed doc typo, thanks to @yvadher PR #296
- Fixed HID usage code location comment in exmaple sketch, thanks to @stefandz PR #297
- Fixed #277 conn LED doesn't stop when scanner is time out
- Added connection handle to Bluefruit.connParied()

## 0.11.0

- Rework USB driver to support Adafruit_TinyUSB library (support HID and MSC)
- Added Metro nRF52840 Express
- Update bootloader binaries to 0.2.11
- Rework Filesystem
  - Seperate LittleFS and InternalFS into `Adafruit_LittleFS` and `InternalFileSystem`
  - Remove ExternalFS in favor of using Adafruit_QSPI and Adafruit_SPIFlash library
- Update nrfx to 1.6.2
- Fixed #250 wrong values for g_ADigitalPinMap, thanks to @henrygab
- Fixed interrupts for device with multiple I/O Port, thanks to MacGyverNL PR #261
- Update adc_vbat.ino sketch to work with nrf52840, thanks to @pyro9
- Extend SoftwareTimer with option to make it non-repeating, add reset function & ISR-safe functions, thanks to @MacGyverNL PR #260
- Fixed connection handle in BLEHidAdafruit single connection api, thanks to @ogatatsu PR #267
- Fixed spelling & Add Environmental Sensing GATT Service and UV Index GATT Characteristics UUID, thanks to @sayanee
- Fixed #276 rename macro FILE_READ/WRITE to enum FILE_O_READ/WRITE
- Upgrade compiler toolchain from gcc 5.2 2015q2 to gcc 7 2017q4
- Rename hid client setProtocolMode() to setBootMode()
- Removed `rtos_idle_callback()`, sketch define vApplicationIdleHook() if needed
- enhance Serial.available()/write() to prevent blocking wait without yield/delay
- Clean up compiler warnings

## 0.10.1

This release added multiple concurrent peripheral connections support, allow Bluefruit device to multiple central (phones/PC) simultaneously. It introduces new BLE class: BLEPeriph, BLEConnection, remove BLEGap, refactor/rename/move functions and callbacks.     

- Fixed Servo detach issue
- Fixed pulseIn() compile issue: implement countPulseASM() using C instead of ASM
- Update bootloader to 0.2.9
  - Fixed OTA issue with latest BLE5 central such as iPhone X
  - Fixed incomplete writes on Windows. Updated tinyusb to handle write10 completion, and use it for finalizing the DFU process
- Fixed various warnings, thanks @brijohn
- Added ARDUINO_NRF52832_FEATHER for feather 832, ARDUINO_NRF52840_FEATHER for feather 840, ARDUINO_NRF52_FEATHER for both
- Fixed an memory leak with LFS, also extend to allow it to be used with SPI flash on other boards. Thanks @jeremypoulter
- Seperate OTA DFU service from Bluefruit.begin(), sketch must explicit add it if needed.
- Added multiple peripheral-role connections support, example sketch is at `examples/Peripherals/bleuart_multi`
- Introduce BLEPeriph class (Bluefruit.Periph) to mange peripheral role's connection
  - setConnInterval(), setConnIntervalMS(), setConnSupervisionTimeout(), setConnSupervisionTimeoutMS()
  - setConnectCallback(), setDisconnectCallback()
- Bluefruit
  - Bluefruit.getPeerAddr() is replaced by BLEConnection's getPeerAddr()
  - Bluefruit.connInterval() is replaced by BLEConnection's getConnInterval()
  - Bluefruit.Central.disconnect() is repalced by Bluefruit.disconnect()
  - Bluefruit.begin() return type is changed from err_t to bool
  - Bluefruit.setConnectCallback()/setDisconnectCallback() are replaced by BLEPeriph's setConnectCallback()/setDisconnectCallback()
- Introduce BLEConnection class (Bluefruit.Connection(conn)) to mange both peripheral and central connections
  - Added setRssiCallback(), monitorRssi(), getRssi(), stopRssi() for tracking rssi of a connection. `rssi_poll` and `rssi_callback` are added as example sketches 
- Remove BLEGap, API functions are taken by Bluefruit, BLEPeriph, BLECentral, BLEConnection
  - Gap.setAddr()/getAddr() are replaced by Bluefruit.setAddr()/getAddr()
  - Gap.requestPairing() is replaced by Bluefruit.requestPairing(), conn_handle parameter is also added
  - Most of other functions of BLEGap are replaced by BLEConnection's one
- BLECharacteristic 
  - Change callback signature's parameter from `BLECharacteristic&` to `BLECharacteristic*`
  - conn_handle is added to all callbacks to support multiple peripheral's link
  - Use AdaCallback thread for BLECharacteristic callbacks
  - Support LONG WRITE a.k.a send more than MTU ( default = 20 bytes) per request. This fixed issue #91, #220
  - Fixed read32(), thanks @techno
  - Removed offset parameter in write callback signature
- BLEUart
  - Added conn_handle to API and callbacks
  - Removed auto flush TXD() with timer, user must call flushTXD() should bufferTXD() is enabled. 
- BLEHidAdafruit
  - Removed keyboardReport() variant with flat keycode parameters
  - Added conn_handle parameter to keyboard led callback

## 0.9.3

- Correct bootloader version text in IDE to 0.2.6
- Fixed #173 bleuart return incorrect value when failed to send (PR #178 thanks Nenik)
- Added Client Battery support BLEClientBas
- Added BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST event support for Central
- Added Jlink as programmer to upload sketch #133. Though at least one serial DFU upload is needed to disable firmware crc checking
- Fixed issue with high speed uart baud ~ 1 Mbps (PR #158 thanks Ureloc)
- Add HardwardPWM removePin(), refactor hwpwm.ino sketch
- Fixed print float issue with precision > 10

## 0.9.2

- Fully support Feather nRF52840
- Update bootloader with new led pattern
- Fix #203: return software timer handle

## 0.9.1

- Rename FileIO.h to Bluefruit_FileIO to prevent conflict with other libraries.
- Minor upgrade for bootloader to prevent issue with WDT enabled by application
- 52840: call cdc flush before delay()

## 0.9.0

- Added nRF52840-based board support (pca10056 and feather nRF52840 express)
- Upgrade bootloader to v6.1.1 ( single bank only )
- Upgrade BSP code to match SD v6
  - Bluefruit.Scanner.resume() must be called in scan callback to resume scanning after received an report.
- Upgrade freeRTOS from v8 to v10.0.1
- Upgrade Segger SysView to 2.52d
- Added nrfx to core
- Added tinyusb stack to libraries
- Added LittleFS to replace NFFS
- Added FileSystem Libraries base on SD File API
  - InternalFS use LittleFS to manage internal flash for bonding and other user data
  - ExternalFS use fatfs to manage external spi flash (nrf52840 only) for usb msc.
- Replace cpritnf by std printf
- Added HwPWM3 for nRF52840
- Added ISR_DEFERRED option for attachInterrupt() to defer callback from ISR context
  - Added digital_interrupt_deferred sketch for demo
- Added support for using the Low Frequency RC oscillator ( PR #144 thanks to @jeremypoulter )
  - USE_LFRC or USE_LFXO must be defined in board's variant.h
- Fixed Scanner running state when timeout ( PR #186 thanks to @Ryscho )
- Fixed #192 Client Characteristic write() return number of writtent instead of error code
- Added #181 Bluefruit.setEventCallback() for user to handle ble event

## 0.8.6

- Fixed dbgDumpMemory for buffer > 255 byte, thanks to @airbornemint
- Added setConnSupervisionTimeout and setConnSupervisionTimeoutMS ( PR #177 thanks to @airbornemint )
- Decrease gpio's interrupt level to 2 to avoid conlfict with SD timing critical task ( PR #179 thanks to @Nenik )
- Fixed #174 window build error with Arduino 1.8.6 with verbose = off

## 0.8.5

- Migrate nrfutil to adafruit-nrfutil, executable binaries for windows and macOS are included.
- Implement #166 BLE HID Keyboard LED receive from Central, update hid_keyscan & hid_keyboard example.
- Add hardware's systick sketch example
- Add software timer's sketch example

## 0.8.4

- Fix #160: hardware PWM issue that cause Servo freq is 640 hz instead of 50hz
- Fix #134: hardcoded pulse limits with Servo
- Fix upload issue with windows when username has spaces
- Support serialEvent()

## 0.8.3

- Enhanced indicate API() to wait for confirm or timeout from peer.
- Added BLEScanner filterService() for BLEService and BLEClientService
- Enhanced bonding management to support central bond and re-connection
- Added BLEClientHidAdafruit implementation for client HID
	- Added Central HID example
- Enhanced BLEHidGeneric/BLEHidAdafruit to support boot protocol mode.
- Fixed I2C lock-up when endTransmission() is called with empty txBuffer.
- Fixed #125 : DFU temp memory typo
- Fixed #126 : setWriteAuthorizeCallback typo
- Fixed #90 : using VDD as analog reference

## 0.8.2

- Fixed burning bootloader issue with MacOS
- Added gpstest_swuart example sketch
- Added indicate API for BLECharacteristic
- Added custom_htm as usage example for indicate API.
- BLEClientCharacteristic
  - Added setIndicateCallback(), issue #113
  - Added useAdaCallback option to setNotifyCallback(), setIndicateCallback()
  - Change notify callback signature from 
notify_cb(BLEClientCharacteristic& chr, uint8_t* data, uint16_t len) to notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)

## 0.8.1

- Prevent sketch compiled with S132 v2 upload to device running S132 v5 and vice versa.

## 0.8.0

## Core

- Added IDE programmer and DFU option to upgrade bootloader from IDE.
- Added IDE option to choose either old & new bootloader (Softdevice 2.0.1 and 5.1.0)
- Added printBufferReverse() for Print class.
- Added Error String for easier debugging.
- Added analog oversampling #89 thanks to @Ureloc.

## BLE

- Upgrade Bluetooth 5 with Softdevice to S132 v5.0.0
- Support max ATT MTU up to 247 (configurable)
- Added SoftDevice configuration that affects SRAM used by SD. These function must be called before Bluefruit.begin()
  - configServiceChanged() add service changed characteristic
  - configUuid128Count() set the number of max uuid128 base
  - configAttrTableSize() set the size of buffer for GATT table
  - configPrphConn(), configPrphBandwidth() set the connection bandwidth setting for peripheral connections
  - configCentralConn(), configCentralBandwidth() set the connection bandwidth setting for central connections
- BLEAdvertising
  - Added multiple services aaddition API
  - Added addManufacturerData()
  - Added getInterval() to retrieve current active interval
  - Added Slow interval callback support via setSlowCallback()
- Added setPresentationFormatDescriptor() support
- Added addDescriptor() for BLECharacteritsic
- Added set/get appearance
- Added experimental (work in progress) BLE Homekit
- Enhanced bleuart to work with larger MTU
- Partially support data length extension
- Added BLECLientCharacteristic isValid() and read 8,16, 32 bit
- Added cental_custom_hrm example for how to use client service and characteristic
- BLECharacteristic
  - Added read8(), read16(), read32(), write8(), write16(), write32(), notify8(), notify16(), notify32()
  - Remove read(), write(), notify() with different uint8_t, uint16_t, uint32_t, int variant to prevent confusion
- Added discover(), discovered() to BLEClientCharacteristic
- Added getHandleRange() to BLEClientService

## Bug Fixs

- Fixed #92 PWM incorrect logic check, thanks to @burbankb 
- Fixed issue #108 with GPIO output when reset.
- Fixed an discovery bug introduced in 0.7.5
- Fixed #99 setStopCallback() for BLEScanner is not implemented
- Fixed #104 non-English keyboard BLE Hid, thanks to @ogatatsu
- Fixed weekday for client_cts example
- Fixed #110 Advertising.isRunning() returns true if stopped manually

## 0.7.5

### Core

- Change FreeRTOS to tickless mode
- Added systemOff() to enter System Off mode
- Added suspendLoop() to disable loop() task to further save power
- Correct waitForEvent() function with SoftDevice enabled and disabled.
- Added RotaryEncoder support for both Hardware and Software Encoder.
	- Software Encoder couldn't response fast enough due to gpio interrupt latency
- Ada Callback task is started along with loop task, used as worker thread to defer interrupt callback.

### BLE

- Fixed Characteristic Descriptor discovery issue with ancs
- Fixed memory issue with discovery characteristic

## 0.7.0

### Core

- Added printf with float format aka %f
- Added Servo library port
- Added Firmata library with BLEUART (AKA Nordic UART Service) support
	- New example via BLEUART at `Peripheral\StandardFirmataBLE`
- Enhanced `Adafruit_FIFO`: added `overwriteIfFull()`, `begin()`. Changed constructor signature.
- Added `Serial.setPins()` to remap Serial RX and TX pin location. **Must call before `Serial.begin()`!**
- Added `SoftwareSerial` support

### Bluefruit

- Renamed `Bluefruit.peerAddr()` to `Bluefruit.getPeerAddr()`
- Added connection handle to the connect and disconnect callback prototypes
- Change signature of Bluefruit.getName()
- Add printInfo() for configuration summary
- Change Bond Data layout to include paired Device Name. CCCD setting is also saved to the same file --> one file for each bond.

### BLEGap

- Added `getAddr()` and `setAddr()`

### BLE Service

- Added `BLEClientUart&` reference pointer to `BLEClientUart` RX callback
- Added new `BLEClientCts`class for client side Current Time Service
- Added `bufferTXD()` to `BLEUart` service to handle consecutive small `write()`calls
- Added `EddyStoneUrl` support
	- New example `Peripheral\eddystone_url`
- New hid example demonstrate how to implement an keyboard `Peripheral\hid_keyscan`

### BLEAdvertising

- Separated `BLEAdvertisingData` and `BLEAdvertising`
- Added `setStopCallback()` support to declare a callback when advertising stops
- Added the option to advertise for a specific time period. There are multiple
  timeouts: initial fast advertising mode, slow advertising mode, and an optional
  delay to stop advertising entirely. Values can be set in multiples of timeout
  ticks (0.625ms per unit) or in ms (approximate since it gets converted to 0.625ms
  units). The optional timeout to stop advertising entire is set via the
  `.start(timeout)` parameter.
- Blue LED will blink 2x faster in fast mode compared to slow mode.
- Default advertising timeout in slow mode = 30 seconds. Default adv interval in
  fast mode = 20 ms, slow mode = 152.5 ms
- Expanded `addUuid()`, `addService()` to take a list of UUID
- Added new example `advance_adv`

### BLEGAP (To support multiple connections, etc.)

- Added `getRole()` to distinguish between peripheral/central

### BLECentral

- Move scanner & report parser into new `BLEScanner` class
- New example showing how to scan for multiple peripherals with a specific
  advertising signature (`examples/Projects/rssi_proximity`).
- Added a new Dual Roles example `DualRoles\dual_bleuart`

## 0.6.5

### Core

- Fixed `setName()` issue
- Added SRAM usage to output when compiling
- Reduced stack size from 3KB to 2KB, and added functions `dbgStackUsed()` and `dbgStackTotal()` for dynamic stack usage
- Fixed #48 SPI & I2C warnings
- Changed board defines from `ARDUINO_FEATHER52` to `ARDUINO_NRF52_FEATHER` (`ARDUINO_FEATHER52` still exists for backward comaptiability)
- Changed pin in hwpwm and hid_camerashutter sketches
- Updated NFFS code to mynewt v1.0.0
- Added event recording support using Segger's Sysview

### BLE

- Updated for the latest Bluefruit LE Connect version with RGBW support
- Implemented Gatt `readCharByUuid()`
- Implemented Gap `getPeerName()`

### New Examples

- Added Low Level format sketch for filesystem: `libraries/nffs/examples/EraseNffs`

## 0.6.0

### Core

- Added a `HardwarePWM` class to support up to 12 PWM channels,
  compatible with Adafruit_Neopixel version 1.1.0 or higher
- Added waitForEvent() as alias for `__WFE()` instruction
- Changed FreeRTOS tick source from systick to RTC for low-power mode.
  `configTICK_RATE_HZ` changed to 1024. Upgraded port_cmsis_systick.c to
  SDK13 for bug fix.
- Enabled FreeRTOS's Idle hook, and call `waitForEvent()` in the Idle hook
  if `rtos_idle_callback()` is not defined
- Added `rtos_idle_callback()` as an optional callback to handle background
  tasks in user sketches
- Added a mutex to prevent UART conflicts
- Add a `SoftwareTimer` wrapper class for FreeRTOS's software timer
- Increased `configMINIMAL_STACK_SIZE` from 60 to 100, Increased
  `configTIMER_TASK_STACK_DEPTH` from 80 to 100

### BLE Library

- Add initial Central support and Gatt client service/characteristic classes
    - Added `BLEClientService`
    - Added `BLEClientCharacteristic`: support for long read/write, write
      with/without response.
    - Added `BLEDiscovery`
- Added `BLEGap` and `BLEGatt` to manage peripheral & central with Gatt client
  and server support
- BLE API changes
    - Added `connPaired()`, `requestPairing()`
    - Renamed `BLEBas.update()` to `.write()`
    - Changed Bluefruit `setConnInterval()`/`setConnIntervalMS()` return types
      from `err_t` to `bool`
    - Changed BLECentral `startScanning()`/`stopScanning()`/`connect()` return
      types from `err_t` to `bool`
    - Changed BLECharacteristic `notify()` return type from `err_t` to `bool`
    - Changed BLEHid `report()` function return type from `err_t` to `bool`
    - Changed BLEMid `send()`/`sendSplit()` return type from `err_t` to `bool`
- New BLE services
    - BLEAncs (Apple Notification Center Service)
    - BLEClientUart
    - BLEClientDis
- Added separate thread for callbacks to allow most API functions to be
  invoked directly inside the callback handler

### Tools

- `nrf5x-command-line-tools` is moved to tool dependency to fix windows 10 installation issue #28

### New Examples

- Hardware/
    - hwpwm
    - Fading
- Central/
    - central_bleuart
- Peripheral/
    - ancs
    - ancs_oled
    - hid_camerashutter

## 0.5.1

- Enhanced BLEUuid
- Fixed various typos

## 0.5.0

- Initial release
