# Adafruit nRF52 Arduino Core Changelog

## 0.9.1

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
