# Adafruit nRF52 Arduino Core Changelog

## 0.9.0

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
