# Adafruit nRF52 Arduino Core Changelog

## 0.7.0

### BLE

- Renamed `Bluefruit.peerAddr()` to `Bluefruit.getPeerAddr()`
- Changed BLEClientUart's RX callback
- Added connection handle to the connect and disconnect callback prototypes

### BLEAdvertising

- Separated `BLEAdvertisingData` and `BLEAdvertising`
- Added `setStopCallback()` support to declare a callback when advertising stops
- Add the option to advertise for a specific time. There are multiple timeouts: initial fast advertising mode, slow advertising mode, and an optional delay to stop advertising entirely. Values can be set in multiple of timeout ticks (0.625ms per unit) or in ms (approximate since it gets converted to 0.625ms units). The optional timeout to stop advertising entire is set via the `.stop(timeout)` parameter.
- Default timeout to slow mode = 30 seconds. Default adv intervals are fast mode = 20 ms, slow = 152.5 ms

### BLEGAP (To support multiple connections)

- Added `getRole()`

## 0.6.5

### Core

- Fixed `setName()` issue
- Add SRAM usage to output when compiling
- Reduced stack size from 3KB to 2KB, and added functions `dbgStackUsed()` and `dbgStackTotal()` for dynamic stack usage
- Fixed #48 SPI & I2C warnings
- Changed board defines from `ARDUINO_FEATHER52` to `ARDUINO_NRF52_FEATHER` (`ARDUINO_FEATHER52` still exists for backward comaptiability)
- Changed pin in hwpwm and hid_camerashutter sketches
- Updated nffs code to mynewt v1.0.0
- Added event recording support using Segger's Sysview

### BLE

- Updated for the latest Bluefruit LE Connect version with RGBW support
- Implemented Gatt `readCharByUuid()`
- Implemented Gap `getPeerName()`


### New Examples

- Add Low Level format sketch for filesystem: `libraries/nffs/examples/EraseNffs`

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
