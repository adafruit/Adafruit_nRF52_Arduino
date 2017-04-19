# Adafruit nRF52 Arduino Core Changelog

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

#### New Examples

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
