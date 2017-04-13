## 0.6.0

### Core

- Added a `HardwarePWM` class to support up to 12 PWM channels, compatible with Neopixel library
- Added waitForEvent() as alias for `__WFE()` instruction
- Changed FreeRTOS tick source from systick to RTC for low-power mode. `configTICK_RATE_HZ` changed to 1024. Upgraded port_cmsis_systick.c to SDK13 for bug fix.
- Enabled FreeRTOS's Idle hook, and call `waitForEvent()` in the Idle hook if `rtos_idle_callback()` is not defined
- Added `rtos_idle_callback()` as an optional callback to handle background tasks in user sketches
- Added a mutex to prevent UART conflicts
- Add a `SoftwareTimer` wrapper class for FreeRTOS's software timer
- Increased `configMINIMAL_STACK_SIZE` from 60 to 100, Increased `configTIMER_TASK_STACK_DEPTH` from 80 to 100

#### New Examples

- Hardware/hwpwm
- Hardware/Fading

### BLE Library

- Add initial Central support and Gatt client service/characteristic classes
  - Added `BLEClientService`
  - Added `BLEClientCharacteristic`: support for long read/write, write with/without response.
  - Added `BLEDiscovery`
- Added `BLEGap` and `BLEGatt` to manage peripheral & central with Gatt client and server support
- BLE API changes
  - Added `connPaired()`, `requestPairing()`
  - Renamed `BLEBas.update()` to `.write()`
  - Changed Bluefruit `setConnInterval()`/`setConnIntervalMS()` return types from `err_t` to `bool`
  - Changed BLECentral `startScanning()`/`stopScanning()`/`connect()` return types from `err_t` to `bool`
  - Changed BLECharacteristic `notify()` return type from `err_t` to `bool`
  - Changed BLEHid `report()` function return type from `err_t` to `bool`
  - Changed BLEMid `send()`/`sendSplit()` return type from `err_t` to `bool`
- New BLE Services
  - Apple Notification Center Service (ANCS)
  - BLEClientUart
  - BLEClientDis
- Add separated Thread for callbacks to allow most functions API() to be invoked directly in the callback

#### New Examples

- Central
  - central_bleuart
- Peripheral
  - ancs
  - hid_camerashutter

## 0.5.1

- Enhance BLEUuid
- fix typos

## 0.5.0

Initial Release
