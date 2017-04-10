## 0.5.5

### Core

- Add HardwarePWM class to support up to 12 channels and compatible with Neopixel library
- Add waitForEvent() as alias to __WFE() instruction
- Enable FreeRTOS's Idle hook, add rtos_idle_callback() as optional callback for user
- Add mutex to prevent uart conflict
- Change FreeRTOS tick source from systick to RTC for low-power mode

#### New Example

- Hardware/hwpwm
- Hardware/Fading

### BLE Library

- Initial Central support
  -  Add BLECentralService
  -  Add BLECentralCharacteristic
  -  Add BLEDiscovery
- Add BLEGap and BLEGatt to manage peripheral & central with Gatt client and server support
- BLE API changes
  - Add connPaired(), requestPairing()
  - Rename BLEBas .update() to .write()
  - Change Bluefruit setConnInterval()/setConnIntervalMS() return type from `err_t` to `bool`
  - Change BLECentral startScanning()/stopScanning()/connect() return type from `err_t` to `bool`
  - Change BLECharacteristic notify() return type from `err_t` to `bool`
  - Change BLEHid report function return type from `err_t` to `bool`
  - Change BLEMid send/sendSplit return type from `err_t` to `bool`
- New BLE Serivce
  - Apple Notification Center Service (ANCS)
  - Central BLE UART
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
