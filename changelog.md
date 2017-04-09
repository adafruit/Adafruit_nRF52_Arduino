## 0.5.5

### Core

- Add HardwarePWM class to support up to 12 channels and compatible with Neopixel library
- Enable FreeRTOS's Idle hook to call waitForEvent()
- Add mutex to prevent uart conflict

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
  - connPaired(), requestPairing()
  - rename BLEBas .update() to .write()
  - change Bluefruit setConnInterval()/setConnIntervalMS() return type from `err_t` to `bool`
  - change BLECentral startScanning()/stopScanning()/connect() return type from `err_t` to `bool`
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
