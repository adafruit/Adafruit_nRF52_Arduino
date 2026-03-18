# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

Adafruit nRF52 Arduino Board Support Package (BSP) — an Arduino core for Nordic nRF52 Bluetooth SoCs (nRF52832, nRF52833, nRF52840). Targets Adafruit Bluefruit boards. Combines the Arduino API with FreeRTOS, Nordic nrfx HAL, Nordic SoftDevice (BLE stack), and TinyUSB.

**Default/preferred board:** `feather52840` (Adafruit Feather nRF52840 Express). Use this board when no specific board is specified.

## Build Commands

This is an Arduino BSP, not a standalone project. It builds via `arduino-cli`.

**Build a single example:**
```bash
arduino-cli compile --warnings all --fqbn adafruit:nrf52:feather52840:softdevice=s140v6,debug=l0 path/to/sketch
```

**Build all examples for a board (CI script):**
```bash
python3 tools/build_all.py feather52840
```

**Build all examples for all default boards:**
```bash
python3 tools/build_all.py
```

**Default CI boards:** `cluenrf52840`, `cplaynrf52840`, `feather52832`, `feather52840`, `feather52840sense`, `itsybitsy52840`

**FQBN format:** `adafruit:nrf52:{variant}:softdevice={sd},debug=l0`
- SoftDevice selection: `s132v6` for nRF52832, `s140v7` for nRF52833, `s140v6` for nRF52840

**Skip markers for examples:** Place these files in an example directory to control CI:
- `.all.test.skip` — skip for all boards
- `.{variant}.test.skip` — skip for a specific board
- `.{variant}.test.only` — run only for that board

## Architecture

### Core (`cores/nRF5/`)
Arduino core implementation. Entry point is `main.cpp` which initializes hardware, creates the main FreeRTOS task (`loop_task`), calls `ada_callback_init(...)` to set up the callback task (`adafruit_callback_task` in `cores/nRF5/utility/AdaCallback.c`), and starts the scheduler. Standard Arduino wiring functions (`wiring_digital.c`, `wiring_analog_nRF52.c`, etc.) wrap Nordic nrfx HAL calls.

Key subsystems:
- **FreeRTOS** (`freertos/`) — all blocking Arduino functions use RTOS primitives internally
- **Nordic nrfx** (`nordic/nrfx/`) — hardware abstraction for peripherals (GPIO, UART, SPI, I2C, ADC, PWM, etc.)
- **SoftDevice API** (`nordic/softdevice/`) — Nordic BLE stack headers (S132 for 52832, S140 for 52840/52833)
- **Linker scripts** (`linker/`) — memory layout per chip, accounting for SoftDevice and bootloader regions

### Variants (`variants/`)
Per-board pin definitions. Each variant has `variant.h` (pin macros, peripheral assignments) and `variant.cpp` (pin mapping array `g_ADigitalPinMap[]`, peripheral init). When adding a new board, create a new variant directory and add entries to `boards.txt`.

### Libraries (`libraries/`)
- **Bluefruit52Lib** — BLE central/peripheral, services (UART, HID, MIDI, Battery, etc.), bonding
- **Adafruit_TinyUSB_Arduino** — USB device stack (CDC, HID, MSC, MIDI) — git submodule
- **Adafruit_LittleFS / InternalFileSytem** — flash filesystem
- **Adafruit_nRFCrypto** — CryptoCell CC310 wrapper (hash, HMAC, ECC) — git submodule
- **Wire, SPI, Servo, PDM, SoftwareSerial, RotaryEncoder** — peripheral drivers

### Bootloader (`bootloader/`)
Precompiled UF2/hex/zip binaries per variant. Upload via UF2 drag-drop, serial DFU (`adafruit-nrfutil`), or J-Link (`nrfjprog`).

### Build Configuration
- `platform.txt` — compiler flags, recipes, tool paths. Compiler: `arm-none-eabi-gcc`, optimization `-Ofast`, hard float ABI.
- `boards.txt` — board definitions (MCU, SoftDevice, memory sizes, USB VID/PID, debug options)
- `tools/build_all.py` — CI build orchestrator using `arduino-cli compile` with multiprocessing

### Submodules
Some libraries (`Adafruit_TinyUSB_Arduino`, `Adafruit_nRFCrypto`) are git submodules. After cloning:
```bash
git submodule update --init
```

## Key Conventions

- Compiler flags include `-Werror=return-type -Wall -Wextra` — all functions must have return statements
- C standard: `gnu11`, C++ standard: `gnu++11` with `-fno-rtti -fno-exceptions`
- In `boards.txt`, each `*.build.variant` value must match a directory name under `variants/` (board IDs like `feather52840` may differ from the variant directory, e.g. `variants/feather_nrf52840_express`)
- FreeRTOS task priorities: `TASK_PRIO_LOWEST` (0) through `TASK_PRIO_HIGHEST` (4)
- `initVariant()` is a weak symbol — boards override it in `variant.cpp` for board-specific init
