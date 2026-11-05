---
name: hil-contract
description: The Adafruit nRF52 core HIL contract, under agentrc's `hil` rules. Use when running the core's HIL battery on rig boards (the nRF52840 DK on the ci.lan farm, or a dev PC's), when a HIL run fails, hangs or reports a board locked, when reading its report, or for a manual session (flash, GDB, console, a check, a command) on a rig board from hw-validator or hw-debugger.
---

# HIL contract

Read agentrc's `hil` skill first; this page supplies its contract sections. The mechanics are agentrc's `arduino`
skill (`arduino_hil.py`, `arduino_remote.py`), shared with the Adafruit SAMD core, Adafruit TinyUSB Arduino and
Adafruit SPIFlash, whose HIL contracts have the same commands, budgets, session rules and recovery; this page names
what is this core's. Manual runs only: no CI job runs this battery. Scripts, all in `scripts/`, each with its usage in
its docstring:

- `hil_test.py`: the harness on the host with the boards (this core's profile and tests).
- `hil_remote.py`: drives the harness on ci.lan from the dev PC that built the firmware.
- `agentrc_pin.py`: which agentrc both load their shared scripts from: the installed one
  (`${AGENTRC_DIR:-$HOME/.claude}`) on a dev PC, never fetched; in CI the commit `test/hil/agentrc.json` pins; on the
  rig, the copy `hil_remote.py` staged. It refuses an agentrc without core mode or base images.

```bash
H=.claude/skills/hil-contract/scripts
python3 $H/agentrc_pin.py check            # the agentrc a run would load, validated; its root
```

## Hosts/rosters

| Host     | How to tell                        | Roster                                                     |
|----------|------------------------------------|------------------------------------------------------------|
| ci.lan   | the HIL farm; reached over ssh     | `test/hil/ci.json` (tracked)                               |
| a dev PC | a board on this PC's USB           | `test/hil/local.json` (ignored; copy `local.example.json`) |

ci.lan has no arduino-cli and no checkout of this core: firmware is built on the dev PC and staged by
`hil_remote.py`. On a dev PC without `test/hil/local.json`, ask the user for one; an agent that cannot ask returns
each requested board not-run naming the missing file.

A roster entry is the `arduino` skill's (`name`, the board's name in agentrc's board registry and its lock; `uid`, the
DUT's USB serial; `usb_port`; `probe`, its debug probe's serial, whose registry entry supplies the flasher, or a dev
PC's unregistered board's own `flasher`, which must be J-Link: the base image is checked and written through it;
`debug`) plus `arduino.tests`: `["cdc", "rtos"]`. A board's name must be a rig board of the build profile
(`nrf52840dk`). The DUT's serial under this core is the FICR DEVICEID in TinyUSB Arduino's byte order
(`Adafruit_TinyUSB_nrf.cpp`), not tinyusb's: the farm DK's is `2467EB236E0E4F2D` here and `2D4F0E6E23EB6724` on
tinyusb's roster.

ci.lan's `nrf52840dk` is also on tinyusb's roster, whose firmware overwrites the MBR and SoftDevice this core's
sketches run on: every flash here checks the base image first and rewrites it when it differs (Build). This contract
performs no rig-wide operation. **Farm check**: tinyusb's launcher,
`ssh ci.lan 'cd ~/code/tinyusb && python3 test/hil/helper/hil_farm_check.py -b <board>...'`.

## Selection

A fixed battery per board, the roster's `arduino.tests`, each over the DUT's USB CDC (TinyUSB, served by the core's
USB task) with a reply line carrying its command's nonce (a missing, stale or other reply fails the test at its
deadline):

- `cdc` (`test/hil/core_cdc/`, also the restoration check): `INFO` must report `stack=tinyusb` and the board's chip
  family; `ECHO` of 1, 63, 64, 65 and 1000 random bytes must return every byte as sent, then a `PASS` line with their
  length and crc32.
- `rtos` (`test/hil/core_rtos/`): `millis()` read twice 3 s apart must lie within the host's monotonic bracket of
  the two exchanges widened by 1%, `micros()` must agree with `millis()` within 2 ms, and `delay(1000)` measured by
  `micros()` must take 1000 to 1002 ms, widened by 1%; then two FreeRTOS worker tasks each queue a message every
  10 ms (10 ticks of 1/1024 s) and a `SoftwareTimer` ticks every 50 ms (51 ticks) while `loop()` drains the queue for
  1 s: each worker's count must lie within its tick-rounded period's bound and the timer's within 1.5 ticks of
  expected (`rtos_problem`). A coarse functional bound on the FreeRTOS tick (RTC1 on the 32.768 kHz clock), no clock
  accuracy claim. Neither sketch starts the SoftDevice or mounts InternalFileSystem.

`-b` narrows the boards. The PR-scoped selector is the build contract's:

```bash
python3 .claude/skills/build/scripts/check_build.py --select-only --base origin/master --hil-config test/hil/ci.json
```

Its `hil.boards` are the roster boards to run, `hil.full` means the harness, a HIL sketch or a roster changed (every
board), and `hil.uncovered` names each family the change reaches with no board on that roster (on the farm,
`nrf52832` and `nrf52833`): report those uncovered, never pass.

## Build

Each run board needs a clean HIL record at HEAD (build skill, "A HIL record"), with the submodules at their gitlinks:

```bash
python3 .claude/skills/build/scripts/check_build.py --board <board> --shared --record build/hil/<board>.build.json \
  --hil-config <roster>
```

The harness admits a record only as the `arduino` skill's "The HIL build record" entry says, the base image's files
included. A manual flash or GDB session also takes an experimental record (`--experimental`).

The record's base image is what runs a sketch on the DK: boards.txt's bootburn HEX (MBR at 0x0, S140 6.1.1 at 0x1000,
bootloader 0.9.1 at 0xF4000 and its data page at 0xFD800), the MBR's bootloader and parameter page words at 0xFF8
and its parameter page at 0xFE000 erased, and the bootloader settings that mark the application valid with no CRC
check and no update pending (0xFF000). Before every flash (battery, commissioning, manual flash and restore) the
harness verifies each of those runs over J-Link and rewrites them only when a `verifybin` proves one differs; the
row's `base` names `matched` or the run that differed. UICR is excluded: never read, compared or written, so the
board's UICR (bootloader and MBR settings addresses, APPROTECT, REGOUT0) is whatever its last full programming left.
The base HEX itself sets the bootloader address only there (`NRFFW[0]`), and the MBR reads its own flash word at
0xFF8 first, so the base holds that word erased: with UICR erased too the MBR boots the SoftDevice and the sketch
directly, skipping the bootloader (S140 SoftDevice Specification §12.3).

## Execution

Budgets, the remote run and the manual session commands are Adafruit TinyUSB Arduino's HIL contract's, with this
core's names: the attempt dir is `~/hil-remote/adafruit-nrf52/<id>`, `check --test` takes `cdc` (default) or `rtos`,
`session close` restores `core_cdc`.

```bash
python3 $H/hil_remote.py run --run-id <id> [-b BOARD]...
python3 $H/hil_remote.py wait <id> --timeout 570    # repeat while it exits 3
python3 $H/hil_test.py report .hil-remote/<id>/hil-run
python3 $H/hil_remote.py session open --run-id <id> -b <board>     # then session exec/stage/close, as TinyUSB's
```

On the dev PC with the board: `python3 $H/hil_test.py run [-b BOARD]... test/hil/local.json`.

## Reservation

As the `arduino` skill's Locks: the board lock `/tmp/tinyusb-hil-locks/<name>.lock` and the pending kind
`arduino-hil` are shared with tinyusb, tinyuf2, the Adafruit SAMD core, Adafruit TinyUSB Arduino and Adafruit
SPIFlash. `hil_test.py run` and `commission` lock each board themselves; a manual session holds it under a reason
unique to its attempt, `arduino-hil session <attempt dir> <token>`. Cleanup state is `core_cdc` flashed and checked
over the base image, every child proven ended, and the hold released.

## Results

`hil_test.py run` writes `hil_report.json` (schema `adafruit-nrf52-core-hil`) with the same rows, exits, verdicts and
delegated output (`hil_test.py report`) as Adafruit TinyUSB Arduino's contract, each test row adding `base`. The
caller retries a fail once under a new run id, per the `hil` Caller rules.

## Recovery

Adafruit TinyUSB Arduino's Recovery, run in this core's attempt dir with its roster and records. The battery writes
the base image's runs (outside the application span, never UICR) and the application span `[0x26000, 0xED000)`,
never InternalFileSystem at 0xED000, and keeps no state across sketches, so restoring `core_cdc` over a checked base
is the whole cleanup. A base rewrite that failed leaves the base unknown: the restore checks and rewrites it again.

## Limits

- **Coverage:** the nRF52840 (the DK, crystal 32.768 kHz clock): USB CDC enumeration and bulk data integrity through
  TinyUSB and the core's USB task, the FreeRTOS tick against the host, task scheduling, queues and software timers, and
  boot through the MBR and SoftDevice (and the bootloader while UICR points to it). Not covered on hardware: nRF52832
  and nRF52833 boards (no farm board; they build only), the SoftDevice's radio and every BLE path (no peer: ci.lan has
  no Bluetooth adapter, and `Bluefruit.begin()` mounts and may format InternalFileSystem), InternalFileSystem and
  LittleFS, nRFCrypto, PDM, I2S, the UARTE, TWIM and SPIM drivers (Serial1, Wire, SPI) and every other peripheral (no
  fixture wired), the bootloader's DFU and UF2 paths, UICR contents (so whether the bootloader is in the boot path at
  all), power and low-power modes, `debug=l3` SystemView builds, and every other variant.
- **Commissioning:** ci.lan's `nrf52840dk` (J-Link `683944004`, uid `2467EB236E0E4F2D`, `239a:80da`) ran the battery
  on 2026-11-05 (`nrf52-core-hil-1`): the first flash found tinyusb's firmware at 0x0 and rewrote the base, `cdc` and
  `rtos` passed, the next flash found the base matched, and `core_cdc` was restored.
