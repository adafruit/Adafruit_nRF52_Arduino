---
name: build
description: The Adafruit nRF52 core build contract. Use when a change must be verified by building, when an agent needs the boards and sketches a diff affects, when a HIL run or debug session needs firmware and its build record, or when a workflow asks for "the project's build". Resolves scope to boards, compiles the bundled libraries' sketches and the HIL sketches with arduino-cli against this checkout as the adafruit:nrf52 platform, reports pass/fail with the first error.
---

# Build contract

`scripts/check_build.py` is the entry point for verifying a change; agents and workflows call it rather than composing
`arduino-cli compile` lines. It is this core's profile for agentrc's `arduino` skill (`arduino_build.py`) in core
mode, loaded from the installed agentrc (`${AGENTRC_DIR:-$HOME/.claude}`), never fetched; CI fetches the commit
`test/hil/agentrc.json` pins. `.claude/skills/hil-contract/scripts/agentrc_pin.py check` names the one in use. That
skill's page ("A core") is the shared behaviour: the isolated arduino-cli environment that makes this checkout the
`adafruit:nrf52` platform whatever is installed or linked under `~/Arduino/hardware`, bundled libraries as the core's
own, submodule identity, the rig board's base image and the record. This page is what is this core's.

```bash
B=.claude/skills/build/scripts/check_build.py
python3 $B --scope <changed paths or dirs>          # boards and sketches a change affects
python3 $B --base origin/master                      # same, from the branch diff
python3 $B --base origin/master --worktree           # same, plus uncommitted and untracked changes
python3 $B --select-only (--scope ... | --base REF [--worktree] | --diff-file FILE) [--hil-config ROSTER]
python3 $B --board feather52840 [--board ...] [--example libraries/Bluefruit52Lib/examples/Peripheral/bleuart ...]
python3 $B --board nrf52840dk --shared --record build/hil/nrf52840dk.build.json --hil-config test/hil/ci.json
```

Submodules (`libraries/Adafruit_TinyUSB_Arduino`, `libraries/Adafruit_nRFCrypto`) must be initialised at their
gitlinks first: `git submodule update --init`. Dependencies are CI's install list (`EXTRA_DEPENDENCIES`), installed
beforehand in the user's Arduino libraries (exit 2 names the `arduino-cli lib install` line); `adafruit-nrfutil` must
be on `PATH` (`pip3 install adafruit-nrfutil`), since `platform.txt` packages every build as a DFU zip.

The last stdout line is JSON: `pass`; `firstError`, and `error` on exit 2 naming the cause or remedy; `boards[]` (one
row per board and sketch: `status` `ok`, `failed` or `error`, `fqbn`, `log`, `artifacts`, `platform`,
`foreignHeaders`, `libraries`, `removed`); `tests[]`; `paths[]` (each changed path's `rule` and `reason`);
`uncovered`; `dependencies`; `nothingToBuild`; `warnings`; `input` (HEAD, dirty, submodules); `narrowed`; `record`
when one was written. Exit 0 pass, 1 a build, test or record check failed, 2 usage or a missing prerequisite, 3 an
uncovered path; 2 outranks 1, and 1 outranks 3. Each build's log is `<buildDir>/check_build.log`.

`--select-only` prints the selection and stops before any compile. With `--hil-config <the host's roster>` it adds
`hil.{boards,full,uncovered}`: the roster boards the change puts under the HIL battery, and each family (`nrf52832`,
`nrf52833`, `nrf52840`) the change reaches with no roster board on this host.

## Reporting

An agent that reports `buildOk` and `notes` (agentrc's `code-writer`) maps the JSON this way: `buildOk` is `pass`. A
pass with no build or test run is "nothing to build": say so in `notes`. Exit 3 is `buildOk` false with each
`uncovered` reason quoted verbatim. Exit 1 or 2 quotes `firstError` (and `error`). Each `warnings` entry is quoted in
`notes`.

## Judgment

- **Selection is the script's**, first match per path; `paths[].reason` says why:
  - a changed file of a bundled library's sketch builds that sketch wherever it is eligible;
  - any `cores/nRF5/` file builds every representative, FreeRTOS included; a SEGGER SystemView file
    (`cores/nRF5/sysview/`) also builds `SYSVIEW_SKETCH` on `feather52840_sysview`, under `debug=l3`, the only menu
    that compiles SystemView (`CFG_SYSVIEW=1`) and no CI build; `#if CFG_SYSVIEW` code elsewhere gets no such build;
    a linker script (`cores/nRF5/linker/<ld>`) every `boards.txt` board linking with it;
  - `variants/<v>/` every `boards.txt` board on that variant, CI's or not; `boards.txt` and `platform.txt` CI's
    whole matrix, the nRF52833 representative and the rig board;
  - the TinyUSB gitlink, `Adafruit_LittleFS`, `InternalFileSytem`, SPI and Wire (`SHARED_LIBRARIES`) every
    representative; another bundled library (Bluefruit52Lib and nRFCrypto included) its own sketches on the
    representatives;
  - the rig board's base image files (`BOARDS['nrf52840dk']['base']`) build the rig board's HIL record; any other
    `bootloader/` file, `tools/adafruit-nrfutil/`, `tools/uf2conv/`, `tools/pynrfbintool/`,
    `tools/update_bootloader.py`, `tools/midi_tests/`, `scripts/` and `programmers.txt` are uncovered (`UNCOVERED`):
    nothing here builds, flashes or uses them; anything unmatched is uncovered too;
  - `tools/build_all.py`, `tools/makeboards.py` and the CI workflow run the contract tests, which hold the board
    table to CI's matrix and `build_all.py`'s SoftDevice choice, and `boards.txt` to `makeboards.py`'s output.
- **Boards** (`BOARDS`): CI's matrix with `tools/build_all.py`'s FQBNs
  (`adafruit:nrf52:<id>:softdevice=<s132v6|s140v6>,debug=l0`), its skip markers and its own skip (the bundled
  TinyUSB's examples, which its own CI builds), plus the bundled nRFCrypto's examples off nRF52840 (no CC310 in
  the MDK; its markers miss `pca10100`); every other `boards.txt` board, built only when its variant or linker
  script changes; `pca10100` (the nRF52833 DK, `softdevice=s140v7`), the nRF52833 representative with no CI board;
  the rig board `nrf52840dk`, boards.txt's `pca10056`; and `feather52840_sysview`, feather52840 at `debug=l3`,
  where only `SYSVIEW_SKETCH` (blinky) is eligible, built for a SystemView change or a change to that sketch.
  Representatives: `feather52832` (nRF52832, no USB), `pca10100` (nRF52833), `feather52840` and `nrf52840dk`
  (nRF52840).
- **Sketches** are every `.ino` dir under `libraries/`, as CI scans them. The contract tests pin the ones no CI board
  builds (`unbuildable_examples()`). A sketch that cannot build on a board carries that board's `.<id>.test.skip`
  marker (`adc_vbat` on the boards whose variant has no `PIN_VBAT`).
- **A HIL record** (`--board nrf52840dk --shared --record FILE --hil-config ROSTER`, by convention
  `build/hil/nrf52840dk.build.json`) is the HIL contract's build receipt. It carries `core_cdc` (the restoration
  image) and `core_rtos`, each linked into the application region `[0x26000, 0xED000)` of `nrf52840_s140_v6.ld`, and
  binds the base image: boards.txt's bootburn HEX (MBR, S140 6.1.1, bootloader 0.9.1) and
  `test/hil/nrf52840dk_boot_words.hex`, the words the boot path reads that the bootburn HEX leaves unprogrammed: the
  MBR's bootloader and parameter page words (0xFF8 and 0xFFC) and its parameter page at 0xFE000 erased, as the
  bootburn HEX leaves them, and the bootloader settings' `bank_0` (0xFF000 = 1: application valid, CRC check
  skipped), without which the bootloader stays in DFU mode over a sketch flashed through J-Link, and `bank_1`
  (0xFF004 = 0xFF, as an application DFU leaves it; 0xAA would resume an update). Their UICR records are
  excluded (`base_exclude`): the base check never reads or writes UICR. Each sketch's VID:PID is the
  `-DUSB_VID`/`-DUSB_PID` `platform.txt` passes to every compile.
- **Private dirs for parallel agents.** Each build uses `build/agent-<pid>/<board>/<sketch path>`, removed when it
  passes; `--shared` builds into `build/hil/<board>/<sketch>` under `build/hil/.lock` and keeps it for flashing.
- **Printing a command is not verification.** A verdict is the JSON of a run that happened; an agent that cannot run
  the script reports the build as not run.
