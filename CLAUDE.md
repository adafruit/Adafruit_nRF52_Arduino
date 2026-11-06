# Adafruit nRF52 Core Agent Instructions

## Working Rules

- An Arduino core (the `adafruit:nrf52` board package) with no standalone build: sketches compile against this
  checkout as that platform through the build contract's isolated arduino-cli environment, never the installed board
  package or a `~/Arduino/hardware` link to another checkout. CI's board matrix is in `.github/workflows/githubci.yml`,
  its FQBNs and skip rules in `tools/build_all.py`. Initialise the submodules (`git submodule update --init`) before
  any build.
- Development boards: `feather52840` (nRF52840) by default, `feather52832` (nRF52832, no USB) and `pca10100`
  (nRF52833); on hardware, the ci.lan farm's `nrf52840dk` (boards.txt's `pca10056`).
- Code references — runtime: `cores/nRF5/` (`main.cpp` starts FreeRTOS with the loop and callback tasks, `freertos/`,
  `nordic/nrfx/` HAL, `nordic/softdevice/` S132/S140 headers, `linker/` per chip and SoftDevice, `sysview/` under
  `debug=l3`, `wiring*` on nrfx); boards: `variants/<variant>/` (`variant.*`), `boards.txt`, `platform.txt`; bundled
  libraries: `libraries/` (Bluefruit52Lib, InternalFileSytem on Adafruit_LittleFS, SPI, Wire, PDM;
  `Adafruit_TinyUSB_Arduino` and `Adafruit_nRFCrypto` as submodules); `bootloader/`: prebuilt bootloaders with their
  SoftDevices per variant, of which only the DK's is used here (the HIL base image).

## Code Rules

- `boards.txt` is generated: edit `tools/makeboards.py` and regenerate (`python3 tools/makeboards.py > boards.txt`);
  a contract test fails when they differ. A board's `build.variant` must name a `variants/` directory, and its
  SoftDevice is per chip: `s132v6` (nRF52832), `s140v7` (nRF52833), `s140v6` (nRF52840).
- Compiler flags include `-Werror=return-type -Wall -Wextra`; C is `gnu11`, C++ `gnu++11` with `-fno-rtti
  -fno-exceptions`.
- Blocking Arduino calls run on FreeRTOS (`delay()` is `vTaskDelay`, `millis()` the tick at 1024 Hz): task priorities
  run `TASK_PRIO_LOWEST` (0) to `TASK_PRIO_HIGHEST` (4), the loop at `TASK_PRIO_LOW`. While the SoftDevice is
  enabled, its peripherals and interrupt priorities are reserved: go through `sd_*` calls.
- `initVariant()` is weak: a board overrides it in `variant.cpp`.
- Check clock, radio, flash, UICR and peripheral behaviour against the nRF52832/52833/52840 product specification,
  errata and the SoftDevice specification (agentrc's `read-doc`) before changing or reviewing code that depends on
  it.
- Fix the submodules' code upstream (adafruit/Adafruit_TinyUSB_Arduino, adafruit/Adafruit_nRFCrypto) and bump the
  gitlink; never edit it here.
- `.pre-commit-config.yaml` is authoritative: the contract tests, and clang-format v15 on `test/hil/` only (the core's
  existing code is not reformatted).
- Keep each file's line endings.

## Skills

- Agents, skills and workflows beyond this repo's `.claude/` come from [agentrc](https://github.com/hathach/agentrc);
  install it before agent work on this core. Its `arduino` skill's core mode and base images hold the mechanics both
  contracts below run, shared with the Adafruit SAMD core, Adafruit TinyUSB Arduino and Adafruit SPIFlash.
- Skill script tests live in `.claude/test/test_*.py`; pre-commit runs them against the installed agentrc
  (`${AGENTRC_DIR:-$HOME/.claude}`); CI fetches the commit `test/hil/agentrc.json` pins instead.
- Hardware diagnosis and repair use agentrc's `hw-debugger`; independent hardware claims and committed-fix validation
  use `hw-validator`; routine HIL runs use `hil-operator` — all orchestrated by `chief`.
- Core audit: the `code-audit` workflow with `dirs` (e.g. `cores/nRF5`, `cores/nRF5/freertos`,
  `libraries/Bluefruit52Lib/src`) and `dimensions`: `RTOS and timing: tasks, priorities, the tick, delay and
  millis, ISR-to-task handoff`; `SoftDevice and BLE: sd_* calls, event dispatch, reserved peripherals and
  priorities`; `flash and storage: InternalFileSystem, LittleFS, flash operations under the SoftDevice`; `USB and
  serial: the TinyUSB integration and its task, CDC, UARTE`; `style: repo conventions and the clang-format scope`.

## Build and Validate

- Build contract: `.claude/skills/build/SKILL.md`. It resolves a change to boards and sketches and builds them with
  arduino-cli against this checkout; `--shared --record` writes the HIL build record and keeps the images HIL flashes.
- HIL contract: `.claude/skills/hil-contract/SKILL.md`. It owns taking and releasing a rig board, names the roster for
  the host you are on, runs the battery on a dev PC's board or the ci.lan farm, and gives hw-validator and
  hw-debugger their manual sessions. Manual runs only: no CI job runs it. Hardware validation comes from its report,
  never from a build; the farm has no nRF52832 or nRF52833 board and no BLE peer, so a change reaching those chips or
  the radio is reported pending on hardware.
- Before submitting, and again after a rebase: `pre-commit run --all-files`.
- For code changes: `python3 .claude/skills/build/scripts/check_build.py --base origin/master --worktree`.

## PRs and Follow-ups

- Before opening or updating a PR, follow Build and Validate; CI does not run HIL.
- A non-draft PR opened on the human's request, unless they declined babysitting, or their request or yes to babysit
  one, authorizes one headless `chief` to drive its reviews and CI to green: push commits to its branch, reply to and
  resolve its review threads through `pr-babysit` with `autoPush: true` and `markSonar: true`. Launch it in the PR
  worktree through agentrc's `headless-chief` skill (`chief_run.py`), never a bare `claude -p`, and relay its exit as
  that skill directs. Merging needs the human's own words asking to merge that PR. `pr-babysit` args:
  `{"pr": <num>, "protected": "^test/hil/[^/]+\\.(json|hex)$"}`; reviewers are agentrc's default (`protected` keeps the
  HIL rosters, the agentrc pin and the base image's settings out of automated fixes).
- This repository has no follow-up label: label a follow-up issue with an applicable existing label, or leave it
  unlabeled.
