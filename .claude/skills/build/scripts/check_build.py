#!/usr/bin/env python3
"""Build this core's bundled-library and HIL sketches with arduino-cli: the project's build contract (SKILL.md).

  check_build.py (--scope PATH... | --base REF [--worktree] | --diff-file FILE) [--example E]... [-j N]
  check_build.py --board B... [--example E]... [--shared] [--build-property K=V]...
  check_build.py --board B --shared --record FILE --hil-config ROSTER [--experimental --build-property K=V...]
  check_build.py --select-only (--scope ... | --base REF [--worktree] | --diff-file FILE) [--hil-config ROSTER]

stdout ends with one JSON line. Exit 0 pass, 1 a build or test failed, 2 usage or a missing
prerequisite ("error" names the remedy), 3 an uncovered path. 2 outranks 1, 1 outranks 3.

This file is the core's profile for agentrc's arduino_build.py in core mode (the `arduino` skill), loaded from the
agentrc test/hil/agentrc.json pins; the mechanics are there.
"""

import functools
import json
import re
import shlex
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402

LIBRARY_NAME = 'Adafruit nRF52 Boards'   # platform.txt name=
PLATFORM = 'adafruit:nrf52'
SKETCH_ROOTS = ('libraries',)
FAMILY_FLAGS = {'-DNRF52832_XXAA': 'nrf52832', '-DNRF52833_XXAA': 'nrf52833', '-DNRF52840_XXAA': 'nrf52840'}
# the SoftDevice each family's boards build with (tools/build_all.py get_sd(), boards.txt softdevice menus)
SOFTDEVICE = {'nrf52832': 's132v6', 'nrf52833': 's140v7', 'nrf52840': 's140v6'}


@functools.cache
def boards_txt():
    """{boards.txt id: {variant, family, ldscript}}, or {} where boards.txt is absent (a tree staged on a rig)."""
    path = ROOT / 'boards.txt'
    if not path.is_file():
        return {}
    out = {}
    for m in re.finditer(r'^(\w+)\.build\.(variant|extra_flags|ldscript)=(.*)$', path.read_text(), re.M):
        out.setdefault(m.group(1), {})[m.group(2)] = m.group(3).strip()
    return {i: {'variant': v['variant'], 'ldscript': v.get('ldscript', ''),
                'family': next((f for flag, f in FAMILY_FLAGS.items() if flag in v.get('extra_flags', '')), '')}
            for i, v in out.items() if 'variant' in v}


def board_fqbn(board_id, family, debug='l0'):
    return f'adafruit:nrf52:{board_id}:softdevice={SOFTDEVICE[family]},debug={debug}'   # tools/build_all.py's


# CI's matrix (.github/workflows/githubci.yml); a board's name is its boards.txt id, which its .<id>.test.skip and
# .<id>.test.only markers use
CI_BOARDS = ('cluenrf52840', 'cplaynrf52840', 'feather52832', 'feather52840', 'feather52840sense',
             'feather_nrf52840_sense_tft', 'itsybitsy52840', 'ledglasses_nrf52840')
CI_FAMILY = {b: 'nrf52832' if b == 'feather52832' else 'nrf52840' for b in CI_BOARDS}   # boards.txt's, offline
BOARDS = {b: {'fqbn': board_fqbn(b, f), 'family': f, 'filters': b} for b, f in CI_FAMILY.items()}
# the rig board, named as in agentrc's board registry: the nRF52840 DK as boards.txt's pca10056. `span` is
# nrf52840_s140_v6.ld's application region; `base` the MBR, SoftDevice, bootloader, the MBR's erased flash words and
# the bootloader's settings word under it, UICR excluded (the HIL contract's Build)
BOARDS['nrf52840dk'] = {'fqbn': board_fqbn('pca10056', 'nrf52840'), 'family': 'nrf52840', 'filters': 'pca10056',
                        'span': (0x26000, 0xED000),
                        'base': ('bootloader/pca10056/pca10056_bootloader-0.9.1_s140_6.1.1.hex',
                                 'test/hil/nrf52840dk_boot_words.hex'),
                        'base_exclude': ((0x10001000, 0x10002000),)}
# nRF52833 has no CI board: the DK's id builds as its family's representative
BOARDS['pca10100'] = {'fqbn': board_fqbn('pca10100', 'nrf52833'), 'family': 'nrf52833', 'filters': 'pca10100'}
# SystemView compiles only under debug=l3 (boards.txt's CFG_SYSVIEW=1), which no CI build selects: a change to it also
# builds SYSVIEW_SKETCH, the only sketch eligible on this board
SYSVIEW_BOARD = 'feather52840_sysview'
BOARDS[SYSVIEW_BOARD] = {'fqbn': board_fqbn('feather52840', 'nrf52840', 'l3'), 'family': 'nrf52840',
                         'filters': 'feather52840'}
SYSVIEW_SKETCH = 'libraries/Bluefruit52Lib/examples/Hardware/blinky'
REPRESENTATIVES = {'nrf52832': ['feather52832'], 'nrf52833': ['pca10100'], 'nrf52840': ['feather52840', 'nrf52840dk']}
# the sketches a HIL record carries, the first also the restoration image: it writes no flash
HIL_SKETCHES = {'device': ('test/hil/core_cdc', 'test/hil/core_rtos')}
CONTRACT_TESTS = {'build': '.claude/test/test_build.py', 'hil': '.claude/test/test_hil.py',
                  'hil_remote': '.claude/test/test_hil_remote.py', 'agentrc_pin': '.claude/test/test_agentrc_pin.py'}
SMOKE_BOARDS = ('nrf52840dk',)
NOTHING = ('.claude/', '.github/', '.agents', 'AGENTS.md', 'CLAUDE.md', 'README.md', 'changelog.md',
           'CODE_OF_CONDUCT.md', 'LICENSE', 'keywords.txt', '.gitignore', '.gitmodules', '.pre-commit-config.yaml')
# outside both contracts: no build here makes or flashes them, so a change to one is reported uncovered
UNCOVERED = {'bootloader/': 'prebuilt bootloaders: no build here makes them, and HIL flashes only the rig board\'s '
                            'base image',
             'tools/adafruit-nrfutil/': 'the DFU upload tool (Windows and macOS builds): no build or HIL run uses it',
             'tools/uf2conv/': 'UF2 packaging: no build or HIL run uses it',
             'tools/pynrfbintool/': 'binary packaging: no build or HIL run uses it',
             'tools/update_bootloader.py': 'the bootloader updater: no build or HIL run uses it',
             'tools/midi_tests/': 'manual MIDI tests', 'programmers.txt': 'upload programmers: no build or HIL run '
                                                                         'uses them',
             'scripts/': 'J-Link and OpenOCD upload/debug scripts: no build or HIL run uses them'}
# libraries every sketch may pull in (the core's USB through TinyUSB, Bluefruit's storage): a change builds all
SHARED_LIBRARIES = ('Adafruit_TinyUSB_Arduino', 'Adafruit_LittleFS', 'InternalFileSytem', 'SPI', 'Wire')
# CI's lib install list (.github/workflows/githubci.yml)
EXTRA_DEPENDENCIES = ('Adafruit AHRS', 'Adafruit APDS9960 Library', 'Adafruit Arcada Library',
                      'Adafruit BMP280 Library', 'Adafruit Circuit Playground', 'Adafruit EPD', 'Adafruit GFX Library',
                      'Adafruit HX8357 Library', 'Adafruit ILI9341', 'Adafruit LIS3MDL', 'Adafruit LSM6DS',
                      'Adafruit NeoPixel', 'Adafruit NeoMatrix', 'Adafruit Sensor Calibration',
                      'Adafruit SHT31 Library', 'Adafruit SSD1306', 'Adafruit ST7735 and ST7789 Library',
                      'SdFat - Adafruit Fork', 'MIDI Library', 'Firmata')
CONTRACT_FILES = ('.claude/skills/build/scripts/check_build.py', '.claude/skills/hil-contract/scripts/hil_test.py',
                  '.claude/skills/hil-contract/scripts/hil_remote.py',
                  '.claude/skills/hil-contract/scripts/agentrc_pin.py', 'test/hil/agentrc.json')
AGENTRC = None   # set by engine(): agentrc_pin.identity()

# every other boards.txt board builds only when its own variant changes; one entry per FQBN, so the rig board
# stands for its boards.txt id
for _id, _b in boards_txt().items():
    if _b['family'] and board_fqbn(_id, _b['family']) not in {b['fqbn'] for b in BOARDS.values()}:
        BOARDS[_id] = {'fqbn': board_fqbn(_id, _b['family']), 'family': _b['family'], 'filters': _id}


def board_id(board):
    return BOARDS[board]['fqbn'].split(':')[2]


def boards_where(key, value):
    """The board names (rig boards included, the SystemView board not) whose boards.txt id has build.<key> == value."""
    ids = {i for i, b in boards_txt().items() if b[key] == value}
    return [b for b in BOARDS if board_id(b) in ids and b != SYSVIEW_BOARD]


def library_sketches(lib):
    return [e for e in E.examples() if e.startswith(f'libraries/{lib}/')]


def classify_source(path):
    """The core's own source rules (arduino_build.classify falls through to the common ones on None; a changed file
    of an example sketch is classified before this)."""
    if path in BOARDS['nrf52840dk']['base']:
        return 'boards', ('nrf52840dk',), 'the rig board\'s base image, which every HIL run checks and restores'
    for prefix, why in UNCOVERED.items():
        if path.startswith(prefix):
            return 'uncovered', None, why
    if path.startswith('cores/nRF5/sysview/'):
        return 'boards', tuple(b for f in REPRESENTATIVES.values() for b in f) + (SYSVIEW_BOARD,), \
            'SEGGER SystemView: every representative, and one sketch under debug=l3, the menu that compiles it'
    m = re.match(r'cores/nRF5/linker/(\w+\.ld)$', path)
    if m and m.group(1) != 'nrf52_common.ld':
        boards = boards_where('ldscript', m.group(1))
        return ('boards', tuple(boards), f'linker script {m.group(1)}') if boards else \
            ('uncovered', None, f'linker script {m.group(1)} is used by no boards.txt board')
    if path.startswith('cores/'):
        return 'shared', None, 'core runtime'
    if path.startswith('variants/'):
        variant = path.split('/')[1]
        boards = boards_where('variant', variant)
        return ('boards', tuple(boards), f'variant {variant}') if boards else \
            ('uncovered', None, f'variant {variant} is built by no boards.txt board')
    if path in ('boards.txt', 'platform.txt'):
        return 'boards', CI_BOARDS + ('pca10100', 'nrf52840dk'), \
            'board and build recipes: CI\'s matrix, the nRF52833 representative and the rig board'
    if path in ('tools/build_all.py', 'tools/makeboards.py', '.github/workflows/githubci.yml'):
        return 'tests', 'build', 'CI\'s matrix, filters or the boards.txt generator: the contract tests check parity'
    if path.startswith('libraries/'):
        lib = path.split('/')[1]
        if lib in SHARED_LIBRARIES:
            return 'shared', None, f'bundled {lib}, which the core or other libraries include'
        sketches = library_sketches(lib)
        return ('sketches', sketches, f'bundled {lib}') if sketches else \
            ('uncovered', None, f'bundled {lib} has no sketch to build')
    return None


def exclude(board, example):
    """tools/build_all.py's own skip: the bundled TinyUSB's examples, which its own CI builds; every sketch but
    SYSVIEW_SKETCH on the SystemView board; and the bundled nRFCrypto's examples off nRF52840, the only MDK defining
    the NRF_CRYPTOCELL its source #errors without (its markers name feather52832 and feather52833, not pca10100)."""
    return (example.startswith('libraries/Adafruit_TinyUSB_Arduino/') or
            board == SYSVIEW_BOARD and example != SYSVIEW_SKETCH or
            example.startswith('libraries/Adafruit_nRFCrypto/') and BOARDS[board]['family'] != 'nrf52840')


def vid_pid(build_dir, board=None, roster_entry=None):
    """VID:PID the sketch enumerates with: platform.txt's build.flags.usb passes -DUSB_VID/-DUSB_PID to every compile
    (compile_commands.json); None when absent or disagreeing (an nRF52832 board has no USB)."""
    try:
        commands = json.loads((Path(build_dir) / 'compile_commands.json').read_text())
    except (OSError, ValueError):
        return None
    seen = set()
    for entry in commands:
        try:
            argv = entry.get('arguments') or shlex.split(entry.get('command', ''))
        except ValueError:
            return None
        flags = dict(a[2:].split('=', 1) for a in argv if a.startswith(('-DUSB_VID=', '-DUSB_PID=')))
        if 'USB_VID' in flags and 'USB_PID' in flags:
            try:   # a C literal's integer suffix (0x239Au) is not Python's
                vid, pid = (int(re.sub(r'[uUlL]+$', '', flags[k]), 0) for k in ('USB_VID', 'USB_PID'))
            except ValueError:
                return None
            seen.add(f'{vid:04x}:{pid:04x}')
    return seen.pop() if len(seen) == 1 else None


E = None   # agentrc's arduino_build, bound to this profile by engine()


def engine():
    """agentrc's arduino_build bound to this profile."""
    global AGENTRC, E
    if E is None:
        AGENTRC = agentrc_pin.identity()
        E = agentrc_pin.load('skills/arduino/scripts/arduino_build.py').bind(sys.modules[__name__])
    return E


def main(argv=None):
    try:
        eng = engine()
    except agentrc_pin.PinError as e:
        print(json.dumps({'version': 2, 'pass': False, 'error': str(e), 'remedy': e.remedy}))
        return 2
    return eng.main(argv)


if __name__ == '__main__':
    sys.modules.setdefault('check_build', sys.modules[__name__])
    sys.exit(main())
