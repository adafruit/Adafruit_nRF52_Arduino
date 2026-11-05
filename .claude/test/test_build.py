#!/usr/bin/env python3
"""The build contract's profile: the board table against CI's matrix and boards.txt, sketch eligibility against
tools/build_all.py, source classification and selection through agentrc's arduino_build.py in core mode, the rig
board's base image, and the VID:PID adapter. arduino-cli is not run; a real build is evidence only from a run of the
script itself."""
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'build' / 'scripts'))
import check_build as cb  # noqa: E402

E = cb.engine()
WORKFLOW = ROOT / '.github/workflows/githubci.yml'


def ci_matrix():
    text = WORKFLOW.read_text()
    return re.findall(r"^\s+- '([^']+)'", text.split('arduino-platform:')[1].split('steps:')[0], re.M)


def build_all():
    """tools/build_all.py's module namespace, without running its __main__."""
    ns = {'__name__': 'build_all'}
    exec(compile((ROOT / 'tools/build_all.py').read_text(), 'build_all.py', 'exec'), ns)
    return ns


class Tables(unittest.TestCase):
    def test_ci_matrix_is_the_table(self):
        self.assertEqual(sorted(ci_matrix()), sorted(cb.CI_BOARDS))
        get_sd = build_all()['get_sd']
        for b in cb.CI_BOARDS:
            self.assertEqual(cb.BOARDS[b]['fqbn'], f'adafruit:nrf52:{b}:softdevice={get_sd(b)},debug=l0', b)

    def test_ci_dependencies_are_the_extras(self):
        lines = [ln for ln in WORKFLOW.read_text().splitlines() if ln.strip().startswith('arduino-cli lib install')]
        self.assertEqual(sorted(n for ln in lines for n in re.findall(r'"([^"]+)"', ln)),
                         sorted(cb.EXTRA_DEPENDENCIES))

    def test_library_name_is_the_platforms(self):
        self.assertIn(f'name={cb.LIBRARY_NAME}\n', (ROOT / 'platform.txt').read_text())

    def test_every_boards_txt_board_has_an_entry_of_its_family(self):
        ids = cb.boards_txt()
        self.assertTrue(ids)
        get_sd = build_all()['get_sd']
        for i, b in ids.items():
            self.assertTrue(b['family'], i)
            entries = [e for e in cb.BOARDS.values() if e['fqbn'].split(':')[2] == i]
            self.assertTrue(entries, i)
            for e in entries:
                self.assertEqual(e['family'], b['family'], i)
            self.assertEqual(cb.SOFTDEVICE[b['family']], get_sd(i), i)   # build_all.py's choice by board name
        self.assertEqual(cb.CI_FAMILY, {b: ids[b]['family'] for b in cb.CI_BOARDS})

    def test_boards_txt_is_makeboards_output(self):
        out = subprocess.run([sys.executable, str(ROOT / 'tools/makeboards.py')], capture_output=True, text=True,
                             check=True, cwd=ROOT).stdout
        self.assertEqual(out, (ROOT / 'boards.txt').read_text(), 'edit tools/makeboards.py and regenerate')

    def test_representatives_and_the_rig_board(self):
        for family, boards in cb.REPRESENTATIVES.items():
            for b in boards:
                self.assertEqual(cb.BOARDS[b]['family'], family)
        self.assertEqual({b for b, v in cb.BOARDS.items() if 'span' in v}, {'nrf52840dk'})
        self.assertEqual(cb.BOARDS['nrf52840dk']['fqbn'], 'adafruit:nrf52:pca10056:softdevice=s140v6,debug=l0')

    def test_the_rig_span_is_the_linker_scripts_application_region(self):
        ld = (ROOT / 'cores/nRF5/linker/nrf52840_s140_v6.ld').read_text()
        m = re.search(r'FLASH \(rx\)\s*:\s*ORIGIN = (0x[0-9A-Fa-f]+), LENGTH = (0x[0-9A-Fa-f]+) - \1', ld)
        self.assertEqual(cb.BOARDS['nrf52840dk']['span'], (int(m[1], 16), int(m[2], 16)))

    def test_the_rig_base_is_the_bootburn_image_and_the_app_valid_settings_word(self):
        base = cb.BOARDS['nrf52840dk']['base']
        self.assertEqual(base[0], 'bootloader/pca10056/pca10056_bootloader-0.9.1_s140_6.1.1.hex')   # platform.txt's
        self.assertIn('{build.variant}_bootloader-0.9.1_{build.sd_name}_{build.sd_version}',
                      (ROOT / 'platform.txt').read_text())
        # the MBR's bootloader and parameter page words erased, so UICR alone points the MBR, and its parameter page
        # (UICR NRFFW[1]) erased: no MBR command pending (S140 SDS §12); bootloader settings bank_0 valid with CRC 0
        # (no check) and bank_1 BANK_INVALID_APP, as an app DFU leaves it (0xAA would resume an update)
        self.assertEqual(E.hex_runs(ROOT / base[1]),
                         [(0xFF8, b'\xff' * 8), (0xFE000, b'\xff' * 0x1000 + b'\x01\x00\x00\x00\xff\x00')])
        runs, why = E.base_runs('nrf52840dk')
        self.assertEqual(why, '')
        self.assertTrue(all(s < 0x10000000 for s, _ in runs))   # no UICR write
        self.assertEqual(runs[0][0], 0)   # the MBR
        self.assertTrue(any(s <= 0xFF8 and s + len(d) >= 0x1000 for s, d in runs))   # MBR_BOOTLOADER/PARAM_PAGE_ADDR

    def test_hil_sketches_exist(self):
        for sketch in cb.HIL_SKETCHES['device']:
            self.assertTrue((ROOT / sketch / f'{Path(sketch).name}.ino').is_file(), sketch)

    def test_unbuildable_examples_are_pinned(self):
        # the sketches tools/build_all.py skips on every CI board (their .test.only/.test.skip markers) or never
        # builds (the bundled TinyUSB's, which its own CI builds): no build here compiles them either
        self.assertEqual([e for e in E.unbuildable_examples()
                          if not e.startswith('libraries/Adafruit_TinyUSB_Arduino/')], UNBUILDABLE)

    def test_contract_files_exist(self):
        for f in cb.CONTRACT_FILES:
            self.assertTrue((ROOT / f).is_file(), f)


UNBUILDABLE = ['libraries/Bluefruit52Lib/examples/Peripheral/tf4micro-motion-kit',   # .none-*.test.only (TFLite)
               'libraries/Bluefruit52Lib/examples/Projects/homekit/homekit_lightbulb']   # .all.test.skip


class Eligibility(unittest.TestCase):
    """build_all.py's rules: its markers and its TinyUSB skip."""

    def test_bundled_tinyusb_examples_never_build(self):
        examples = [e for e in E.examples() if e.startswith('libraries/Adafruit_TinyUSB_Arduino/')]
        self.assertTrue(examples)
        self.assertFalse([e for e in examples for b in cb.BOARDS if E.eligible(b, e)])

    def test_a_board_marker_skips_that_board_only(self):
        ex = 'libraries/Bluefruit52Lib/examples/Hardware/adc_vbat'
        self.assertFalse(E.eligible('nrf52840dk', ex))   # .pca10056.test.skip: the DK has no VBAT divider
        self.assertTrue(E.eligible('feather52840', ex))

    def test_nrfcrypto_examples_build_only_on_nrf52840(self):
        # Adafruit_nRFCrypto.cpp #errors without NRF_CRYPTOCELL, which only the nRF52840 MDK defines
        examples = [e for e in E.examples() if e.startswith('libraries/Adafruit_nRFCrypto/')]
        self.assertTrue(examples)
        for b in cb.BOARDS:
            self.assertEqual([E.eligible(b, e) for e in examples],
                             [cb.BOARDS[b]['family'] == 'nrf52840' and b != cb.SYSVIEW_BOARD] * len(examples), b)


class Selection(unittest.TestCase):
    def test_rules(self):
        core = ('boards', tuple(b for f in cb.REPRESENTATIVES.values() for b in f) + (cb.SYSVIEW_BOARD,))
        cases = {'cores/nRF5/wiring.c': core,
                 'cores/nRF5/freertos/Source/tasks.c': core,
                 'libraries/Adafruit_TinyUSB_Arduino': ('shared', None),
                 'libraries/InternalFileSytem/src/InternalFileSystem.cpp': ('shared', None),
                 'libraries/Servo/src/Servo.h': ('sketches', [e for e in E.examples()
                                                              if e.startswith('libraries/Servo/')]),
                 'test/hil/core_cdc/core_cdc.ino': ('hil_sketch', 'test/hil/core_cdc'),
                 'test/hil/ci.json': ('hil', None),
                 '.claude/skills/build/scripts/check_build.py': ('tests', 'build'),
                 'tools/build_all.py': ('tests', 'build'), 'tools/makeboards.py': ('tests', 'build'),
                 '.github/workflows/githubci.yml': ('tests', 'build'),
                 'README.md': ('nothing', None), 'CLAUDE.md': ('nothing', None), 'changelog.md': ('nothing', None),
                 'bootloader/feather_nrf52840_express/feather_nrf52840_express_bootloader-0.9.1_s140_6.1.1.hex':
                     ('uncovered', None),
                 'tools/uf2conv/uf2conv.py': ('uncovered', None), 'programmers.txt': ('uncovered', None),
                 'scripts/jlink/x': ('uncovered', None), 'weird/file': ('uncovered', None)}
        for path, want in cases.items():
            self.assertEqual(E.classify(path)[:2], want, path)

    def test_the_rig_base_builds_the_rig_board(self):
        for f in cb.BOARDS['nrf52840dk']['base']:
            self.assertEqual(E.classify(f)[:2], ('boards', ('nrf52840dk',)), f)

    def test_a_variant_or_linker_script_builds_every_board_on_it(self):
        effect, boards, _ = E.classify('variants/feather_nrf52840_express/variant.cpp')
        self.assertEqual((effect, sorted(boards)), ('boards', ['feather52840']))
        effect, boards, _ = E.classify('variants/pca10056/variant.h')
        self.assertEqual((effect, sorted(boards)), ('boards', ['nrf52840dk']))
        effect, boards, _ = E.classify('cores/nRF5/linker/nrf52833_s140_v7.ld')
        self.assertEqual((effect, sorted(boards)), ('boards', ['pca10100']))
        self.assertEqual(E.classify('cores/nRF5/linker/nrf52_common.ld')[:2], E.classify('cores/nRF5/wiring.c')[:2])
        self.assertEqual(E.classify('variants/nowhere/variant.h')[0], 'uncovered')

    def test_core_builds_every_representative_and_one_sketch_with_systemview_enabled(self):
        # SEGGER_SYSVIEW_FreeRTOS.c is all under #if CFG_SYSVIEW, which only boards.txt's debug=l3 menu sets, and so
        # are blocks of main.cpp and FreeRTOSConfig.h
        reps = sorted(b for f in cb.REPRESENTATIVES.values() for b in f)
        for path in ('cores/nRF5/sysview/SEGGER_SYSVIEW_FreeRTOS.c', 'cores/nRF5/sysview/SEGGER/SEGGER_SYSVIEW.c',
                     'cores/nRF5/main.cpp', 'cores/nRF5/freertos/config/FreeRTOSConfig.h'):
            sel = E.select([path])
            self.assertEqual(sel['uncovered'], [], path)
            enabled = {b: x for b, x in sel['builds'].items() if E.fqbn(b, x[0]).endswith(',debug=l3')}
            self.assertTrue(enabled, f'{path}: no selected build compiles SystemView')
            self.assertEqual(sorted(set(sel['builds']) - set(enabled)), reps, path)
            self.assertEqual(list(enabled.values()), [[cb.SYSVIEW_SKETCH]], path)
            self.assertIn(f'\n{cb.board_id(next(iter(enabled)))}.menu.debug.l3.build.sysview_flags=-DCFG_SYSVIEW=1\n',
                          (ROOT / 'boards.txt').read_text())

    def test_boards_txt_builds_ci_the_833_representative_and_the_rig_board(self):
        effect, boards, _ = E.classify('boards.txt')
        self.assertEqual(effect, 'boards')
        self.assertEqual(sorted(boards), sorted(cb.CI_BOARDS + ('pca10100', 'nrf52840dk')))

    def test_core_change_runs_the_farm_board_and_names_832_and_833_uncovered(self):
        sel = E.select(['cores/nRF5/wiring.c'], roster=['nrf52840dk'])
        self.assertEqual(sorted(sel['builds']),
                         sorted([b for f in cb.REPRESENTATIVES.values() for b in f] + [cb.SYSVIEW_BOARD]))
        self.assertEqual(sel['hil']['boards'], ['nrf52840dk'])
        self.assertEqual(sorted(u['family'] for u in sel['hil']['uncovered']), ['nrf52832', 'nrf52833'])

    def test_hil_sketch_builds_only_on_the_rig_board(self):
        sel = E.select(['test/hil/core_rtos/core_rtos.ino'], roster=['nrf52840dk'])
        self.assertEqual(sorted(sel['builds']), ['nrf52840dk'])
        self.assertTrue(sel['hil']['full'])


class Identity(unittest.TestCase):
    def commands(self, *flags):
        d = Path(self.tmp.name)
        (d / 'compile_commands.json').write_text(json.dumps(
            [{'file': f'/x/{i}.cpp', 'arguments': ['arm-none-eabi-g++', *f.split()]} for i, f in enumerate(flags)]))
        return cb.vid_pid(d)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_vid_pid_from_the_compile_flags(self):
        self.assertEqual(self.commands('-DUSB_VID=0x239A -DUSB_PID=0x80DA -DUSBCON', '-O2'), '239a:80da')

    def test_vid_pid_absent_or_disagreeing_is_none(self):
        self.assertIsNone(self.commands('-O2'))
        self.assertIsNone(self.commands('-DUSB_VID=0x239A -DUSB_PID=0x8029', '-DUSB_VID=0x239A -DUSB_PID=0x802A'))
        self.assertIsNone(cb.vid_pid(Path(self.tmp.name) / 'missing'))

    def test_vid_pid_takes_c_integer_suffixes_and_refuses_non_numbers(self):
        self.assertEqual(self.commands('-DUSB_VID=0x239Au -DUSB_PID=0x80DAUL'), '239a:80da')
        self.assertIsNone(self.commands('-DUSB_VID=VENDOR -DUSB_PID=0x80DA'))


if __name__ == '__main__':
    unittest.main()
