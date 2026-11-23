#!/usr/bin/env python3
"""The Adafruit nRF52 core HIL harness (SKILL.md beside this directory). Linux only.

  hil_test.py run [-b BOARD]... [-t TEST]... [--record-dir DIR] [--run-dir DIR] [--staged MANIFEST] ROSTER
  hil_test.py commission -b BOARD [--record-dir DIR] ROSTER
  hil_test.py flash -b BOARD --build-record R --sketch S --under-hold REASON [--staged M] ROSTER
  hil_test.py restore -b BOARD --build-record R --under-hold REASON [--report PATH] [--release] [--staged M] ROSTER
  hil_test.py gdb -b BOARD --build-record R --sketch S --under-hold REASON [--ex CMD]... [--timeout S] [--staged M]
                 ROSTER
  hil_test.py exec -b BOARD --under-hold REASON [--timeout S] [--staged M] ROSTER -- ARGV...
  hil_test.py console -b BOARD --under-hold REASON [--seconds S] [--baud N] [--reset] [--staged M] ROSTER
  hil_test.py check -b BOARD --build-record R --under-hold REASON [--test T] [--staged M] ROSTER
  hil_test.py report RUN_DIR [-b BOARD]...
  hil_test.py lock (hold|release|status) ...        agentrc's hil_lock.py

flash, restore, gdb and check take --bundle DIR, a record bundle `hil_remote.py session stage` staged,
--build-record then inside it. restore --release passes only when the hold was released too.

The last stdout line of every command but lock is JSON. Exits: 0 pass, 1 a board failed or did not run (cleaned up
or untouched), 2 refused before any hardware access, 3 a session command (--under-hold) whose hold the lock record
shows no live holder of, 4 a cleanup did not finish (board held).

This file is the core's HIL profile for agentrc's arduino_hil.py (the `arduino` skill): `cdc` is that engine's shared
core battery (CORE_TESTS, test/hil/core_cdc/core_cdc.ino), `rtos` its timing check followed by this core's RTOS
exchange (test/hil/core_rtos/core_rtos.ino); the mechanics are there, loaded from the agentrc agentrc_pin.py selects.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'build' / 'scripts'))
import agentrc_pin  # noqa: E402
import check_build as BUILD  # noqa: E402

TESTS = {'cdc': 'core_cdc', 'rtos': 'core_rtos'}
ROLE_TESTS = {'device': tuple(TESTS)}
RESTORE_TEST = {'device': 'cdc'}
REPORT_SCHEMA = 'adafruit-nrf52-core-hil'
# hil_remote.py (agentrc arduino_remote.py)
REMOTE_BASE = '~/hil-remote/adafruit-nrf52'
REMOTE_ROSTER = 'test/hil/ci.json'
HIL = '.claude/skills/hil-contract/scripts/hil_test.py'
CORE_TESTS = {'cdc': ('cdc', 'tinyusb')}
# ECHO lengths: one byte, either side of a 64-byte full-speed bulk packet, and several packets
ECHO_LENGTHS = (1, 63, 64, 65, 1000)
TIMING_SPAN = 3.0   # seconds between the two TIME reads
DELAY_MS = 1000
# a coarse functional bound on the core's time base (FreeRTOS ticks at 1024 Hz, cores/nRF5/freertos/config/
# FreeRTOSConfig.h) against the host's monotonic clock, beyond the exchange's own round trips: no clock-accuracy claim
TIMING_TOLERANCE = 0.01
RTOS_MS = 1000   # how long core_rtos runs its two workers and its timer
# core_rtos's periods (WORK_MS, TIMER_MS) as whole ticks: vTaskDelay(10 ms) = 10 ticks, a 50 ms timer = 51 ticks
WORK_TICKS, TIMER_TICKS, TICK_HZ = 10, 51, 1024
# vTaskDelay is relative, so a worker's period also takes the wait before it runs again: its count may fall this
# fraction below the 10-tick count
WORK_LATENESS = 0.1

E = None   # agentrc's arduino_hil, bound to this profile by engine()


def rtos_problem(f):
    """'' when a core_rtos RTOS reply ran RTOS_MS and each worker and the timer kept their periods, else why."""
    try:
        ms, a, b, ticks = (int(f[k]) for k in ('ms', 'a', 'b', 'timer'))
    except (KeyError, ValueError):
        return f'RTOS reply {f}'
    if not RTOS_MS <= ms <= RTOS_MS + 50:
        return f'RTOS ran {ms} ms for {RTOS_MS}'
    # vTaskDelay(n) blocks n - 1 to n tick periods; each worker also queued once before its first delay
    lo, hi = ms * TICK_HZ / 1000 / WORK_TICKS * (1 - WORK_LATENESS), ms * TICK_HZ / 1000 / (WORK_TICKS - 1) + 2
    slow = {k: v for k, v in (('a', a), ('b', b)) if not lo <= v <= hi}
    if slow:
        return f'worker messages {slow} outside [{lo:.0f}, {hi:.0f}] over {ms} ms'
    want = ms * TICK_HZ / 1000 / TIMER_TICKS
    return '' if abs(ticks - want) <= 1.5 else f'timer ticked {ticks} times over {ms} ms, {want:.1f} expected'


def test_rtos(dev, nonce, deadline, tracker, run_dir, board):
    """core_rtos: the shared timing check (core_timing_check), then two worker tasks and a software timer keep their
    periods while loop() drains their queue (rtos_problem)."""
    def check(console, tty):
        ok, detail, _ = E.core_timing_check(console, tty, nonce, 'tinyusb', board)
        if not ok:
            return False, detail
        r, _, _ = E.core_timed(console, f'RTOS {nonce}r {RTOS_MS}', nonce + 'r', 'rtos')
        why = rtos_problem(r)
        if why:
            return False, f'{tty}: {why}'
        return True, f'{detail}, rtos {r["ms"]} ms a={r["a"]} b={r["b"]} timer={r["timer"]}'
    return E.core_session(dev, deadline, run_dir, nonce, check)


TEST_FUNCS = {'rtos': test_rtos}


def engine():
    """agentrc's arduino_hil bound to this profile (and its build profile)."""
    global E
    if E is None:
        BUILD.engine()
        E = agentrc_pin.load('skills/arduino/scripts/arduino_hil.py').bind(sys.modules[__name__])
    return E


def main(argv=None):
    try:
        eng = engine()
    except agentrc_pin.PinError as e:
        print(json.dumps({'pass': False, 'error': str(e), 'remedy': e.remedy}), flush=True)
        return 2
    return eng.main(argv)


if __name__ == '__main__':
    sys.modules.setdefault('hil_test', sys.modules[__name__])
    sys.exit(main())
