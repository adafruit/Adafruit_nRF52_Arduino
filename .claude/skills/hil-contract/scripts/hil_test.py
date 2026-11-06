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

This file is the core's HIL profile for agentrc's arduino_hil.py (the `arduino` skill): its tests over the HIL
sketches' serial protocol (test/hil/core_cdc/core_cdc.ino, test/hil/core_rtos/core_rtos.ino); the mechanics are
there, loaded from the agentrc agentrc_pin.py selects.
"""

import json
import os
import sys
import time
import zlib
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


class Console:
    """The DUT's USB CDC tty: one command line out, its reply line (the one carrying the command's nonce) back."""

    def __init__(self, port, deadline, log):
        self.port, self.deadline, self.log = port, deadline, log

    def send(self, data):
        import serial
        self.port.write_timeout = max(0.1, self.deadline - time.monotonic())
        try:
            self.port.write(data)
        except serial.SerialTimeoutException:
            raise TimeoutError(f'write of {data[:40]!r} timed out')

    def read_exact(self, n):
        got = b''
        while len(got) < n and time.monotonic() < self.deadline:
            got += self.port.read(n - len(got))
        return got

    def ask(self, line, nonce):
        self.send(line.encode() + b'\n')
        self.log.append(f'> {line}')
        return self.reply(nonce)

    def reply(self, nonce):
        buf = b''
        while time.monotonic() < self.deadline:
            buf += self.port.read(self.port.in_waiting or 1)   # read(n) blocks for n bytes or the timeout
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                text = raw.decode(errors='replace').strip()
                if text:
                    self.log.append(f'< {text}')
                words = text.split()
                if len(words) >= 2 and words[0] in ('HIL_INFO', 'HIL_RESULT') and words[1] == nonce:
                    return words
        raise TimeoutError(f'no reply for nonce {nonce} within the test timeout')


def fields(words):
    return dict(w.split('=', 1) for w in words if '=' in w)


class Rejected(Exception):
    pass


def passed(words, op):
    """The fields of a `HIL_RESULT <nonce> <op> PASS ...` reply; Rejected naming the reply for any other."""
    if words[0] != 'HIL_RESULT' or words[2:4] != [op, 'PASS']:
        raise Rejected(f'{op}: {" ".join(words)}')
    return fields(words)


def chip_of(board):
    return BUILD.BOARDS[board['name']]['family'] if board.get('name') in BUILD.BOARDS else None


def open_console(dev, deadline):
    import serial
    tty = E.node(dev, '*:*/tty/tty*', deadline)
    port = serial.Serial(tty, 115200, timeout=0.2)
    port.reset_input_buffer()
    return tty, port


def session(dev, deadline, run_dir, nonce, check):
    """Open the DUT's CDC tty, run check(console, tty) -> (ok, detail), keep the exchange in the run dir."""
    tty, port = open_console(dev, deadline)
    log = []
    try:
        return check(Console(port, deadline, log), tty)
    except Rejected as e:
        return False, f'{tty}: {e}'
    finally:
        port.close()
        (Path(run_dir) / f'core-{nonce}.log').write_text('\n'.join(log) + '\n')


def info_problem(console, nonce, board):
    """'' when INFO reports TinyUSB and the board's chip family, else why not; the INFO fields."""
    words = console.ask(f'INFO {nonce}', nonce)
    if words[0] != 'HIL_INFO':
        return f'INFO answered {" ".join(words)}', {}
    got = fields(words)
    want = {'stack': 'tinyusb', 'chip': chip_of(board)}
    wrong = {k: got.get(k) for k, v in want.items() if v and got.get(k) != v}
    return (f'INFO {wrong} != {want}' if wrong else ''), got


def echo(console, nonce):
    """ECHO each of ECHO_LENGTHS: the bytes come back as sent, then a PASS line with their length and crc32."""
    done = []
    for i, n in enumerate(ECHO_LENGTHS):
        sub = f'{nonce}e{i}'
        data = os.urandom(n)
        console.send(f'ECHO {sub} {n}\n'.encode() + data)
        got = console.read_exact(n)
        if got != data:
            return False, f'ECHO {n}: {len(got)} bytes back, {"equal" if got == data[:len(got)] else "differing"}'
        f = passed(console.reply(sub), 'echo')
        if (f.get('len'), f.get('crc')) != (str(n), f'{zlib.crc32(data):08x}'):
            return False, f'ECHO {n}: result {f} (want len={n} crc={zlib.crc32(data):08x})'
        done.append(n)
    return True, f'echoed {done} bytes'


def test_cdc(dev, nonce, deadline, tracker, run_dir, board):
    """core_cdc: INFO names the chip, ECHO returns every byte across packet bounds."""
    def check(console, tty):
        why, got = info_problem(console, nonce + 'i', board)
        if why:
            return False, f'{tty}: {why}'
        ok, detail = echo(console, nonce)
        return ok, f'{tty}: f_cpu={got.get("f_cpu")}: {detail}'
    return session(dev, deadline, run_dir, nonce, check)


def timed(console, line, nonce, op):
    """(fields of its PASS reply, host send time, host receive time) of one exchange."""
    t0 = time.monotonic()
    words = console.ask(line, nonce)
    return passed(words, op), t0, time.monotonic()


def timing_problem(dev_ms, host_lo, host_hi):
    """'' when the DUT's elapsed ms lies within the host's bracket [lo, hi] widened by TIMING_TOLERANCE, else why."""
    lo, hi = host_lo * (1 - TIMING_TOLERANCE), host_hi * (1 + TIMING_TOLERANCE)
    return '' if lo <= dev_ms <= hi else f'{dev_ms:.1f} ms outside the host bracket [{lo:.1f}, {hi:.1f}] ms'


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
    """core_rtos: millis() and micros() advance with the host's monotonic clock over TIMING_SPAN seconds, agree with
    each other, delay(DELAY_MS) measures DELAY_MS within the bound (TIMING_TOLERANCE, coarse), and two worker tasks
    and a software timer keep their periods while loop() drains their queue (rtos_problem)."""
    def check(console, tty):
        why, info = info_problem(console, nonce + 'i', board)
        if why:
            return False, f'{tty}: {why}'
        a, a0, a1 = timed(console, f'TIME {nonce}a', nonce + 'a', 'time')
        time.sleep(TIMING_SPAN)
        b, b0, b1 = timed(console, f'TIME {nonce}b', nonce + 'b', 'time')
        try:
            ms = (int(b['ms']) - int(a['ms'])) & 0xffffffff
            us = (int(b['us']) - int(a['us'])) & 0xffffffff
        except (KeyError, ValueError):
            return False, f'{tty}: TIME replies {a}, {b}'
        # the DUT read its clock somewhere inside each exchange: the elapsed host time lies in [b0 - a1, b1 - a0]
        why = timing_problem(ms, (b0 - a1) * 1000, (b1 - a0) * 1000)
        if why:
            return False, f'{tty}: millis() {why}'
        if abs(us / 1000 - ms) > 2:
            return False, f'{tty}: micros() advanced {us} us while millis() advanced {ms} ms'
        d, _, _ = timed(console, f'DELAY {nonce}d {DELAY_MS}', nonce + 'd', 'delay')
        try:
            delay_us = int(d['us'])
        except (KeyError, ValueError):
            return False, f'{tty}: DELAY reply {d}'
        why = timing_problem(delay_us / 1000, DELAY_MS, DELAY_MS + 2)   # delay() may run past its last tick
        if why:
            return False, f'{tty}: delay({DELAY_MS}) {why}'
        r, _, _ = timed(console, f'RTOS {nonce}r {RTOS_MS}', nonce + 'r', 'rtos')
        why = rtos_problem(r)
        if why:
            return False, f'{tty}: {why}'
        return True, f'{tty}: f_cpu={info.get("f_cpu")}: millis {ms} ms over host [{(b0 - a1) * 1000:.1f}, ' \
                     f'{(b1 - a0) * 1000:.1f}] ms, micros {us} us, delay({DELAY_MS}) {delay_us} us, rtos ' \
                     f'{r["ms"]} ms a={r["a"]} b={r["b"]} timer={r["timer"]}'
    return session(dev, deadline, run_dir, nonce, check)


TEST_FUNCS = {'cdc': test_cdc, 'rtos': test_rtos}


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
