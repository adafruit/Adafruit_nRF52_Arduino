#!/usr/bin/env python3
"""The HIL profile: the host side of the HIL sketches' serial protocol against a simulated DUT, the rosters, and the
profile bound to agentrc's arduino_hil.py. The simulation proves the host's parsing and verdicts only; what the
firmware does is a farm run's evidence."""
import json
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import hil_test as ht  # noqa: E402

E = ht.engine()
BOARD = {'name': 'nrf52840dk'}


class FakeClock:
    """hil_test's `time`: the host clock moves only on sleep() and on the FakeDut's port I/O, so every exchange takes
    the same host time whatever the machine's load."""

    def __init__(self):
        self.us = 0

    def monotonic(self):
        return self.us / 1e6

    def sleep(self, seconds):
        self.us += round(seconds * 1e6)


class FakeDut:
    """core_cdc's and core_rtos's protocol (test/hil/core_cdc/core_cdc.ino, test/hil/core_rtos/core_rtos.ino) over a
    byte stream, each write and read taking IO_US of the host's clock, with millis() running `rate` and micros()
    `us_rate` times the host's clock, `wrong_len` added to each echo's reported length and `rtos` the RTOS reply's
    counts."""
    IO_US = 1000

    def __init__(self, clock, stack='tinyusb', chip='nrf52840', rate=1.0, us_rate=1.0, corrupt=False, wrong_len=0,
                 delay_us=1000000, status='PASS', rtos=None):
        self.clock, self.stack, self.chip, self.rate, self.us_rate = clock, stack, chip, rate, us_rate
        self.corrupt, self.wrong_len, self.delay_us = corrupt, wrong_len, delay_us
        self.rtos = rtos or {'ms': 1002, 'a': 103, 'b': 103, 'timer': 20}   # nrf52-core-hil-1 on the DK
        self.status = status   # what every HIL_RESULT reports
        self.out, self.pending = b'', b''
        self.echo = None   # (nonce, bytes left, bytes so far)
        self.closed = False

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return min(len(self.out), 17)   # replies arrive in fragments

    def read(self, n):
        self.clock.us += self.IO_US
        chunk, self.out = self.out[:n], self.out[n:]
        return chunk

    def write(self, data):
        self.clock.us += self.IO_US
        self.pending += data
        while self.pending:
            if self.echo:
                nonce, left, seen = self.echo
                take, self.pending = self.pending[:left], self.pending[left:]
                seen += take
                self.out += bytes([take[0] ^ 1]) + take[1:] if self.corrupt and take else take
                left -= len(take)
                self.echo = (nonce, left, seen) if left else None
                if not left:
                    self.out += (f'\r\nHIL_RESULT {nonce} echo {self.status} len={len(seen) + self.wrong_len} '
                                 f'crc={zlib.crc32(seen):08x}\r\n').encode()
                continue
            if b'\n' not in self.pending:
                return
            line, self.pending = self.pending.split(b'\n', 1)
            self.command(line.decode())

    def command(self, line):
        w = line.split()
        cmd, nonce = w[0], w[1]
        if cmd == 'INFO':
            reply = f'HIL_INFO {nonce} stack={self.stack} chip={self.chip} f_cpu=64000000'
        elif cmd == 'ECHO':
            self.echo = (nonce, int(w[2]), b'')
            return
        elif cmd == 'TIME':
            ms, us = int(self.clock.us * self.rate) // 1000, int(self.clock.us * self.us_rate)
            reply = f'HIL_RESULT {nonce} time {self.status} ms={ms} us={us}'
        elif cmd == 'DELAY':
            reply = f'HIL_RESULT {nonce} delay {self.status} ms={w[2]} us={self.delay_us}'
        elif cmd == 'RTOS':
            reply = f'HIL_RESULT {nonce} rtos {self.status} ' + ' '.join(f'{k}={v}' for k, v in self.rtos.items())
        else:
            reply = f'HIL_RESULT {nonce} ? REJECT why=unknown'
        self.out += (reply + '\r\n').encode()


class Protocol(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = FakeClock()
        clock = mock.patch.object(ht, 'time', self.clock)
        clock.start()
        self.addCleanup(clock.stop)

    def dut(self, **kw):
        return FakeDut(self.clock, **kw)

    def run_test(self, fn, dut, timeout=5):
        with mock.patch.object(ht, 'open_console', return_value=('/dev/ttyACM0', dut)):
            result = fn({'dir': '/x'}, 'abcd1234', self.clock.monotonic() + timeout, None, self.tmp.name, BOARD)
        self.assertTrue(dut.closed)
        self.assertTrue((Path(self.tmp.name) / 'core-abcd1234.log').is_file())
        return result

    def test_cdc_passes(self):
        ok, detail = self.run_test(ht.test_cdc, self.dut())
        self.assertTrue(ok, detail)
        self.assertIn('echoed [1, 63, 64, 65, 1000] bytes', detail)

    def test_cdc_fails_on_another_chip_or_stack(self):
        ok, detail = self.run_test(ht.test_cdc, self.dut(chip='nrf52833'))
        self.assertFalse(ok)
        self.assertIn("'chip': 'nrf52833'", detail)
        ok, detail = self.run_test(ht.test_cdc, self.dut(stack='arduino'))
        self.assertFalse(ok)

    def test_cdc_fails_on_corrupted_echo(self):
        ok, detail = self.run_test(ht.test_cdc, self.dut(corrupt=True))
        self.assertFalse(ok)
        self.assertIn('differing', detail)

    def test_cdc_fails_on_a_wrong_echo_length(self):
        ok, detail = self.run_test(ht.test_cdc, self.dut(wrong_len=1))
        self.assertFalse(ok)
        self.assertIn('ECHO 1: result', detail)

    def test_rtos_passes_at_the_hosts_rate(self):
        ok, detail = self.run_test(ht.test_rtos, self.dut())
        self.assertTrue(ok, detail)
        self.assertIn('rtos 1002 ms a=103 b=103 timer=20', detail)

    def test_rtos_fails_on_a_clock_off_by_more_than_the_bound(self):
        ok, detail = self.run_test(ht.test_rtos, self.dut(rate=1.05))
        self.assertFalse(ok)
        self.assertIn('millis()', detail)

    def test_rtos_fails_on_micros_off_while_millis_agrees(self):
        ok, detail = self.run_test(ht.test_rtos, self.dut(us_rate=1.05))
        self.assertFalse(ok)
        self.assertIn('micros()', detail)

    def test_rtos_fails_on_a_short_delay(self):
        ok, detail = self.run_test(ht.test_rtos, self.dut(delay_us=900000))
        self.assertFalse(ok)
        self.assertIn('delay(1000)', detail)

    def test_rtos_fails_on_a_stalled_worker_or_timer(self):
        for rtos, why in (({'ms': 1002, 'a': 103, 'b': 4, 'timer': 20}, "worker messages {'b': 4}"),
                          ({'ms': 1002, 'a': 103, 'b': 103, 'timer': 0}, 'timer ticked 0 times'),
                          ({'ms': 1400, 'a': 140, 'b': 140, 'timer': 28}, 'RTOS ran 1400 ms'),
                          ({'ms': 1002, 'a': 103}, 'RTOS reply')):
            with self.subTest(why=why):
                ok, detail = self.run_test(ht.test_rtos, self.dut(rtos=rtos))
                self.assertFalse(ok)
                self.assertIn(why, detail)

    def test_rtos_bounds_take_the_tick_rounding(self):
        # over 1000 ms: 9-tick periods of 1/1024 s plus 2 at most (115.8), 10-tick periods less the 10% lateness
        # allowance at least (92.2); the timer's 51 ticks give 20
        for a in (93, 115):
            self.assertEqual(ht.rtos_problem({'ms': '1000', 'a': str(a), 'b': str(a), 'timer': '20'}), '', a)
        for a in (92, 116):
            self.assertIn('worker messages', ht.rtos_problem({'ms': '1000', 'a': str(a), 'b': str(a), 'timer': '20'}))

    def test_a_reply_with_another_nonce_times_out(self):
        dut = self.dut()
        dut.command = lambda line: setattr(dut, 'out', dut.out + b'HIL_INFO other stack=tinyusb\r\n')
        with self.assertRaises(TimeoutError):
            self.run_test(ht.test_cdc, dut, timeout=0.5)

    def test_a_reply_other_than_pass_fails(self):
        for fn in (ht.test_cdc, ht.test_rtos):
            with self.subTest(fn=fn):
                ok, detail = self.run_test(fn, self.dut(status='REJECT'))
                self.assertFalse(ok)
                self.assertIn('REJECT', detail)

    def test_info_answered_by_a_result_fails(self):
        dut = self.dut()
        info = dut.command

        def command(line):
            if line.startswith('INFO'):
                dut.out += f'HIL_RESULT {line.split()[1]} ? REJECT why=unknown\r\n'.encode()
            else:
                info(line)
        dut.command = command
        ok, detail = self.run_test(ht.test_cdc, dut)
        self.assertFalse(ok)
        self.assertIn('INFO answered', detail)

    def test_timing_bracket(self):
        self.assertEqual(ht.timing_problem(1000, 995, 1005), '')
        self.assertIn('outside', ht.timing_problem(1100, 995, 1005))


class Rosters(unittest.TestCase):
    def test_checked_in_roster_loads(self):
        roster = E.load_roster(ROOT / 'test/hil/ci.json')
        self.assertEqual(sorted(roster), ['nrf52840dk'])
        self.assertEqual(roster['nrf52840dk']['arduino']['tests'], list(ht.ROLE_TESTS['device']))
        self.assertEqual(roster['nrf52840dk']['flasher']['name'], 'jlink')   # the base image is J-Link's

    def test_example_roster_names_a_profile_board(self):
        boards = json.loads((ROOT / 'test/hil/local.example.json').read_text())['boards']
        for b in boards:
            self.assertIn('span', ht.BUILD.BOARDS[b['name']])
            self.assertEqual(b['arduino']['tests'], list(ht.ROLE_TESTS['device']))


class Profile(unittest.TestCase):
    def test_tests_and_sketches_agree(self):
        sketches = {Path(s).name for s in ht.BUILD.HIL_SKETCHES['device']}
        self.assertEqual(set(ht.TESTS.values()), sketches)   # the engine's bind() checks the restore and the roles

    def test_engine_is_bound_to_this_profile(self):
        self.assertIs(E.P, ht)
        self.assertIs(E.B.P, ht.BUILD)


if __name__ == '__main__':
    unittest.main()
