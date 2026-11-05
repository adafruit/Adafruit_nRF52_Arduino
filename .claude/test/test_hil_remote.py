#!/usr/bin/env python3
"""hil_remote.py, a wrapper running agentrc's arduino_remote.py (the agentrc agentrc_pin.py selects) with hil_test.py's
profile. The staging and session mechanics are agentrc's and tested there; nothing here reaches a rig. Stubs prove the
plumbing only; the transport is verified by a real run."""
import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_remote  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy


class Wrapper(unittest.TestCase):
    def test_runs_the_remote_engine_with_the_hil_profile(self):
        remote = mock.Mock()
        remote.run.return_value = 3
        with mock.patch.object(agentrc_pin, 'load', return_value=remote) as load:
            self.assertEqual(hil_remote.main(['wait', 'r1']), 3)
        load.assert_called_once_with('skills/arduino/scripts/arduino_remote.py')
        remote.run.assert_called_once_with(hil_test, ['wait', 'r1'])

    def test_an_unavailable_agentrc_exits_2_naming_why_and_the_remedy(self):
        with mock.patch.object(hil_test, 'engine', side_effect=agentrc_pin.PinError('agentrc gone', 'install it')), \
                contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(hil_remote.main(['wait', 'r1']), 2)
        self.assertEqual(err.getvalue(), 'error: agentrc gone (install it)\n')

    def test_the_real_engine_parses_the_profiles_tests(self):
        with contextlib.redirect_stderr(io.StringIO()) as err, self.assertRaises(SystemExit) as e:
            hil_remote.main(['run', '--run-id', 'r1', '-t', 'nope'])
        self.assertEqual(e.exception.code, 2)
        # newer argparse quotes each choice
        self.assertIn(', '.join(sorted(hil_test.TESTS)), err.getvalue().replace("'", ''))

    def test_the_rig_gets_every_contract_file(self):
        for f in hil_test.BUILD.CONTRACT_FILES:
            self.assertTrue((ROOT / f).is_file(), f)
        self.assertIn('.claude/skills/hil-contract/scripts/hil_remote.py', hil_test.BUILD.CONTRACT_FILES)
        self.assertIn('.claude/skills/hil-contract/scripts/agentrc_pin.py', hil_test.BUILD.CONTRACT_FILES)

    def test_remote_names_are_this_cores(self):
        self.assertTrue((ROOT / hil_test.HIL).is_file())
        self.assertTrue((ROOT / hil_test.REMOTE_ROSTER).is_file())


if __name__ == '__main__':
    unittest.main()
