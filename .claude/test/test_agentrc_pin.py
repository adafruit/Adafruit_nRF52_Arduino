#!/usr/bin/env python3
"""agentrc_pin.py as this core configures and selects it: test/hil/agentrc.json's values and the identity this core's
profile binds, and agentrc's conformance check, which check_build.py's direct run of this file would otherwise skip
(hathach/agentrc#585). Nothing here is fetched or reaches a rig."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy


class Config(unittest.TestCase):
    def test_this_cores_configuration(self):
        data = agentrc_pin.config()
        self.assertEqual({k: data[k] for k in ('cache_namespace', 'profile_api', 'features', 'extra_engines')},
                         {'cache_namespace': 'adafruit-nrf52', 'profile_api': 1,
                          'features': ['core', 'base', 'battery'], 'extra_engines': []})


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertIs(hil_test.BUILD.AGENTRC, ident)


class Conformance(unittest.TestCase):
    def test_bootstrap_is_the_canonical_copy(self):
        canonical = agentrc_pin.check() / agentrc_pin.SELF
        self.assertEqual(Path(agentrc_pin.__file__).read_bytes(), canonical.read_bytes())

    def test_conforms(self):
        self.assertEqual(agentrc_pin.conformance(), [])


if __name__ == '__main__':
    unittest.main()
