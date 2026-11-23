#!/usr/bin/env python3
"""agentrc_pin.py as this core configures and selects it: test/hil/agentrc.json's values, the canonical copy, and the
real engines loaded from the selected agentrc and from a rig tree staged from it. The bootstrap's common behaviour
(selection, validation, fetch, drift) is agentrc's and tested there; nothing here is fetched or reaches a rig."""
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy

CLEAN_ENV = {k: v for k, v in os.environ.items()
             if not k.startswith('GIT_') and k not in ('AGENTRC_DIR', 'CI', 'GITHUB_ACTIONS')}


class Config(unittest.TestCase):
    def test_this_cores_configuration(self):
        data = agentrc_pin.config()
        self.assertRegex(data['commit'], r'^[0-9a-f]{40}$')
        self.assertEqual({k: data[k] for k in ('cache_namespace', 'profile_api', 'features', 'extra_engines')},
                         {'cache_namespace': 'adafruit-nrf52', 'profile_api': 1,
                          'features': ['core', 'base', 'battery'], 'extra_engines': []})

    def test_the_rig_gets_the_bootstrap_and_its_configuration(self):
        for f in ('.claude/skills/hil-contract/scripts/agentrc_pin.py', 'test/hil/agentrc.json'):
            self.assertIn(f, hil_test.BUILD.CONTRACT_FILES)

    def test_the_copy_is_the_selected_canonical_bootstrap(self):
        canonical = agentrc_pin.check() / agentrc_pin.SELF
        self.assertEqual(Path(agentrc_pin.__file__).read_bytes(), canonical.read_bytes(), canonical)


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertLessEqual({*agentrc_pin.ENGINES, agentrc_pin.SELF, *agentrc_pin.FILES,
                              'skills/hil/scripts/hil_remote_core.py'}, set(ident['files']))
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertIs(hil_test.BUILD.AGENTRC, ident)

    def test_the_real_engines_load_from_their_staged_closure_alone(self):
        helper, _, ident = agentrc_pin._selection()
        # a fresh interpreter, so the staged modules never replace the engines the other tests load
        code = '''if True:
            import json, sys, sysconfig
            from pathlib import Path
            from unittest import mock
            sys.path.insert(0, sys.argv[1])
            import agentrc_pin
            before = set(sys.modules)
            with mock.patch.object(agentrc_pin, 'ROOT', Path(sys.argv[2])), \\
                    mock.patch.object(agentrc_pin, 'checkout', side_effect=AssertionError('a rig has no cache')), \\
                    mock.patch.object(agentrc_pin, 'installed', side_effect=AssertionError('a rig has no install')):
                staged = agentrc_pin.identity()
                engines = [agentrc_pin.load(e).__file__ for e in agentrc_pin.ENGINES]
            lib = tuple(sysconfig.get_paths()[k] for k in ('stdlib', 'platstdlib', 'purelib', 'platlib'))
            new = sorted(f for n in set(sys.modules) - before
                         if (f := getattr(sys.modules[n], '__file__', None)) and not f.startswith(lib))
            print(json.dumps({'engines': engines, 'modules': new, 'source': staged['source'],
                              'portable': {'commit': staged['commit'], 'sha256': staged['sha256']}}))
        '''
        with tempfile.TemporaryDirectory() as d:
            helper.stage(ident, d, name='agentrc')   # as arduino_remote.py stages a rig tree
            Path(d, 'manifest.json').write_text(json.dumps({'agentrc': {'commit': ident['commit'],
                                                                        'dirty': ident['dirty']}}))
            p = subprocess.run([sys.executable, '-I', '-c', code, str(Path(agentrc_pin.__file__).parent), d],
                               capture_output=True, text=True, env=CLEAN_ENV)
            self.assertEqual(p.returncode, 0, p.stderr)
            got = json.loads(p.stdout)
            staged = Path(d, 'agentrc').resolve()
            self.assertEqual(got['engines'], [str(Path(d, 'agentrc', e)) for e in agentrc_pin.ENGINES])
            self.assertEqual((got['source'], got['portable']), ('staged', helper.portable(ident)))
            for f in got['modules']:
                self.assertTrue(Path(f).resolve().is_relative_to(staged), f)


if __name__ == '__main__':
    unittest.main()
