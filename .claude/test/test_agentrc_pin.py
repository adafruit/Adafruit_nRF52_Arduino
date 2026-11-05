#!/usr/bin/env python3
"""agentrc_pin.py: which agentrc the contracts load (a rig's staged copy, CI's pin, else the installed one), the
identity a record binds, and the CI-only fetch. Git runs against throwaway repositories; nothing is fetched from the
network and nothing reaches a rig."""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy

HIL_LOAD = 'skills/hil/scripts/hil_load.py'
CLEAN_ENV = {k: v for k, v in os.environ.items()
             if not k.startswith('GIT_') and k not in ('AGENTRC_DIR', 'CI', 'GITHUB_ACTIONS')}


def git(root, *a):
    return subprocess.run(['git', '-C', str(root), *a], check=True, capture_output=True, text=True).stdout.strip()


def commit_all(repo, msg):
    git(repo, 'add', '-A')
    git(repo, '-c', 'user.name=t', '-c', 'user.email=t@t', '-c', 'commit.gpgsign=false', 'commit', '-qm', msg)
    return git(repo, 'rev-parse', 'HEAD')


def fake_agentrc(repo, hil_load=''):
    """A git checkout of stand-in engines whose closure is themselves and hil_load; its HEAD."""
    repo.mkdir(parents=True)
    git(repo, 'init', '-q')
    for e in agentrc_pin.ENGINES:
        (repo / e).parent.mkdir(parents=True, exist_ok=True)
        (repo / e).write_text(f'FILES = ({HIL_LOAD!r},)\nPROFILE_API = {agentrc_pin.PROFILE_API}\n'
                              f'PROFILE_FEATURES = {agentrc_pin.FEATURES!r}\n')
    (repo / HIL_LOAD).parent.mkdir(parents=True, exist_ok=True)
    (repo / HIL_LOAD).write_text(hil_load)
    (repo / 'README.md').write_text('not in the closure\n')
    return commit_all(repo, 'agentrc')


class Env(unittest.TestCase):
    """No CI, AGENTRC_DIR or GIT_* variables, and a scratch directory."""

    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.tmp = Path(self.stack.enter_context(tempfile.TemporaryDirectory())).resolve()
        self.stack.enter_context(mock.patch.dict(os.environ, CLEAN_ENV, clear=True))

    def env(self, **values):
        self.stack.enter_context(mock.patch.dict(os.environ, values))


class Selection(Env):
    def test_ci_reads_github_actions_and_ci_and_refuses_anything_else(self):
        self.assertFalse(agentrc_pin.ci())
        for var in ('GITHUB_ACTIONS', 'CI'):
            for value, on in (('true', True), ('1', True), ('YES', True), ('false', False), ('0', False), ('', False)):
                with self.subTest(var=var, value=value), mock.patch.dict(os.environ, {var: value}):
                    self.assertEqual(agentrc_pin.ci(), on)
        with mock.patch.dict(os.environ, {'CI': 'maybe'}), \
                self.assertRaisesRegex(agentrc_pin.PinError, "CI='maybe' is neither true nor false"):
            agentrc_pin.ci()

    def test_a_tree_is_staged_only_with_its_manifest_and_agentrc_copy(self):
        with mock.patch.object(agentrc_pin, 'ROOT', self.tmp):
            self.assertFalse(agentrc_pin.staged())   # a plain copy of the checkout
            (self.tmp / 'agentrc').mkdir()
            with self.assertRaisesRegex(agentrc_pin.PinError, r"partial staged tree: \['manifest.json'\] missing"):
                agentrc_pin.staged()
            (self.tmp / 'manifest.json').write_text('{}')
            self.assertTrue(agentrc_pin.staged())
            (self.tmp / '.git').mkdir()
            self.assertFalse(agentrc_pin.staged())   # a checkout is never a rig tree

    def test_the_source_is_staged_then_ci_then_installed(self):
        with mock.patch.object(agentrc_pin, 'ROOT', self.tmp):
            self.assertEqual(agentrc_pin.source(), 'installed')
            self.env(GITHUB_ACTIONS='true')
            self.assertEqual(agentrc_pin.source(), 'pin')
            (self.tmp / 'agentrc').mkdir()
            (self.tmp / 'manifest.json').write_text('{}')
            self.assertEqual(agentrc_pin.source(), 'staged')

    def test_installed_is_agentrc_dir_else_home_claude(self):
        repo, home = self.tmp / 'agentrc', self.tmp / 'home'
        fake_agentrc(repo)
        (home / '.claude').mkdir(parents=True)
        (home / '.claude' / 'skills').symlink_to(repo / 'skills')
        with mock.patch.dict(os.environ, {'AGENTRC_DIR': str(repo), 'HOME': str(home)}):
            self.assertEqual(agentrc_pin.installed(), repo)
        for unset in ({'HOME': str(home)}, {'HOME': str(home), 'AGENTRC_DIR': ''}):   # as ${AGENTRC_DIR:-$HOME/.claude}
            with self.subTest(env=unset), mock.patch.dict(os.environ, unset):
                self.assertEqual(agentrc_pin.installed(), home / '.claude')

    def test_an_install_without_the_arduino_skill_names_the_install(self):
        self.env(AGENTRC_DIR=str(self.tmp))
        with self.assertRaisesRegex(agentrc_pin.PinError, 'has no agentrc arduino skill') as e:
            agentrc_pin.installed()
        self.assertEqual(e.exception.remedy, agentrc_pin.INSTALL_HINT)

    def test_fetch_runs_in_ci_only(self):
        with mock.patch.object(agentrc_pin, 'pin', side_effect=AssertionError('read the pin')):
            with self.assertRaisesRegex(agentrc_pin.PinError, 'fetch runs in CI only'):
                agentrc_pin.fetch()
            with contextlib.redirect_stderr(io.StringIO()) as err:
                self.assertEqual(agentrc_pin.main(['fetch']), 1)
        self.assertIn('fetch runs in CI only', err.getvalue())

    def test_ci_ignores_an_installed_agentrc(self):
        repo, pinned = self.tmp / 'agentrc', self.tmp / 'pinned'
        fake_agentrc(repo)
        fake_agentrc(pinned)
        self.env(AGENTRC_DIR=str(repo), CI='true')
        with mock.patch.object(agentrc_pin, 'ROOT', self.tmp / 'lib'), \
                mock.patch.object(agentrc_pin, 'checkout', return_value=pinned):
            self.assertEqual(agentrc_pin.selected()[:2], ('pin', pinned))


class Installed(Env):
    """$HOME/.claude/skills/{arduino,hil} linked into a checkout, as agentrc's install.py links them."""

    def setUp(self):
        super().setUp()
        self.repo = self.tmp / 'agentrc'
        self.head = fake_agentrc(self.repo)
        self.claude = self.tmp / 'home' / '.claude'
        (self.claude / 'skills').mkdir(parents=True)
        for skill in ('arduino', 'hil'):
            (self.claude / 'skills' / skill).symlink_to(self.repo / 'skills' / skill)
        self.env(HOME=str(self.tmp / 'home'))
        self.stack.enter_context(mock.patch.object(agentrc_pin, 'ROOT', self.tmp / 'lib'))

    def test_identity_through_the_skill_links_is_the_checkouts_head(self):
        ident = agentrc_pin.identity()
        self.assertEqual((ident['commit'], ident['root'], ident['dirty'], ident['source']),
                         (self.head, self.claude, False, 'installed'))
        self.assertEqual(ident['files'], sorted([*agentrc_pin.ENGINES, HIL_LOAD]))
        self.assertEqual(set(ident['sha256']), set(ident['files']))

    def test_dirty_is_the_closures_alone(self):
        (self.repo / 'README.md').write_text('edited\n')
        self.assertFalse(agentrc_pin.identity()['dirty'])
        (self.repo / HIL_LOAD).write_text('# edited\n')
        self.assertEqual(agentrc_pin.identity()['commit'], self.head)
        self.assertTrue(agentrc_pin.identity()['dirty'])

    def test_skills_linked_from_two_checkouts_are_refused_by_identity_and_check(self):
        other = self.tmp / 'other'
        fake_agentrc(other)
        (self.claude / 'skills' / 'hil').unlink()
        (self.claude / 'skills' / 'hil').symlink_to(other / 'skills' / 'hil')
        with self.assertRaisesRegex(agentrc_pin.PinError, 'one agentrc checkout must hold them all'):
            agentrc_pin.identity()
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(agentrc_pin.main(['check']), 1)
        self.assertIn('one agentrc checkout must hold them all', err.getvalue())

    def test_a_copy_outside_git_is_refused(self):
        copy = self.tmp / 'copy'
        shutil.copytree(self.repo, copy, ignore=shutil.ignore_patterns('.git'))
        self.env(AGENTRC_DIR=str(copy))
        with self.assertRaisesRegex(agentrc_pin.PinError, 'in no git checkout'):
            agentrc_pin.identity()

    def test_engines_of_another_profile_api_are_refused_before_loading(self):
        (self.repo / agentrc_pin.ENGINES[0]).write_text(f'FILES = ({HIL_LOAD!r},)\nPROFILE_API = 99\n')
        with mock.patch.object(agentrc_pin, 'loader', side_effect=AssertionError('loaded')), \
                self.assertRaisesRegex(agentrc_pin.PinError, 'implements profile API'):
            agentrc_pin.load(agentrc_pin.ENGINES[0])

    def test_engines_without_core_mode_or_base_images_are_refused_before_loading(self):
        for features in ((), ('core',)):
            with self.subTest(features=features):
                (self.repo / agentrc_pin.ENGINES[0]).write_text(
                    f'FILES = ({HIL_LOAD!r},)\nPROFILE_API = 1\nPROFILE_FEATURES = {features!r}\n')
                with mock.patch.object(agentrc_pin, 'loader', side_effect=AssertionError('loaded')), \
                        self.assertRaisesRegex(agentrc_pin.PinError, 'lacks .*base'):
                    agentrc_pin.load(agentrc_pin.ENGINES[0])

    def test_check_and_test_never_fetch(self):
        with mock.patch.object(agentrc_pin, 'fetch', side_effect=AssertionError('fetched')):
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(agentrc_pin.main(['check']), 0)
            self.assertEqual(out.getvalue().strip(), str(self.claude))
            with mock.patch.object(agentrc_pin.subprocess, 'run', return_value=mock.Mock(returncode=5)) as run:
                self.assertEqual(agentrc_pin.test(), 5)
        self.assertIn('discover', run.call_args.args[0])

    def test_test_hands_the_child_the_agentrc_a_relative_dir_named_here(self):
        self.addCleanup(os.chdir, os.getcwd())
        os.chdir(self.tmp)
        self.env(AGENTRC_DIR=str(self.claude.relative_to(self.tmp)))
        self.assertEqual(agentrc_pin.identity()['root'], self.claude)
        with mock.patch.object(agentrc_pin.subprocess, 'run', return_value=mock.Mock(returncode=0)) as run:
            self.assertEqual(agentrc_pin.test(), 0)
        self.assertEqual(run.call_args.kwargs['cwd'], agentrc_pin.ROOT)
        self.assertEqual(run.call_args.kwargs['env']['AGENTRC_DIR'], str(self.claude))

    def test_a_hook_run_tests_git_init_leaves_the_hooks_repository_alone(self):
        primary, target = self.repo, self.tmp / 'target'
        git(primary, 'worktree', 'add', '-q', '--detach', str(self.tmp / 'wt'))
        probe = agentrc_pin.ROOT / '.claude' / 'test' / 'test_probe.py'
        probe.parent.mkdir(parents=True)
        probe.write_text(f'''if True:
            import subprocess, unittest

            class Probe(unittest.TestCase):
                def test_init(self):
                    subprocess.run(['git', 'init', '-q', {str(target)!r}], check=True)
            ''')
        config = (primary / '.git' / 'config').read_bytes()
        hook_git_dir = primary / '.git' / 'worktrees' / 'wt'   # what git exports to a hook committing in wt
        self.env(GIT_DIR=str(hook_git_dir), GIT_INDEX_FILE=str(hook_git_dir / 'index'))
        run, done = subprocess.run, []
        with mock.patch.object(agentrc_pin.subprocess, 'run',
                               lambda *a, **k: done.append(run(*a, capture_output=True, text=True, **k)) or done[-1]):
            self.assertEqual(agentrc_pin.test(), 0, done[-1].stdout + done[-1].stderr if done else None)
        self.assertTrue((target / '.git').is_dir())
        self.assertEqual((primary / '.git' / 'config').read_bytes(), config)


class Runner(Env):
    """test()'s child environment, whichever agentrc is selected."""

    def test_the_child_gets_no_git_variable_from_any_source(self):
        self.env(GIT_DIR='x', GIT_INDEX_FILE='x', GIT_CONFIG_COUNT='1')
        before = dict(os.environ)
        for src in ('installed', 'pin', 'staged'):
            with self.subTest(src=src), mock.patch.object(agentrc_pin, 'selected', return_value=(src, self.tmp, [])), \
                    mock.patch.object(agentrc_pin.subprocess, 'run', return_value=mock.Mock(returncode=0)) as run:
                self.assertEqual(agentrc_pin.test(), 0)
                env = run.call_args.kwargs['env']
                self.assertEqual([k for k in env if k.startswith('GIT_')], [])
                self.assertEqual(env['PATH'], os.environ['PATH'])
                self.assertEqual(env.get('AGENTRC_DIR'), str(self.tmp) if src == 'installed' else None)
        self.assertEqual(dict(os.environ), before)


class Staged(Env):
    def stage(self, manifest, hil_load=''):
        """A rig tree: stand-in engines copied under agentrc/, and `manifest`."""
        fake_agentrc(self.tmp / 'src', hil_load)
        shutil.copytree(self.tmp / 'src' / 'skills', self.tmp / 'agentrc' / 'skills')
        (self.tmp / 'manifest.json').write_text(json.dumps(manifest))
        self.stack.enter_context(mock.patch.object(agentrc_pin, 'ROOT', self.tmp))
        self.stack.enter_context(mock.patch.object(agentrc_pin, 'checkout',
                                                   side_effect=AssertionError('a rig has no cache')))
        self.stack.enter_context(mock.patch.object(agentrc_pin, 'installed',
                                                   side_effect=AssertionError('a rig has no install')))

    def test_identity_is_the_manifests_even_with_an_install_and_ci(self):
        self.stage({'agentrc': {'commit': 'c' * 40, 'dirty': True, 'source': 'installed'}})
        self.env(AGENTRC_DIR=str(self.tmp / 'src'), CI='true')
        ident = agentrc_pin.identity()
        self.assertEqual((ident['commit'], ident['dirty'], ident['source'], ident['root']),
                         ('c' * 40, True, 'staged', self.tmp / 'agentrc'))

    def test_a_manifest_without_a_valid_agentrc_identity_is_refused(self):
        self.stage({})
        for bad in (None, {}, {'commit': 'not-a-commit', 'dirty': False}, {'commit': '', 'dirty': False},
                    {'commit': 'c' * 40}, {'commit': 'c' * 40, 'dirty': 'false'}, {'commit': int('1' * 40), 'dirty': False}):
            with self.subTest(agentrc=bad):
                (self.tmp / 'manifest.json').write_text(json.dumps({'agentrc': bad}))
                with self.assertRaisesRegex(agentrc_pin.PinError, 'names no agentrc commit and dirty flag'):
                    agentrc_pin.identity()
        for bad in ('[]', '"agentrc"'):   # JSON, but not an object
            with self.subTest(manifest=bad):
                (self.tmp / 'manifest.json').write_text(bad)
                with self.assertRaisesRegex(agentrc_pin.PinError, 'names no agentrc commit and dirty flag'):
                    agentrc_pin.identity()
        for bad in (b'{not json', b'\xff', b'{"agentrc": ' + b'[' * 200000):
            with self.subTest(manifest=bad):
                (self.tmp / 'manifest.json').write_bytes(bad)
                with self.assertRaisesRegex(agentrc_pin.PinError, 'is not readable JSON.*stage it again'):
                    agentrc_pin.identity()

    def test_a_staged_tree_without_the_engines_is_refused(self):
        self.stage({'agentrc': {'commit': 'c' * 40, 'dirty': False}})
        os.remove(self.tmp / 'agentrc' / HIL_LOAD)
        with self.assertRaisesRegex(agentrc_pin.PinError, 'lacks'):
            agentrc_pin.load(agentrc_pin.ENGINES[0])

    def test_a_staged_tree_loads_its_own_copy(self):
        self.stage({'agentrc': {'commit': 'c' * 40, 'dirty': False}}, hil_load=(
            'class _L:\n    def load_file(self, rel):\n        return "STAGED:" + rel\n\n\n'
            'def load(name, origin):\n    return _L()\n'))
        with mock.patch.dict(sys.modules):
            sys.modules.pop('agentrc_hil_load', None)
            self.assertEqual(agentrc_pin.load(agentrc_pin.ENGINES[0]), 'STAGED:' + agentrc_pin.ENGINES[0])


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertLessEqual({*agentrc_pin.ENGINES, HIL_LOAD, 'skills/hil/scripts/hil_remote_core.py'},
                             set(ident['files']))
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertEqual(hil_test.BUILD.AGENTRC, ident)

    def test_the_real_engines_load_from_their_staged_closure_alone(self):
        # a fresh interpreter, so the staged modules never replace the engines the other tests load
        code = '''if True:
            import json, shutil, sys, sysconfig
            from pathlib import Path
            from unittest import mock
            sys.path.insert(0, sys.argv[1])
            import agentrc_pin
            ident = agentrc_pin.identity()
            src, d = ident['root'], Path(sys.argv[2])
            for f in ident['files']:
                (d / 'agentrc' / f).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src / f, d / 'agentrc' / f)
            (d / 'manifest.json').write_text(json.dumps({'agentrc': {'commit': ident['commit'], 'dirty': ident['dirty']}}))
            before = set(sys.modules)
            with mock.patch.object(agentrc_pin, 'ROOT', d), \\
                    mock.patch.object(agentrc_pin, 'checkout', side_effect=AssertionError('a rig has no cache')), \\
                    mock.patch.object(agentrc_pin, 'installed', side_effect=AssertionError('a rig has no install')):
                staged = agentrc_pin.identity()
                engines = [agentrc_pin.load(e).__file__ for e in agentrc_pin.ENGINES]
            lib = tuple(sysconfig.get_paths()[k] for k in ('stdlib', 'platstdlib', 'purelib', 'platlib'))
            new = sorted(f for n in set(sys.modules) - before
                         if (f := getattr(sys.modules[n], '__file__', None)) and not f.startswith(lib))
            print(json.dumps({'engines': engines, 'modules': new, 'source': staged['source'],
                              'same': [staged[k] == ident[k] for k in ('commit', 'files', 'sha256')]}))
        '''
        with tempfile.TemporaryDirectory() as d:
            p = subprocess.run([sys.executable, '-I', '-c', code, str(Path(agentrc_pin.__file__).parent), d],
                               capture_output=True, text=True)
            self.assertEqual(p.returncode, 0, p.stderr)
            got = json.loads(p.stdout)
            staged = Path(d, 'agentrc').resolve()
            self.assertEqual(got['engines'], [str(Path(d, 'agentrc', e)) for e in agentrc_pin.ENGINES])
            self.assertEqual((got['source'], got['same']), ('staged', [True, True, True]))
            for f in got['modules']:
                self.assertTrue(Path(f).resolve().is_relative_to(staged), f)

    def test_closure_reads_files_without_importing(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / 'a.py').write_text("import nothing_here\nFILES = ('b.py',)\n")
            (Path(d) / 'b.py').write_text('')
            self.assertEqual(agentrc_pin.closure(d, ['a.py']), ['a.py', 'b.py'])
            (Path(d) / 'b.py').unlink()
            with self.assertRaisesRegex(agentrc_pin.PinError, r"lacks \['b.py'\]"):
                agentrc_pin.closure(d, ['a.py'])


class PinFile(Env):
    def test_the_repository_pin_is_valid(self):
        url, commit = agentrc_pin.pin()
        self.assertTrue(url)
        self.assertRegex(commit, r'^[0-9a-f]{40}$')

    def test_a_missing_or_malformed_pin_is_refused_by_pin_and_check(self):
        good = {'url': 'https://example.invalid/agentrc.git', 'commit': '0' * 40}
        cases = {'missing': None, 'not json': '{', 'a list': '[]', 'no commit': {'url': good['url']},
                 'no url': {'commit': good['commit']}, 'a blank url': {**good, 'url': '   '},
                 'a short commit': {**good, 'commit': 'abc123'},
                 'a non-hex commit': {**good, 'commit': 'g' * 40}, 'a numeric commit': {**good, 'commit': 1}}
        self.env(CI='true')
        for name, content in cases.items():
            pin_file = self.tmp / f'{name}.json'
            if content is not None:
                pin_file.write_text(content if isinstance(content, str) else json.dumps(content))
            with self.subTest(name), mock.patch.object(agentrc_pin, 'PIN_FILE', pin_file), \
                    mock.patch.object(agentrc_pin, 'ROOT', self.tmp / 'lib'):
                with self.assertRaisesRegex(agentrc_pin.PinError, re.escape(str(pin_file))):
                    agentrc_pin.pin()
                with contextlib.redirect_stderr(io.StringIO()) as err, contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(agentrc_pin.main(['check']), 1)
                self.assertIn(f'error: {pin_file}', err.getvalue())


class PinFetch(Env):
    """agentrc_pin.fetch in CI, against a local repository standing in for the pinned agentrc."""

    def setUp(self):
        super().setUp()
        self.env(XDG_CACHE_HOME=str(self.tmp), CI='true')
        repo = self.tmp / 'upstream'
        repo.mkdir()
        git(repo, 'init', '-q')
        (repo / 'f').write_text('x')
        self.commit = commit_all(repo, 'pin')
        self.stack.enter_context(mock.patch.object(agentrc_pin, 'pin', lambda: (str(repo), self.commit)))
        self.dest = agentrc_pin.cache_dir(self.commit)

    def race(self, winner):
        """fetch() with `winner` put at the cache dir by a concurrent fetch while this one checks out."""
        real = agentrc_pin._git

        def git_then_win(root, *args):
            p = real(root, *args)
            if args[0] == 'checkout':
                winner()
            return p
        with mock.patch.object(agentrc_pin, '_git', git_then_win):
            return agentrc_pin.fetch()

    def test_a_concurrent_first_fetch_reuses_the_winner(self):
        won = self.tmp / 'won'
        shutil.copytree(agentrc_pin.fetch(), won, symlinks=True)
        shutil.rmtree(self.dest)
        self.assertEqual(self.race(lambda: shutil.copytree(won, self.dest, symlinks=True)), self.dest)
        self.assertEqual(agentrc_pin.problem(self.dest, self.commit), '')
        self.assertEqual([p.name for p in self.dest.parent.iterdir()], [self.dest.name])

    def test_a_bad_winner_is_refused(self):
        def bad():
            self.dest.mkdir(parents=True)
            (self.dest / 'junk').write_text('x')
        with self.assertRaisesRegex(agentrc_pin.PinError, 'not a git checkout'):
            self.race(bad)

    def test_a_moved_or_modified_cache_is_refused_naming_the_fetch(self):
        agentrc_pin.fetch()
        self.assertEqual(agentrc_pin.checkout(), self.dest)
        (self.dest / 'f').write_text('edited')
        with self.assertRaisesRegex(agentrc_pin.PinError, 'local changes') as e:
            agentrc_pin.checkout()
        self.assertEqual(e.exception.remedy, agentrc_pin.FETCH_HINT)
        self.assertIn('is not a git checkout', agentrc_pin.problem(self.dest.parent, self.commit))
        self.assertIn(f'not the pin {"0" * 40}', agentrc_pin.problem(self.dest, '0' * 40))


if __name__ == '__main__':
    unittest.main()
