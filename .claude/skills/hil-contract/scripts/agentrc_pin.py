#!/usr/bin/env python3
"""Where the build and HIL contracts load agentrc's shared scripts from (agentrc `arduino` and `hil` skills):

  staged     a rig tree hil_remote.py staged (manifest.json and agentrc/, no .git): its copy, the manifest's identity
  pin        CI (GITHUB_ACTIONS or CI true): test/hil/agentrc.json's {url, commit}, fetched into
             ${XDG_CACHE_HOME:-~/.cache}/adafruit-nrf52/agentrc-<commit>, outside this checkout
  installed  otherwise: ${AGENTRC_DIR:-$HOME/.claude}, the agentrc install every skill uses; never fetched

  agentrc_pin.py test     run .claude/test against the selected agentrc (CI fetches the pin first); its exit
  agentrc_pin.py fetch    CI only: fetch the pin into the cache
  agentrc_pin.py check    validate the selected agentrc as a record would bind it, fetching nothing; print its root
"""

import ast
import hashlib
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
PIN_FILE = ROOT / 'test' / 'hil' / 'agentrc.json'
FETCH_HINT = 'python3 .claude/skills/hil-contract/scripts/agentrc_pin.py fetch'
INSTALL_HINT = ('install agentrc with its arduino skill (python3 <agentrc checkout>/install.py install --skill), '
                'or set AGENTRC_DIR to such a checkout')
MANIFEST = 'manifest.json'
PROFILE_API = 1   # arduino_build.py's PROFILE_API: the engines' profile interface these profiles implement
# arduino_build.py's PROFILE_FEATURES entries beyond API 1 this core needs: core mode (PLATFORM profiles) and a rig
# board's base image (BOARDS base)
FEATURES = ('core', 'base')
# the agentrc engines the contracts run; their FILES closures are what a staged rig tree carries and a record binds
ENGINES = ('skills/arduino/scripts/arduino_build.py', 'skills/arduino/scripts/arduino_hil.py',
           'skills/arduino/scripts/arduino_remote.py')


class PinError(Exception):
    def __init__(self, msg, remedy=''):
        super().__init__(msg)
        self.remedy = remedy


def ci():
    """True in CI: GITHUB_ACTIONS or CI set true; an unset or empty one counts as false, any other value refused."""
    on = False
    for var in ('GITHUB_ACTIONS', 'CI'):
        value = os.environ.get(var, '').strip().lower()
        if value in ('1', 'true', 'yes'):
            on = True
        elif value not in ('', '0', 'false', 'no'):
            raise PinError(f'{var}={os.environ[var]!r} is neither true nor false')
    return on


def pin():
    """test/hil/agentrc.json's (url, commit), refusing an unreadable pin or one naming no url and full commit hash."""
    try:
        data = json.loads(PIN_FILE.read_text())
    except (OSError, ValueError, RecursionError) as e:
        raise PinError(f'{PIN_FILE} is not readable JSON ({e})') from None
    url, commit = (data.get('url'), data.get('commit')) if isinstance(data, dict) else (None, None)
    if (not isinstance(url, str) or not url.strip()
            or not isinstance(commit, str) or not re.fullmatch(r'[0-9a-f]{40}', commit)):
        raise PinError(f'{PIN_FILE} names no {{url, commit}} with a 40-hex commit ({data!r})')
    return url, commit


def cache_dir(commit):
    base = os.environ.get('XDG_CACHE_HOME') or os.path.expanduser('~/.cache')
    return Path(base) / 'adafruit-nrf52' / f'agentrc-{commit}'


def _git(root, *args):
    return subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True,
                          env={k: v for k, v in os.environ.items() if not k.startswith('GIT_')})


def problem(root, commit):
    """Why `root` is not a clean checkout of `commit`, '' when it is."""
    if not (Path(root) / '.git').exists():
        return f'{root} is not a git checkout'
    head = _git(root, 'rev-parse', 'HEAD')
    if head.returncode or head.stdout.strip() != commit:
        return f'{root} is at {head.stdout.strip() or "?"}, not the pin {commit}'
    status = _git(root, 'status', '--porcelain')
    if status.returncode or status.stdout.strip():
        return f'{root} has local changes: {status.stdout.strip()[:200]}'
    return ''


def fetch():
    if not ci():
        raise PinError('fetch runs in CI only: a dev PC loads the installed agentrc', INSTALL_HINT)
    url, commit = pin()
    dest = cache_dir(commit)
    if dest.exists():
        why = problem(dest, commit)
        if why:
            raise PinError(f'{dest} exists but {why}; remove it and fetch again')
        return dest
    tmp = dest.with_name(dest.name + f'.tmp-{os.getpid()}')
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    for args in (('init', '-q'), ('remote', 'add', 'origin', url), ('fetch', '-q', '--depth', '1', 'origin', commit),
                 ('checkout', '-q', '--detach', 'FETCH_HEAD')):
        p = _git(tmp, *args)
        if p.returncode:
            shutil.rmtree(tmp, ignore_errors=True)
            raise PinError(f'git {" ".join(args)}: {p.stderr.strip()}')
    try:
        tmp.rename(dest)
    except OSError as e:   # a concurrent first fetch renamed its checkout in first: checked below like ours
        shutil.rmtree(tmp, ignore_errors=True)
        if not dest.exists():
            raise PinError(f'{tmp} -> {dest}: {e}')
    why = problem(dest, commit)
    if why:
        raise PinError(why)
    return dest


def checkout():
    """The pinned checkout, refusing a missing, moved or modified one."""
    _, commit = pin()
    dest = cache_dir(commit)
    why = problem(dest, commit)
    if why:
        raise PinError(f'agentrc pin: {why}', FETCH_HINT)
    return dest


def literal(root, rel, name, default=None):
    """The literal a script assigns to `name` at module level, read without importing it."""
    for node in ast.parse((Path(root) / rel).read_text()).body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    return default


def files_of(root, rel):
    """A script's FILES literal (agentrc hil_load.py)."""
    return tuple(literal(root, rel, 'FILES', ()))


def closure(root, scripts):
    """Every agentrc-relative file `scripts` load or launch, themselves included."""
    out = set()
    for rel in scripts:
        out.add(rel)
        if (Path(root) / rel).is_file():
            out.update(files_of(root, rel))
    missing = sorted(r for r in out if not (Path(root) / r).is_file())
    if missing:
        raise PinError(f'agentrc at {root} lacks {missing}')
    return sorted(out)


def loader(root):
    """agentrc's hil_load, bootstrapped as its docstring prescribes."""
    path = Path(root) / 'skills' / 'hil' / 'scripts' / 'hil_load.py'
    spec = importlib.util.spec_from_file_location('agentrc_hil_load', path)
    if spec.name not in sys.modules:
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        sys.modules.setdefault(spec.name, mod)
    return sys.modules[spec.name].load(spec.name, spec.origin)


def staged():
    """A rig tree hil_remote.py staged: no .git, and both its manifest and its agentrc copy; one without the other is
    refused, never taken for a checkout."""
    if (ROOT / '.git').exists():
        return False
    marks = {MANIFEST: (ROOT / MANIFEST).is_file(), 'agentrc/': (ROOT / 'agentrc').is_dir()}
    if any(marks.values()) and not all(marks.values()):
        raise PinError(f'{ROOT} is a partial staged tree: {[m for m, ok in marks.items() if not ok]} missing')
    return all(marks.values())


def source():
    return 'staged' if staged() else 'pin' if ci() else 'installed'


def installed():
    """${AGENTRC_DIR:-$HOME/.claude}, absolute, holding the arduino skill: never fetched, never another location."""
    r = Path(os.environ.get('AGENTRC_DIR') or Path.home() / '.claude').absolute()
    if not (r / ENGINES[0]).is_file():
        raise PinError(f'{r} has no agentrc arduino skill ({ENGINES[0]})', INSTALL_HINT)
    return r


def checkout_of(r, files):
    """The one git checkout every closure file under r resolves into, at its own path there (a mix of installs,
    say skills/hil from another checkout than skills/arduino, is refused)."""
    top = _git((Path(r) / ENGINES[0]).resolve().parent, 'rev-parse', '--show-toplevel')
    if top.returncode:
        raise PinError(f'{(Path(r) / ENGINES[0]).resolve()} is in no git checkout: its commit is unknown', INSTALL_HINT)
    co = Path(top.stdout.strip()).resolve()
    stray = [f for f in files if (Path(r) / f).resolve() != (co / f).resolve()]
    if stray:
        raise PinError(f'{stray[:3]} under {r} resolve outside {co}: one agentrc checkout must hold them all',
                       INSTALL_HINT)
    return co


def selected():
    """(source, root, closure) of the selected agentrc, refusing a missing closure file or engines implementing
    another profile API than PROFILE_API before any is loaded."""
    src = source()
    r = {'staged': lambda: ROOT / 'agentrc', 'pin': checkout, 'installed': installed}[src]()
    files = closure(r, ENGINES)
    api = literal(r, ENGINES[0], 'PROFILE_API')
    if api != PROFILE_API:
        raise PinError(f'agentrc at {r} implements profile API {api}, this core needs {PROFILE_API}: use an '
                       f'agentrc that does, or move this core to the API it has', INSTALL_HINT)
    missing = [f for f in FEATURES if f not in (literal(r, ENGINES[0], 'PROFILE_FEATURES') or ())]
    if missing:
        raise PinError(f'agentrc at {r} lacks {missing} ({ENGINES[0]} PROFILE_FEATURES): use an agentrc with core '
                       'mode and base images', INSTALL_HINT)
    return src, r, files


def load(rel):
    """agentrc's `rel`, loaded through its hil_load from the selected agentrc."""
    return loader(selected()[1]).load_file(rel)


def identity():
    """{commit, root, files, sha256, dirty, source}: where agentrc is loaded from, the engines' closure
    (agentrc-relative) and each file's digest now, the commit it is from and whether the closure differs from it."""
    src, r, files = selected()
    if src == 'staged':
        try:
            manifest = json.loads((ROOT / MANIFEST).read_text())
        except (OSError, ValueError, RecursionError) as e:
            raise PinError(f'{ROOT / MANIFEST} is not readable JSON ({e}): stage it again with hil_remote.py')
        staged_from = manifest.get('agentrc') if isinstance(manifest, dict) else None
        if (not isinstance(staged_from, dict) or not isinstance(staged_from.get('commit'), str)
                or not re.fullmatch(r'[0-9a-f]{40}', staged_from['commit'])
                or not isinstance(staged_from.get('dirty'), bool)):
            raise PinError(f'{ROOT / MANIFEST} names no agentrc commit and dirty flag ({staged_from!r}): stage it '
                           'again with hil_remote.py')
        commit, dirty = staged_from['commit'], staged_from['dirty']
    elif src == 'pin':
        commit, dirty = pin()[1], False   # checkout() refused a moved or modified cache
    else:
        co = checkout_of(r, files)
        head, status = _git(co, 'rev-parse', 'HEAD'), _git(co, 'status', '--porcelain', '--', *files)
        if head.returncode or status.returncode:
            raise PinError(f'git in {co}: {(head.stderr or status.stderr).strip()}')
        commit, dirty = head.stdout.strip(), bool(status.stdout.strip())
    return {'commit': commit, 'root': r, 'files': files, 'dirty': dirty, 'source': src,
            'sha256': {f: hashlib.sha256((r / f).read_bytes()).hexdigest() for f in files}}


def test():
    """The contract tests against the selected agentrc, CI fetching the pin first; their exit."""
    if source() == 'pin':
        fetch()
    src, r, _ = selected()
    # a git hook exports GIT_DIR and GIT_INDEX_FILE: a test's git in a scratch repository would act on this one
    env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
    if src == 'installed':
        # the child runs in ROOT: a relative AGENTRC_DIR must still name the agentrc validated here
        env['AGENTRC_DIR'] = str(r)
    return subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', '.claude/test', '-p', 'test_*.py'],
                          cwd=ROOT, env=env).returncode


def main(argv):
    cmd = argv[0] if argv else ''
    try:
        if cmd == 'test':
            return test()
        if cmd == 'fetch':
            print(fetch())
        elif cmd == 'check':
            print(identity()['root'])
        else:
            print(__doc__, file=sys.stderr)
            return 2
    except PinError as e:
        print(f'error: {e}' + (f' ({e.remedy})' if e.remedy else ''), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
