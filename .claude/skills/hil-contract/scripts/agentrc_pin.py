#!/usr/bin/env python3
"""The Arduino contracts' bootstrap, copied verbatim to .claude/skills/hil-contract/scripts/agentrc_pin.py.
Configure test/hil/agentrc.json (all fields required):

  url              agentrc's fetch URL
  commit           full lowercase 40-hex CI pin
  cache_namespace  plain directory name below ${XDG_CACHE_HOME:-~/.cache}
  profile_api      positive integer, arduino_build.py's PROFILE_API
  features         required PROFILE_FEATURES strings (empty for a plain library)
  extra_engines    additional agentrc-relative scripts, e.g. skills/hil/scripts/gh_pr_comment.py

Selection, once per process: a staged tree's agentrc/ (outer manifest.json, no .git), else CI's pin
(GITHUB_ACTIONS or CI true), else "${AGENTRC_DIR:-$HOME/.claude}", never fetched locally. Identity, closure,
loading and staging use agentrc_source and hil_load; drift is checked on every load and record binding.
A staged tree needs agentrc/manifest.json as well as the outer manifest. Older trees must be staged again.

  test   compare this copy with the selected canonical file, then run .claude/test; CI fetches first
  fetch  CI only: fetch the pin into <cache_namespace>/agentrc-<commit>, outside this checkout
  check  validate the selected identity without fetching, print its checkout root
"""

import ast
import functools
import importlib.util
import json
import os
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
PIN_FILE = ROOT / "test" / "hil" / "agentrc.json"
FETCH_HINT = "python3 .claude/skills/hil-contract/scripts/agentrc_pin.py fetch"
INSTALL_HINT = (
    "install agentrc with its arduino skill (python3 <agentrc checkout>/install.py install --skill), "
    "or set AGENTRC_DIR to such a checkout"
)
STAGE_HINT = "stage the tree again with hil_remote.py"
MANIFEST = "manifest.json"
SELF = "skills/arduino/scripts/agentrc_pin.py"
SOURCE = "skills/hil/scripts/agentrc_source.py"
LOADER = "skills/hil/scripts/hil_load.py"
SOURCE_API = 1
FILES = ("skills/hil/scripts/agentrc_source.py", "skills/hil/scripts/hil_load.py")
ENGINES = (
    "skills/arduino/scripts/arduino_build.py",
    "skills/arduino/scripts/arduino_hil.py",
    "skills/arduino/scripts/arduino_remote.py",
)


class PinError(Exception):
    def __init__(self, msg, remedy=""):  # noqa: B042
        super().__init__(msg)
        self.remedy = remedy

    def with_remedy(self):
        return f"{self} ({self.remedy})" if self.remedy else str(self)


def ci():
    """GITHUB_ACTIONS or CI true; empty counts as false, any non-boolean value is refused."""
    on = False
    for var in ("GITHUB_ACTIONS", "CI"):
        value = os.environ.get(var, "").strip().lower()
        if value in ("1", "true", "yes"):
            on = True
        elif value not in ("", "0", "false", "no"):
            raise PinError(f"{var}={os.environ[var]!r} is neither true nor false")
    return on


@functools.cache
def config():
    """Validated consumer configuration; no profile import (the profile imports this bootstrap)."""
    try:
        data = json.loads(PIN_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError, RecursionError) as e:
        raise PinError(f"{PIN_FILE} is not readable JSON ({e})") from None
    if not isinstance(data, dict):
        raise PinError(f"{PIN_FILE}: configuration must be a JSON object")
    rules = {
        "url": lambda v: isinstance(v, str) and bool(v.strip()),
        "commit": lambda v: isinstance(v, str) and bool(re.fullmatch(r"[0-9a-f]{40}", v)),
        "cache_namespace": lambda v: isinstance(v, str) and bool(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", v)),
        "profile_api": lambda v: type(v) is int and v > 0,
        "features": lambda v: isinstance(v, list) and all(isinstance(f, str) and bool(f.strip()) for f in v),
        "extra_engines": lambda v: isinstance(v, list) and all(isinstance(f, str) and bool(f) for f in v),
    }
    for field, valid in rules.items():
        if not valid(data.get(field)):
            raise PinError(f"{PIN_FILE}: invalid {field}: {data.get(field)!r}")
    return data


def pin():
    data = config()
    return data["url"], data["commit"]


def cache_dir(commit):
    base = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return Path(base) / config()["cache_namespace"] / f"agentrc-{commit}"


def _git(root, *args):
    return subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        text=True,
        env={k: v for k, v in os.environ.items() if not k.startswith("GIT_")},
    )


def problem(root, commit):
    """Check a fetch before executing any of it: clean checkout at the full pin, or a refusal reason."""
    if not (Path(root) / ".git").exists():
        return f"{root} is not a git checkout"
    head = _git(root, "rev-parse", "HEAD")
    if head.returncode or head.stdout.strip() != commit:
        return f'{root} is at {head.stdout.strip() or "?"}, not the pin {commit}'
    status = _git(root, "status", "--porcelain")
    if status.returncode or status.stdout.strip():
        return f"{root} has local changes: {status.stdout.strip()[:200]}"
    return ""


def _remove(tree):
    """Best-effort rmtree that also removes git's read-only object files, which Windows refuses to unlink."""

    def writable(func, path, _):
        if func is not os.unlink or os.path.islink(path):  # chmod follows links out of the tree
            return
        try:
            os.chmod(path, stat.S_IWRITE)
            func(path)
        except OSError:
            pass

    shutil.rmtree(tree, onexc=writable)


def fetch():
    if not ci():
        raise PinError("fetch runs in CI only: a dev PC loads the installed agentrc", INSTALL_HINT)
    url, commit = pin()
    dest = cache_dir(commit)
    if dest.exists():
        why = problem(dest, commit)
        if why:
            raise PinError(f"{dest} exists but {why}; remove it and fetch again")
        return dest
    tmp = dest.with_name(dest.name + f".tmp-{os.getpid()}")
    _remove(tmp)
    tmp.mkdir(parents=True)
    for args in (
        ("init", "-q"),
        ("remote", "add", "origin", url),
        ("fetch", "-q", "--depth", "1", "origin", commit),
        ("checkout", "-q", "--detach", "FETCH_HEAD"),
    ):
        p = _git(tmp, *args)
        if p.returncode:
            _remove(tmp)
            raise PinError(f'git {" ".join(args)}: {p.stderr.strip()}')
    try:
        tmp.rename(dest)
    except OSError as e:  # A concurrent first fetch may have published its checkout first.
        _remove(tmp)
        if not dest.exists():
            raise PinError(f"{tmp} -> {dest}: {e}") from None
    why = problem(dest, commit)
    if why:
        raise PinError(why)
    return dest


def checkout():
    _, commit = pin()
    dest = cache_dir(commit)
    why = problem(dest, commit)
    if why:
        raise PinError(f"agentrc pin: {why}", FETCH_HINT)
    return dest


def literals(root, rel, *names):
    """Read selected module-level literals in one parse, without executing the script."""
    values = {}
    try:
        for node in ast.parse((Path(root) / rel).read_text(encoding="utf-8")).body:
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if getattr(target, "id", None) in names:
                        values[target.id] = ast.literal_eval(node.value)
    except (OSError, ValueError, SyntaxError, RecursionError) as e:
        raise PinError(f"agentrc {rel} at {root}: {names} is unreadable ({e})") from None
    return values


def loader(root):
    """hil_load's bootstrap; its resolved-path check refuses a loader from another checkout in this process."""
    spec = importlib.util.spec_from_file_location("agentrc_hil_load", Path(root) / LOADER)
    if spec.name not in sys.modules:
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        sys.modules.setdefault(spec.name, mod)
    return sys.modules[spec.name].load(spec.name, spec.origin)


def staged():
    """A rig tree has no .git and both outer manifest and agentrc/; partial staging never falls back."""
    if (ROOT / ".git").exists():
        return False
    marks = {MANIFEST: (ROOT / MANIFEST).is_file(), "agentrc/": (ROOT / "agentrc").is_dir()}
    if any(os.path.lexists(ROOT / name) for name in (MANIFEST, "agentrc")) and not all(marks.values()):
        raise PinError(
            f"{ROOT} is a partial staged tree: {[m for m, ok in marks.items() if not ok]} missing", STAGE_HINT
        )
    return all(marks.values())


def source():
    return "staged" if staged() else "pin" if ci() else "installed"


def installed():
    r = Path(os.environ.get("AGENTRC_DIR") or Path.home() / ".claude").absolute()
    if not (r / ENGINES[0]).is_file():
        raise PinError(f"{r} has no agentrc arduino skill ({ENGINES[0]})", INSTALL_HINT)
    return r


@functools.cache
def _selection():
    """The source module, real-checkout loader and frozen identity; cache only successful discovery."""
    data = config()
    src = source()
    root, hint = {
        "staged": (lambda: ROOT / "agentrc", STAGE_HINT),
        "pin": (checkout, FETCH_HINT),
        "installed": (installed, INSTALL_HINT),
    }[src]
    try:
        r = root()
        api = literals(r, SOURCE, "API").get("API")
        if type(api) is not int or api != SOURCE_API:
            raise PinError(f"agentrc at {r} implements source API {api}, this checkout needs {SOURCE_API}")
        real = (r / SOURCE).resolve().parents[3]
        hil_load = loader(real)
        helper = hil_load.load_file(SOURCE)
        try:
            ident = helper.frozen(
                src, r, (*ENGINES, SELF, *data["extra_engines"]), data["commit"] if src == "pin" else None
            )
        except helper.SourceError as e:
            raise PinError(str(e)) from None
        profile = literals(ident["root"], ENGINES[0], "PROFILE_API", "PROFILE_FEATURES")
        api = profile.get("PROFILE_API")
        if type(api) is not int or api != data["profile_api"]:
            raise PinError(f'agentrc at {r} implements profile API {api}, this checkout needs {data["profile_api"]}')
        features = profile.get("PROFILE_FEATURES", ())
        if not isinstance(features, (list, tuple)) or not all(isinstance(f, str) for f in features):
            raise PinError(f"agentrc at {r} has malformed PROFILE_FEATURES")
        missing = [f for f in data["features"] if f not in features]
        if missing:
            raise PinError(f"agentrc at {r} lacks profile features {missing} ({ENGINES[0]} PROFILE_FEATURES)")
    except PinError as e:
        e.remedy = hint
        raise
    except (OSError, ImportError) as e:
        raise PinError(str(e), hint) from None
    return helper, hil_load, ident


def selected():
    ident = identity()
    return ident["source"], Path(ident["root"]), ident["files"]


def identity():
    return _selection()[2]


def load(rel):
    helper, hil_load, ident = _selection()
    try:
        helper.require(ident, rel)
        return hil_load.load_file(rel)
    except (helper.SourceError, OSError, ImportError) as e:
        raise PinError(str(e), "rerun with an unchanged agentrc checkout") from None


def check():
    helper, _, ident = _selection()
    try:
        helper.binding(ident)
    except helper.SourceError as e:
        raise PinError(str(e), "rerun with an unchanged agentrc checkout") from None
    return Path(ident["root"])


def test():
    if source() == "pin":
        fetch()
    r = check()
    canonical = r / SELF
    consumer = Path(__file__).resolve()
    if consumer.read_bytes() != canonical.read_bytes():
        raise PinError(
            f"bootstrap copy {consumer} differs from {canonical}",
            f"copy {canonical} verbatim to {consumer} and repin test/hil/agentrc.json to that agentrc commit",
        )
    # A hook's GIT_DIR or GIT_INDEX_FILE would aim tests' scratch-repository commands at the caller's checkout.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    if identity()["source"] == "installed":
        env["AGENTRC_DIR"] = str(r)
    return subprocess.run(
        [sys.executable, "-m", "unittest", "discover", "-s", ".claude/test", "-p", "test_*.py"], cwd=ROOT, env=env
    ).returncode


def main(argv):
    cmd = argv[0] if argv else ""
    try:
        if cmd == "test":
            return test()
        if cmd == "fetch":
            print(fetch())
        elif cmd == "check":
            print(check())
        else:
            print(__doc__, file=sys.stderr)
            return 2
    except PinError as e:
        print(f"error: {e.with_remedy()}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
