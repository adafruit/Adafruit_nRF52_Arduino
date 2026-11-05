#!/usr/bin/env python3
"""Run the HIL harness on a remote rig (default ci.lan) from the dev PC that built the firmware.

SKILL.md beside this directory is the contract. The rig gets binaries only: the harness, the roster, the build records
with their images and the agentrc files the harness loads, each pinned by digest in manifest.json, in a create-only
attempt dir <base>/<run-id> (agentrc hil_remote_core). Local records and collections live under .hil-remote/.

  hil_remote.py run --run-id ID [--host H] [--roster R] [-b BOARD]... [-t TEST]...
      stage HEAD's clean build records and launch `hil_test.py run` detached; prints the launch JSON
  hil_remote.py wait ID [--timeout S]      until done (collected into .hil-remote/ID), or the timeout
  hil_remote.py collect ID                 fetch whatever the attempt has now, whatever its state
  hil_remote.py session open --run-id ID -b BOARD [--host H] [--roster R]
      stage the board's clean record and take a hold on the board under a reason unique to the attempt
      (`arduino-hil session DIR TOKEN`, kept in .hil-remote/ID.session.json)
  hil_remote.py session stage ID RECORD...    stage records (experimental too) into a new bundle; prints its options
  hil_remote.py session exec ID flash|gdb|restore|console|check OPTS...  hil_test.py's command under the hold
  hil_remote.py session exec ID exec [--timeout S] -- ARGV...
  hil_remote.py session close ID           restore the board's restoration sketch, check it, release the hold (one
                                           rig command); after an open whose hold was not taken, release one
                                           the rig took anyway under the session's reason, restoring nothing

Exits: run, session open/stage 0 launched/held/staged, 1 refused or failed (an `error:` line names why), 2 no usable
agentrc (agentrc_pin.py; the line names the remedy); wait 0 done, 3 running, 4 dead, 5 inconclusive (hil_remote_core);
collect 0 complete (a session's: fetched); session exec passes the rig command's exit through, 3 for a session whose
hold the rig no longer shows; session close passes it through too, except that 3 closes the session and
exits 0: `restoreExit: 3` in its JSON, the restore unconfirmed, or after an open whose hold was not taken
`releaseExit: 3`, nothing having run.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402


def main(argv=None):
    """hil_test's profile, run by agentrc's arduino_remote.py."""
    try:
        hil_test.engine()
        remote = agentrc_pin.load('skills/arduino/scripts/arduino_remote.py')
    except agentrc_pin.PinError as e:
        print(f'error: {e}' + (f' ({e.remedy})' if e.remedy else ''), file=sys.stderr)
        return 2
    return remote.run(hil_test, argv)


if __name__ == '__main__':
    sys.exit(main())
