import os
import glob
import sys
import subprocess
from subprocess import Popen, PIPE
import time

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[33mskipped\033[0m"
WARNING = "\033[31mwarnings\033[0m"

all_warnings = False
exit_status = 0
success_count = 0
fail_count = 0
skip_count = 0

build_format = '| {:20} | {:35} | {:18} | {:6} |'
build_separator = '-' * 83

default_boards = [ 'cluenrf52840', 'cplaynrf52840', 'feather52832', 'feather52840', 'feather52840sense', 'itsybitsy52840' ]
build_boards = []

# build all variants if input not existed
if len(sys.argv) > 1:
    build_boards.append(sys.argv[1])
else:
    build_boards = default_boards

all_examples = list(glob.iglob('libraries/**/*.ino', recursive=True))
all_examples.sort()

def errorOutputFilter(line):
    if len(line) == 0:
        return False
    if line.isspace(): # Note: empty string does not match here!
        return False
    # TODO: additional items to remove?
    return True


def build_examples(variant):
    global exit_status, success_count, fail_count, skip_count, build_format, build_separator

    print('\n')
    print(build_separator)
    print('| {:^79} |'.format('Board ' + variant))
    print(build_separator)
    print(build_format.format('Library', 'Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)
    
    fqbn = "adafruit:nrf52:{}:softdevice={},debug=l0".format(variant, 's140v6' if variant != 'feather52832' else 's132v6')

    for sketch in all_examples:
        start_time = time.monotonic()

        # Skip if contains: ".board.test.skip" or ".all.test.skip"
        # Skip if not contains: ".board.test.only" for a specific board
        sketchdir = os.path.dirname(sketch)
        if os.path.exists(sketchdir + '/.all.test.skip') or os.path.exists(sketchdir + '/.' + variant + '.test.skip'):
            success = SKIPPED
        elif glob.glob(sketchdir+"/.*.test.only") and not os.path.exists(sketchdir + '/.' + variant + '.test.only'):
            success = SKIPPED
        else:
            # TODO - preferably, would have STDERR show up in **both** STDOUT and STDERR.
            #        preferably, would use Python logging handler to get both distinct outputs and one merged output
            #        for now, split STDERR when building with all warnings enabled, so can detect warning/error output.
            if all_warnings:
                build_result = subprocess.run("arduino-cli compile --warnings all --fqbn {} {}".format(fqbn, sketch), shell=True, stdout=PIPE, stderr=PIPE)
            else:
                build_result = subprocess.run("arduino-cli compile --warnings default --fqbn {} {}".format(fqbn, sketch), shell=True, stdout=PIPE, stderr=PIPE)

            # get stderr into a form where len(warningLines) indicates a true warning was output to stderr
            warningLines = [];
            if all_warnings and build_result.stderr:
                tmpWarningLines = build_result.stderr.decode("utf-8").splitlines()
                warningLines = list(filter(errorOutputFilter, (tmpWarningLines)))

            if build_result.returncode != 0:
                exit_status = build_result.returncode
                success = FAILED
                fail_count += 1
            elif len(warningLines) != 0:
                exit_status = -1
                success = WARNING
                fail_count += 1
            else:
                success = SUCCEEDED
                success_count += 1

        build_duration = time.monotonic() - start_time

        print(build_format.format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success, '{:5.2f}s'.format(build_duration)))

        if success != SKIPPED:
            if build_result.returncode != 0:
                print(build_result.stdout.decode("utf-8"))
                if (build_result.stderr):
                    print(build_result.stderr.decode("utf-8"))
            if len(warningLines) != 0:
                for line in warningLines:
                    print(line)
        else:
            skip_count += 1

build_time = time.monotonic()

for board in build_boards:
    build_examples(board)

print(build_separator)
build_time = time.monotonic() - build_time
print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(success_count, SUCCEEDED, fail_count, FAILED, skip_count, SKIPPED, build_time))
print(build_separator)

sys.exit(exit_status)
