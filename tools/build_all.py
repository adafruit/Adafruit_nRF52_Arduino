import os
import glob
import sys
import subprocess
from subprocess import Popen, PIPE
import time

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[35mskipped\033[0m"
WARNING = "\033[33mwarnings\033[0m "

exit_status = 0
success_count = 0
fail_count = 0
skip_count = 0

build_format = '| {:25} | {:35} | {:18} | {:6} |'
build_separator = '-' * 88

default_boards = [ 'cluenrf52840', 'cplaynrf52840', 'feather52832', 'feather52840', 'feather52840sense', 'itsybitsy52840' ]
build_boards = []

# build all variants if input not existed
if len(sys.argv) > 1:
    build_boards.append(sys.argv[1])
else:
    build_boards = default_boards

all_examples = list(glob.iglob('libraries/**/*.ino', recursive=True))
all_examples.sort()

def build_examples(variant):
    global exit_status, success_count, fail_count, skip_count, build_format, build_separator

    print('\n')
    print(build_separator)
    print('| {:^84} |'.format('Board ' + variant))
    print(build_separator)
    print(build_format.format('Library', 'Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)
    
    fqbn = "adafruit:nrf52:{}:softdevice={},debug=l0".format(variant, 's140v6' if variant != 'feather52832' else 's132v6')

    for sketch in all_examples:
        # skip TinyUSB library examples for nRF52832
        if variant == 'feather52832' and "libraries/Adafruit_TinyUSB_Arduino" in sketch:
            continue

        start_time = time.monotonic()

        # Skip if contains: ".board.test.skip" or ".all.test.skip"
        # Skip if not contains: ".board.test.only" for a specific board
        sketchdir = os.path.dirname(sketch)
        if os.path.exists(sketchdir + '/.all.test.skip') or os.path.exists(sketchdir + '/.' + variant + '.test.skip'):
            success = SKIPPED
            skip_count += 1
        elif glob.glob(sketchdir+"/.*.test.only") and not os.path.exists(sketchdir + '/.' + variant + '.test.only'):
            success = SKIPPED
            skip_count += 1
        else:
            build_result = subprocess.run("arduino-cli compile --warnings all --fqbn {} {}".format(fqbn, sketch), shell=True, stdout=PIPE, stderr=PIPE)

            # get stderr into a form where warning/error was output to stderr
            if build_result.returncode != 0:
                exit_status = build_result.returncode
                success = FAILED
                fail_count += 1
            else:
                success_count += 1
                if build_result.stderr:
                    success = WARNING
                else:
                    success = SUCCEEDED

        build_duration = time.monotonic() - start_time

        print(build_format.format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success, '{:5.2f}s'.format(build_duration)))

        if success != SKIPPED:
            # Build failed
            if build_result.returncode != 0:
                print(build_result.stdout.decode("utf-8"))
            
            # Build with warnings
            if build_result.stderr:
                print(build_result.stderr.decode("utf-8"))

build_time = time.monotonic()

for board in build_boards:
    build_examples(board)

print(build_separator)
build_time = time.monotonic() - build_time
print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(success_count, SUCCEEDED, fail_count, FAILED, skip_count, SKIPPED, build_time))
print(build_separator)

sys.exit(exit_status)
