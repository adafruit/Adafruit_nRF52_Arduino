import os
import glob
import sys
import subprocess
import time
from multiprocessing import Pool

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[35mskipped\033[0m"
WARNING = "\033[33mwarnings\033[0m "

build_format = '| {:25} | {:35} | {:18} | {:6} |'
build_separator = '-' * 88

default_boards = [
    'cluenrf52840',
    'cplaynrf52840',
    'feather52832',
    'feather52840',
    'feather52840sense',
    'itsybitsy52840'
]
build_boards = []

def get_sd(name):
    if '52832' in name:
        return 's132v6'
    elif '52833' in name or name == 'pca10100':
        return 's140v7'
    else:
        # most of the board is 52840
        return 's140v6'

def build_a_example(arg):
    variant = arg[0]
    sketch = arg[1]

    fqbn = "adafruit:nrf52:{}:softdevice={},debug=l0".format(variant, get_sd(variant))

    # succeeded, failed, skipped
    ret = [0, 0, 0]

    start_time = time.monotonic()

    # Skip if contains: ".board.test.skip" or ".all.test.skip"
    # Skip if not contains: ".board.test.only" for a specific board
    sketchdir = os.path.dirname(sketch)
    if os.path.exists(sketchdir + '/.all.test.skip') or os.path.exists(sketchdir + '/.' + variant + '.test.skip'):
        success = SKIPPED
        ret[2] = 1
    elif glob.glob(sketchdir + "/.*.test.only") and not os.path.exists(sketchdir + '/.' + variant + '.test.only'):
        success = SKIPPED
        ret[2] = 1
    else:
        build_result = subprocess.run("arduino-cli compile --warnings all --fqbn {} {}".format(fqbn, sketch), shell=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # get stderr into a form where warning/error was output to stderr
        if build_result.returncode != 0:
            ret[1] = 1
            success = FAILED
        else:
            ret[0] = 1
            if build_result.stderr:
                success = WARNING
            else:
                success = SUCCEEDED

    build_duration = time.monotonic() - start_time
    print(build_format.format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success,
                              '{:5.2f}s'.format(build_duration)))

    if success != SKIPPED:
        # Build failed
        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))

        # Build with warnings
        if build_result.stderr:
            print(f"::group::warning-message")
            print(build_result.stderr.decode("utf-8"))
            print(f"::endgroup::")

    return ret


def build_all_examples(variant):
    print('\n')
    print(build_separator)
    print('| {:^84} |'.format('Board ' + variant))
    print(build_separator)
    print(build_format.format('Library', 'Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)

    args = [[variant, s] for s in all_examples]
    with Pool() as pool:
        result = pool.map(build_a_example, args)
        # sum all element of same index (column sum)
        return list(map(sum, list(zip(*result))))


if __name__ == "__main__":
    # build all variants if input not existed
    if len(sys.argv) > 1:
        build_boards.append(sys.argv[1])
    else:
        build_boards = default_boards

    # All examples in libraries except TinyUSB which has its own ci to build
    all_examples = list(glob.iglob('libraries/**/*.ino', recursive=True))
    all_examples = [i for i in all_examples if "Adafruit_TinyUSB_Arduino" not in i]
    all_examples.sort()

    build_time = time.monotonic()

    # succeeded, failed, skipped
    total_result = [0, 0, 0]

    for board in build_boards:
        fret = build_all_examples(board)
        if len(fret) == len(total_result):
            total_result = [total_result[i] + fret[i] for i in range(len(fret))]

    build_time = time.monotonic() - build_time

    print(build_separator)
    print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(total_result[0], SUCCEEDED, total_result[1], FAILED, total_result[2], SKIPPED, build_time))
    print(build_separator)

    sys.exit(total_result[1])
