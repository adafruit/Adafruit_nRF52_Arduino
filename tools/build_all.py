import os
import glob
import sys
import subprocess
from subprocess import Popen, PIPE
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

# return [succeeded, failed, skipped]
def build_sketch(variant, sketch):
    fqbn = "adafruit:nrf52:{}:softdevice={},debug=l0".format(variant,
                                                             's140v6' if variant != 'feather52832' else 's132v6')
    ret = [0, 0, 0]

    # skip TinyUSB library examples for nRF52832
    if variant == 'feather52832' and "libraries/Adafruit_TinyUSB_Arduino" in sketch:
        return ret

    start_time = time.monotonic()
    # Skip if contains: ".board.test.skip" or ".all.test.skip"
    # Skip if not contains: ".board.test.only" for a specific board
    sketchdir = os.path.dirname(sketch)
    if os.path.exists(sketchdir + '/.all.test.skip') or os.path.exists(sketchdir + '/.' + variant + '.test.skip') or \
            (glob.glob(sketchdir + "/.*.test.only") and not os.path.exists(sketchdir + '/.' + variant + '.test.only')):
        success = SKIPPED
        ret[2] = 1
    else:
        build_result = subprocess.run("arduino-cli compile --warnings all --fqbn {} {}".format(fqbn, sketch),
                                      shell=True, stdout=PIPE, stderr=PIPE)

        # get stderr into a form where warning/error was output to stderr
        if build_result.returncode != 0:
            success = FAILED
            ret[1] = 1
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
            print(build_result.stderr.decode("utf-8"))
    return ret

# return [succeeded, failed, skipped]
def build_variant(variant):
    print('\n')
    print(build_separator)
    print('| {:^84} |'.format('Board ' + variant))
    print(build_separator)
    print(build_format.format('Library', 'Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)

    with Pool(processes=os.cpu_count()) as pool:
        pool_args = list((map(lambda e, b=variant: [b, e], all_examples)))
        result = pool.starmap(build_sketch, pool_args)
        # sum all element of same index (column sum)
        return list(map(sum, list(zip(*result))))

if __name__ == '__main__':
    build_boards = []

    # build all variants if input not existed
    if len(sys.argv) > 1:
        build_boards.append(sys.argv[1])
    else:
        build_boards = default_boards

    all_examples = list(glob.iglob('libraries/**/*.ino', recursive=True))
    all_examples.sort()

    total_time = time.monotonic()

    total_result = [0, 0, 0]
    for board in build_boards:
        ret = build_variant(board)
        total_result = list(map(lambda x, y: x + y, total_result, ret))

    print(build_separator)
    total_time = time.monotonic() - total_time
    print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(total_result[0], SUCCEEDED, total_result[1],
                                                                       FAILED, total_result[2], SKIPPED, total_time))
    print(build_separator)

    sys.exit(total_result[1])
