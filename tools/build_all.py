import os
import glob
import sys
import subprocess
import time

travis = False
if "TRAVIS" in os.environ and os.environ["TRAVIS"] == "true":
    travis = True

exit_status = 0
success_count = 0
fail_count = 0

build_format = '| {:20} | {:30} | {:9} '
build_separator = '-' * 78

variants_dict = {
    'feather52840': 'Feather nRF52840 Express',
    'cplaynrf52840': 'Circuit Playground Bluefruit Express',
    'feather52832': 'Feather nRF52832'
}


def build_examples(variant):
    global exit_status, success_count, fail_count, build_format, build_separator

    print('\n')
    print(build_separator)
    print('| {:^74} |'.format(variants_dict[variant]))
    print(build_separator)
    print((build_format + '| {:6} |').format('Library', 'Example', 'Result', 'Time'))
    print(build_separator)
    subprocess.run("arduino --board adafruit:nrf52:{}:softdevice={},debug=l0 --save-prefs".format(variant, 's140v6' if variant != 'feather52832' else 's132v6'), shell=True,
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    for sketch in glob.iglob('libraries/**/*.ino', recursive=True):
        start_time = time.monotonic()

        if os.path.exists(os.path.dirname(sketch) + '/.skip.' + variant):
            success = "skipped"
        else:
            build_result = subprocess.run("arduino --verify {}".format(sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if build_result.returncode != 0:
                exit_status = build_result.returncode
                success = "\033[31mfailed\033[0m   "
                fail_count += 1
            else:
                success = "\033[32msucceeded\033[0m"
                success_count += 1

        build_duration = time.monotonic() - start_time

        if travis:
            print('travis_fold:start:build-{}\\r'.format(sketch))

        print((build_format + '| {:5.2f}s |').format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success, build_duration))

        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))

        if travis:
            print('travis_fold:end:build-{}\\r'.format(sketch))


build_time = time.monotonic()

for var in variants_dict:
    build_examples(var)

print(build_separator)
build_time = time.monotonic() - build_time
print("Build Summary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, build_time))
print(build_separator)

sys.exit(exit_status)
