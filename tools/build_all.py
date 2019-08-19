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
build_separator = '-' * 77


def build_examples(variant):
    global exit_status, success_count, fail_count, build_format, build_separator
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

        print((build_format + '| {:.2f}s |').format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success, build_duration))

        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))

        if travis:
            print('travis_fold:end:build-{}\\r'.format(sketch))


build_time = time.monotonic()

print(build_separator)
print('| {:^73} |'.format('Feather nRF52840 Express'))
print(build_separator)
print((build_format + '| {:5} |').format('Library', 'Example', 'Result', 'Time'))
print(build_separator)
subprocess.run("arduino --board adafruit:nrf52:feather52840:softdevice=s140v6,debug=l0 --save-prefs", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
build_examples('feather52840')

print('\r\n')
print(build_separator)
print('| {:^73} |'.format('Feather nRF52832'))
print((build_format + '| {:5} |').format('Library', 'Example', 'Result', 'Time'))
print(build_separator)
subprocess.run("arduino --board adafruit:nrf52:feather52832:softdevice=s132v6,debug=l0 --save-prefs", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
build_examples('feather52832')

print('\r\n')
print(build_separator)
print('| {:^73} |'.format('Circuit Playground Bluefruit Express'))
print(build_separator)
print((build_format + '| {:5} |').format('Library', 'Example', 'Result', 'Time'))
print(build_separator)
subprocess.run("arduino --board adafruit:nrf52:cplaynrf52840:softdevice=s140v6,debug=l0 --save-prefs", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
build_examples('cplaynrf52840')


print(build_separator)
build_time = time.monotonic() - build_time
print("Build Sumamary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, build_time))
print(build_separator)

sys.exit(exit_status)
