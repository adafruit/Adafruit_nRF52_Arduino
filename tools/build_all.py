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

def build_examples(variant):
    global exit_status, success_count, fail_count
    for sketch in glob.iglob('libraries/**/*.ino', recursive=True):
        if ( os.path.exists( os.path.dirname(sketch) + '/.skip.' + variant ) ):
            print("Build {} SKIP".format(sketch))
            continue

        start_time = time.monotonic()
        build_result = subprocess.run("arduino --verify {}".format(sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        build_duration = time.monotonic() - start_time

        if build_result.returncode != 0:
            exit_status = build_result.returncode
            success = "\033[31mfailed\033[0m"
            fail_count += 1
        else:
            success = "\033[32msucceeded\033[0m"
            success_count += 1

        if travis:
            print('travis_fold:start:build-{}\\r'.format(sketch))

        print("Build {} took {:.2f}s and {}".format(sketch, build_duration, success))
        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))

        if travis:
            print('travis_fold:end:build-{}\\r'.format(sketch))

build_time = time.monotonic()

print("-----------------------------")
print("Set board to Feather nRF52840")
print("-----------------------------")
subprocess.run("arduino --board adafruit:nrf52:feather52840:softdevice=s140v6,debug=l0 --save-prefs", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
build_examples('feather52840')

print("-----------------------------")
print("Set board to Feather nRF52832")
print("-----------------------------")
subprocess.run("arduino --board adafruit:nrf52:feather52832:softdevice=s132v6,debug=l0 --save-prefs", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
build_examples('feather52832')

print("--------------")
print("Build Sumamary")
print("--------------")
build_time = time.monotonic() - build_time
print("{} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, build_time))
sys.exit(exit_status)
