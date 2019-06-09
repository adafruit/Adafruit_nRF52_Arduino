import os
import glob
import sys
import subprocess
import time
from pathlib import Path

home = str(Path.home())

exit_status = 0
success_count = 0
fail_count = 0

for sketch in glob.iglob('libraries/**/*.ino', recursive=True):
    start_time = time.monotonic()
    #build_result = subprocess.run("/home/hathach/Applications/arduino-1.8.9/arduino-builder -compile -hardware /home/hathach/Applications/arduino-1.8.9/hardware -hardware /home/hathach/.arduino15/packages -hardware /home/hathach/Arduino/hardware/ -tools /home/hathach/.arduino15/packages/adafruit/tools -tools /home/hathach/Applications/arduino-1.8.9/tools-builder -tools /home/hathach/.arduino15/packages -built-in-libraries /home/hathach/Applications/arduino-1.8.9/libraries -libraries /home/hathach/Arduino/libraries -fqbn=adafruit:nrf52:feather52840:softdevice=s140v6,debug=l0 -ide-version=10807 -warnings=none -prefs=build.warn_data_percentage=75 -verbose {}".format(sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    
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


print("Build Sumamary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m".format(success_count, fail_count))
sys.exit(exit_status)