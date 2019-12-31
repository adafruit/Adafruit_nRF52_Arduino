import os
import glob
import sys
import subprocess
import time

travis = False
if "TRAVIS" in os.environ and os.environ["TRAVIS"] == "true":
    travis = True

all_warnings = False
if "ALL_WARNINGS" in os.environ and os.environ["ALL_WARNINGS"] == "true":
    all_warnings = True

ENV_VARIABLE_NAME = 'VARIANT'


exit_status = 0
success_count = 0
fail_count = 0

build_format = '| {:20} | {:30} | {:9} '
build_separator = '-' * 78

variants_dict = {
    'feather52840': 'Feather nRF52840 Express',
    'cplaynrf52840': 'Circuit Playground Bluefruit Express',
    'itsybitsy52840': 'ItsyBitsy nRF52840 Express',
    'cluenrf52840': 'Clue nRF52840',
    'feather52832': 'Feather nRF52832'
}

# STDERR receives output that starts with the following text, none of which should be considered a warning or error...
output_to_ignore = (
    'Picked up JAVA_TOOL_OPTIONS:',
    'Loading configuration...',
    'Initializing packages...',
    'Preparing boards...',
    'Verifying...',
)

def errorOutputFilter(line):
    if len(line) == 0:
        return False
    if line.isspace(): # Note: empty string does not match here!
        return False
    if line.startswith(output_to_ignore): # alternatively, can trim() each line, but that would create lots of short-lived strings...
        return False
    # TODO: additional items to remove?
    return True


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
    
    if all_warnings:
        subprocess.run("arduino --pref 'compiler.warning_level=all' --save-prefs", shell=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    for sketch in glob.iglob('libraries/**/*.ino', recursive=True):
        start_time = time.monotonic()

        # skip if example contains: ".skip" or ".skip.variant"
        # however ".build.variant" file can overwrite ".skip", used to build a specific variant only
        sketchdir = os.path.dirname(sketch)
        if ( (os.path.exists(sketchdir + '/.skip') or os.path.exists(sketchdir + '/.skip.' + variant)) and
                not os.path.exists(sketchdir + '/.build.' + variant)):
            success = "skipped"
        else:
            # TODO - preferably, would have STDERR show up in **both** STDOUT and STDERR.
            #        preferably, would use Python logging handler to get both distinct outputs and one merged output
            #        for now, split STDERR when building with all warnings enabled, so can detect warning/error output.
            if all_warnings:
                build_result = subprocess.run("arduino --verify {}".format(sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                build_result = subprocess.run("arduino --verify {}".format(sketch), shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            # get stderr into a form where len(warningLines) indicates a true warning was output to stderr
            warningLines = [];
            if all_warnings and build_result.stderr:
                tmpWarningLines = build_result.stderr.decode("utf-8").splitlines()
                warningLines = list(filter(errorOutputFilter, (tmpWarningLines)))

            if build_result.returncode != 0:
                exit_status = build_result.returncode
                success = "\033[31mfailed\033[0m   "
                fail_count += 1
            elif len(warningLines) != 0:
                exit_status = -1
                success = "\033[31mwarnings\033[0m "
                fail_count += 1
            else:
                success = "\033[32msucceeded\033[0m"
                success_count += 1

        build_duration = time.monotonic() - start_time

        if travis:
            print('travis_fold:start:build-{}\\r'.format(sketch))

        print((build_format + '| {:5.2f}s |').format(sketch.split(os.path.sep)[1], os.path.basename(sketch), success, build_duration))

        if success != "skipped":
            if build_result.returncode != 0:
                print(build_result.stdout.decode("utf-8"))
                if (build_result.stderr):
                    print(build_result.stderr.decode("utf-8"))
            if len(warningLines) != 0:
                for line in warningLines:
                    print(line)

        if travis:
            print('travis_fold:end:build-{}\\r'.format(sketch))            


build_time = time.monotonic()


# build only one variant if the environment variable is specified
if (ENV_VARIABLE_NAME in os.environ):
    variant = os.environ.get(ENV_VARIABLE_NAME)
    # only use the environment variable if the variant exists in the dictionary
    if (variant in variants_dict):
        build_examples(variant)
    else:
        print('\033[31INTERNAL ERR\033[0m - invalid variant name "{}"'.format(variant))
        fail_count += 1
        exit_status = -1

else: # no environment variable specified, so build all variants
    for var in variants_dict:
        build_examples(var)


print(build_separator)
build_time = time.monotonic() - build_time
print("Build Summary: {} \033[32msucceeded\033[0m, {} \033[31mfailed\033[0m and took {:.2f}s".format(success_count, fail_count, build_time))
print(build_separator)

sys.exit(exit_status)
