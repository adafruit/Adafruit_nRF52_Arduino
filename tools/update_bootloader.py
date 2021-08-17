import os
import shutil
import urllib.request
import zipfile

# Get all variants
all_variant = []
for entry in os.scandir("variants"):
    if entry.is_dir():
        all_variant.append(entry.name)
all_variant.sort()

# Detect version in platform.txt
version = '';
with open('platform.txt') as pf:
    platform_txt = pf.read()
    e = '{build.variant}_bootloader-'
    v1 = platform_txt.index(e) + len(e)
    v2 = platform_txt.index('_', v1)
    version = platform_txt[v1:v2]

print('version {}'.format(version))

for variant in all_variant:
    # Download from bootloader release
    name = '{}_bootloader-{}.zip'.format(variant, version)
    url = 'https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases/download/{}/{}'.format(version, name)
    print("Downloading", name)
    urllib.request.urlretrieve(url, name)

    # remove existing bootloader
    shutil.rmtree('bootloader/{}'.format(variant), ignore_errors=True)

    # unzip
    with zipfile.ZipFile(name, "r") as zip_ref:
        zip_ref.extractall("bootloader/{}".format(variant))

    # Remove update.uf2 for 832
    if variant == 'feather_nrf52832':
        os.remove("bootloader/{}/update-{}_nosd.uf2".format(variant, name[:-4]))

    # remove zip file
    os.remove(name)