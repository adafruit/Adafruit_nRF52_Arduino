import os
import shutil
import urllib.request
import pathlib


def get_sd(name):
    if '52832' in name:
        return 's132_6.1.1'
    elif '52833' in name:
        return 's140_7.3.0'
    else:
        # most of the board is 52840
        return 's140_6.1.1'


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
    sd = get_sd(variant)

    # Download from bootloader release
    name = '{}_bootloader-{}'.format(variant, version)
    url = 'https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases/download/{}/'.format(version)

    # Download zip, hex and uf2 if not 832
    fzip = '{}_{}.zip'.format(name, sd)
    fhex = '{}_{}.hex'.format(name, sd)
    fuf2 = 'update-{}_nosd.uf2'.format(name, sd)

    print("Downloading ", fzip)
    urllib.request.urlretrieve(url + fzip, fzip)

    print("Downloading ", fhex)
    urllib.request.urlretrieve(url + fhex, fhex)

    if variant != 'feather_nrf52832':
        print("Downloading ", fuf2)
        urllib.request.urlretrieve(url + fuf2, fuf2)

    # remove existing bootloader and copy new files
    boot_path = pathlib.Path('bootloader/{}'.format(variant))
    shutil.rmtree(boot_path, ignore_errors=True)
    os.mkdir(boot_path)

    # move file
    os.rename(fzip, boot_path / fzip)
    os.rename(fhex, boot_path / fhex)
    if variant != 'feather_nrf52832':
        os.rename(fuf2, boot_path / fuf2)
