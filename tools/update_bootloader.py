import os
import urllib.request
from multiprocessing import Pool

# Get all variants
all_variant = []
for entry in os.scandir("variants"):
    if entry.is_dir():
        all_variant.append(entry.name)
all_variant.sort()

# Detect version in platform.txt
version = ''
with open('platform.txt') as pf:
    platform_txt = pf.read()
    e = '{build.variant}_bootloader-'
    v1 = platform_txt.index(e) + len(e)
    v2 = platform_txt.index('_', v1)
    version = platform_txt[v1:v2]

print(f'version {version}')


# Download a variant's bootloader
def download_variant(variant):
    # Download from bootloader release
    sd_version = '6.1.1'
    sd_name = 's140'
    if variant == 'feather_nrf52832':
        sd_name = 's132'

    f_zip = f'{variant}_bootloader-{version}_{sd_name}_{sd_version}.zip'
    f_hex = f'{variant}_bootloader-{version}_{sd_name}_{sd_version}.hex'
    f_uf2 = f'update-{variant}_bootloader-{version}_nosd.uf2'
    url_prefix = f'https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases/download/{version}/'

    # remove existing bootloader files
    if os.path.exists(f'bootloader/{variant}'):
        for item in os.listdir(f'bootloader/{variant}'):
            os.remove(os.path.join(f'bootloader/{variant}', item))
    else:
        os.makedirs(f'bootloader/{variant}')

    print(f"Downloading {f_zip}")
    urllib.request.urlretrieve(url_prefix + f_zip, f'bootloader/{variant}/{f_zip}')

    print(f"Downloading {f_hex}")
    urllib.request.urlretrieve(url_prefix + f_hex, f'bootloader/{variant}/{f_hex}')

    if sd_name != 's132':
        print(f"Downloading {f_uf2}")
        urllib.request.urlretrieve(url_prefix + f_uf2, f'bootloader/{variant}/{f_uf2}')

if __name__ == "__main__":
    with Pool() as p:
        p.map(download_variant, all_variant)
