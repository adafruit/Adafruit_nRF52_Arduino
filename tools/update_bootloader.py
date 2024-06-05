import os
import urllib.request
from multiprocessing import Pool


# Detect version in platform.txt
version = ''
with open('platform.txt') as pf:
    platform_txt = pf.read()
    e = '{build.variant}_bootloader-'
    v1 = platform_txt.index(e) + len(e)
    v2 = platform_txt.index('_', v1)
    version = platform_txt[v1:v2]

print(f'version {version}')


def get_sd(name):
    if '52832' in name:
        return 's132_6.1.1'
    elif '52833' in name or name == 'pca10100':
        return 's140_7.3.0'
    else:
        # most of the board is 52840
        return 's140_6.1.1'


# Download a variant's bootloader
def download_variant(variant):
    sd = get_sd(variant)

    # Download from bootloader release
    f_zip = f'{variant}_bootloader-{version}_{sd}.zip'
    f_hex = f'{variant}_bootloader-{version}_{sd}.hex'
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

    if 's132' not in sd:
        print(f"Downloading {f_uf2}")
        urllib.request.urlretrieve(url_prefix + f_uf2, f'bootloader/{variant}/{f_uf2}')


if __name__ == "__main__":
    # Get all variants
    all_variant = []
    for entry in os.scandir("variants"):
        if entry.is_dir():
            all_variant.append(entry.name)
    all_variant.sort()

    with Pool() as p:
        p.map(download_variant, all_variant)
