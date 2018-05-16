# nrfutil

`nrfutil` is a Python package that includes the nrfutil command line utility
and the nordicsemi library.

This tool can be used used with the [Adafruit nRF52 Feather](https://www.adafruit.com/product/3406)
to flash firmware images onto the device using the simple serial port.

This library is written for Python 2.7.

# Installation

Run the following commands to make `nrfutil` available from the command line
or to development platforms like the Arduino IDE or CircuitPython:

**Notes** : Do **NOT** install nrfutil from the pip package (ex. `sudo pip
install nrfutil`). The latest nrfutil does not support DFU via Serial, and you
should install version 0.5.2 from a local copy of this repo via the methods
detailed below:

### OS X and Linux

```
$ sudo pip install -r requirements.txt
$ sudo python setup.py install
```

### Windows

#### Option 1: Pre-Built Binary

A pre-built 32-bit version of nrfutil is included as part of this repo in the
`binaries/win32` folder. You can use this pre-built binary by adding it to your
systems `$PATH` variable.

#### Option 2: Build nrfutil from Source

- Make sure that you have **Python 2.7** available on your system.
- Manually install **py2exe version 0.6.9** from this link (selecting
    the 32-bit or 64-bit version depending on your Python installation):
    [Download py2exe 0.6.9](https://sourceforge.net/projects/py2exe/files/py2exe/0.6.9/).
- VC compiler for Python (Windows only) (http://www.microsoft.com/en-us/download/confirmation.aspx?id=44266)
- From the command prompt, install nrfutil via pip as follows:

```
pip install -r requirements.txt
python setup.py install
```

To generate a self-contained Windows exe version of the utility (Windows only):

```
python setup.py py2exe
```

# Usage

To get info on usage of nrfutil:

```
nrfutil --help
```

To convert an nRF52 .hex file into a DFU pkg file that the serial bootloader
can make use of:

```
nrfutil dfu genpkg --dev-type 0x0052 --application firmware.hex dfu-package.zip
```

To flash a DFU pkg file over serial:

```
nrfutil dfu serial --package dfu-package.zip -p /dev/tty.SLAB_USBtoUART -b 115200
```
