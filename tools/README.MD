## Windows 32-bit Python side-by-side 64-bit Python

Typically on Windows you will install the 64-bit version of Python, however
this presents a challenge when building binaries with cx_Freeze since it will
use this 64-bit version of Python exclusively.  To generate a 32-bit binary you
will need to install a 32-bit version of Python side-by-side the 64-bit version.
Unfortunately this is a tricky process but it can be simplified using the
virtualenv tool.

The steps below outline a general process for installing 64-bit Python as the
main Python interpretor and using a 32-bit Python virtualenv side-by-side to
build 32-bit executables with cx_Freeze.

First install Python 2.7 **x64** in its normal C:\Python27 location.

Next install Python 2.7 **x86** but override the location to be something unique
like C:\Python27-x86.  Also you **must** set the register extensions option in the
installer to disabled/not installed (this will prevent the 32-bit version taking
over the default path, etc.).

Now open a command window and navigate to the path that Python 32bit was installed
and run the `scripts/pip.exe` to install virtualenv in 32bit python:
```
c:\Python27-x86\Scripts\pip.exe install virtualenv
```

To create a virtualenv use the virtualenv.exe (also installed in the Scripts folder)
like normal.  For example to create a new virtual environment called py32 open a
terminal in your home directory and run:
```
c:\Python27-x86\Scripts\virtualenv.exe py32
```

The created virtualenv will have a Scripts subdirectory with an activate.bat.  When
this bat file is run it will setup the environment to use Python 32-bit.  For example
from your home directory run in a command terminal:
```
.\py32\Scripts\activate.bat
```

Now Python, pip, etc. should all be using the 32-bit Python virtual environment.
You can build software with cx_Freeze etc and it will use this 32-bit version
of Python.

