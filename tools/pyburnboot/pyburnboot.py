import sys
import platform
import os
import os.path
from pynrfjprog import API, Hex

# Command Line option
if (len(sys.argv) < 2):
  print "Burn bootloader with Softdevice hex using JLink"
  print ("Usage: python %s <hexfile>")%(sys.argv[0])
  sys.exit()

hexfile = sys.argv[1]

api = API.API('NRF52')
api.open()
api.connect_to_emu_without_snr()

print 'Erasing Feather'
api.erase_all()

print 'Writing bootloader'

# Convert hex to array
hexdata = Hex.Hex(hexfile)

# Write data to MCU
for seg in hexdata:
    api.write(seg.address, seg.data, True)

# Reset device and run
api.sys_reset()
api.go()

api.close()

print 'Done'