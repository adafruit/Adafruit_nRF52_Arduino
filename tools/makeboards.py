#!/usr/bin/env python3
from bokeh.palettes import name

mcu_dict = {
    52832: {
        'flash_size': 290816,
        'data_size': 52224,
        'extra_flags': '-DNRF52832_XXAA -DNRF52',
        'ldscript': 'nrf52832_s132_v6.ld'
    },
    52840: {
        'flash_size': 815104,
        'data_size': 237568,
        'extra_flags': '-DNRF52840_XXAA {build.flags.usb}',
        'ldscript': 'nrf52840_s140_v6.ld'
    }
}


def get_mcu(name):
    if name == 'feather52832':
        return 52832
    else:
        return 52840


def build_upload(name):
    mcu = get_mcu(name)
    print("# Upload")
    print("%s.bootloader.tool=bootburn" % name)
    print("%s.upload.tool=nrfutil" % name)
    print("%s.upload.protocol=nrfutil" % name)
    if mcu == 52832:
        print("%s.upload.use_1200bps_touch=false" % name)
        print("%s.upload.wait_for_upload_port=false" % name)
        print("%s.upload.native_usb=false" % name)
    else:
        print("%s.upload.use_1200bps_touch=true" % name)
        print("%s.upload.wait_for_upload_port=true" % name)
    print("%s.upload.maximum_size=%d" % (name, mcu_dict[mcu]['flash_size']))
    print("%s.upload.maximum_data_size=%d" % (name, mcu_dict[mcu]['data_size']))
    print()


def build_header(name, variant, vendor_name, product_name, boarddefine, vid, pid_list):
    prettyname = vendor_name + " " + product_name
    print()
    print("# -----------------------------------")
    print("# %s" % prettyname)
    print("# -----------------------------------")
    print("%s.name=%s" % (name, prettyname))
    print()

    print("# VID/PID for Bootloader, Arduino & CircuitPython")
    for i in range(len(pid_list)):
        print("%s.vid.%d=%s" % (name, i, vid))
        print("%s.pid.%d=%s" % (name, i, pid_list[i]))
    print()

    build_upload(name)

    print("# Build")
    print("%s.build.mcu=cortex-m4" % name)
    print("%s.build.f_cpu=64000000" % name)
    print("%s.build.board=%s" % (name, boarddefine))
    print("%s.build.core=nRF5" % name)
    print("%s.build.variant=%s" % (name, variant))
    print('%s.build.usb_manufacturer="%s"' % (name, vendor_name))
    print('%s.build.usb_product="%s"' % (name, product_name))

    mcu = get_mcu(name)
    print("%s.build.extra_flags=%s" % (name, mcu_dict[mcu]['extra_flags']))
    print("%s.build.ldscript=%s" % (name, mcu_dict[mcu]['ldscript']))
    if mcu != 52832:
        print("%s.build.vid=%s" % (name, vid))
        print("%s.build.pid=%s" % (name, pid_list[0]))
    print()


def build_softdevice(name):
    print("# SoftDevice Menu")
    if get_mcu(name) == 52832:
        print("%s.menu.softdevice.s132v6=S132 6.1.1" % name)
        print("%s.menu.softdevice.s132v6.build.sd_name=s132" % name)
        print("%s.menu.softdevice.s132v6.build.sd_version=6.1.1" % name)
        print("%s.menu.softdevice.s132v6.build.sd_fwid=0x00B7" % name)
    else:
        print("%s.menu.softdevice.s140v6=S140 6.1.1" % name)
        print("%s.menu.softdevice.s140v6.build.sd_name=s140" % name)
        print("%s.menu.softdevice.s140v6.build.sd_version=6.1.1" % name)
        print("%s.menu.softdevice.s140v6.build.sd_fwid=0x00B6" % name)
    print()


def build_debug(name):
    print("# Debug Menu")
    print("%s.menu.debug.l0=Level 0 (Release)" % name)
    print("%s.menu.debug.l0.build.debug_flags=-DCFG_DEBUG=0" % name)
    print("%s.menu.debug.l1=Level 1 (Error Message)" % name)
    print("%s.menu.debug.l1.build.debug_flags=-DCFG_DEBUG=1" % name)
    print("%s.menu.debug.l2=Level 2 (Full Debug)" % name)
    print("%s.menu.debug.l2.build.debug_flags=-DCFG_DEBUG=2" % name)
    print("%s.menu.debug.l3=Level 3 (Segger SystemView)" % name)
    print("%s.menu.debug.l3.build.debug_flags=-DCFG_DEBUG=3" % name)
    print("%s.menu.debug.l3.build.sysview_flags=-DCFG_SYSVIEW=1" % name)
    print()

def build_debug_output(name):
    print("# Debug Output Menu")
    print("%s.menu.debug_output.serial=Serial" % name)
    print("%s.menu.debug_output.serial.build.logger_flags=-DCFG_LOGGER=0" % name)
    print("%s.menu.debug_output.serial1=Serial1" % name)
    print("%s.menu.debug_output.serial1.build.logger_flags=-DCFG_LOGGER=1 -DCFG_TUSB_DEBUG=CFG_DEBUG" % name)
    print("%s.menu.debug_output.rtt=Segger RTT" % name)
    print("%s.menu.debug_output.rtt.build.logger_flags=-DCFG_LOGGER=2 -DCFG_TUSB_DEBUG=CFG_DEBUG -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL" % name)

def build_global_menu():
    print("menu.softdevice=SoftDevice")
    print("menu.debug=Debug")
    print("menu.debug_output=Debug Output")

def make_board(name, vendor_name, product_name, vid, pid, boarddefine, variant):
    build_header(name, vendor_name, product_name, vid, pid, boarddefine, variant)
    build_softdevice(name)
    build_debug(name)
    build_debug_output(name)


build_global_menu()

make_board("feather52832", "feather_nrf52832", "Adafruit", "Feather nRF52832", "NRF52832_FEATHER",
           "0x239A", [])
make_board("feather52840", "feather_nrf52840_express", "Adafruit", "Feather nRF52840 Express", "NRF52840_FEATHER",
           "0x239A", ["0x8029", "0x0029", "0x002A", "0x802A"])

make_board("feather52840sense", "feather_nrf52840_sense", "Adafruit", "Feather nRF52840 Sense", "NRF52840_FEATHER_SENSE",
           "0x239A", ["0x8087", "0x0087", "0x0088", "0x8088"])

make_board("itsybitsy52840", "itsybitsy_nrf52840_express", "Adafruit", "ItsyBitsy nRF52840 Express", "NRF52840_ITSYBITSY -DARDUINO_NRF52_ITSYBITSY",
           "0x239A", ["0x8051", "0x0051", "0x0052", "0x8052"])

make_board("cplaynrf52840", "circuitplayground_nrf52840", "Adafruit", "Circuit Playground Bluefruit", "NRF52840_CIRCUITPLAY",
           "0x239A", ["0x8045", "0x0045", "0x8046"])

make_board("cluenrf52840", "clue_nrf52840", "Adafruit", "CLUE", "NRF52840_CLUE",
           "0x239A", ["0x8071", "0x0071", "0x8072"])

make_board("ledglasses_nrf52840", "ledglasses_nrf52840", "Adafruit", "LED Glasses Driver nRF52840", "NRF52840_LED_GLASSES",
           "0x239A", ["0x810D", "0x010D", "0x810E"])

make_board("mdbt50qrx", "raytac_mdbt50q_rx", "Raytac", "nRF52840 Dongle", "MDBT50Q_RX",
           "0x239A", ["0x810B", "0x010B", "0x810C"])

make_board("metro52840", "metro_nrf52840_express", "Adafruit", "Metro nRF52840 Express", "NRF52840_METRO",
           "0x239A", ["0x803F", "0x003F", "0x0040", "0x8040"])

print()
print()
print("# -------------------------------------------------------")
print("#")
print("# Boards that aren't made by Adafruit")
print("#")
print("# -------------------------------------------------------")

make_board("pca10056", "pca10056", "Nordic", "nRF52840 DK", "NRF52840_PCA10056",
           "0x239A", ["0x8029", "0x0029"])

make_board("particle_xenon", "particle_xenon", "Particle", "Xenon", "PARTICLE_XENON",
           "0x239A", ["0x8029", "0x0029"])
