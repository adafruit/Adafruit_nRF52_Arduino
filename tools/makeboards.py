#!/usr/bin/env python3

mcu_dict = {
    52832: {
        'flash_size': 290816,
        'data_size': 51712,
        'extra_flags': '-DNRF52832_XXAA -DNRF52',
        'ldscript': 'nrf52832_s132_v6.ld',
        'uf2_family': 0x00
    },
    52833: {
        'flash_size': 286720,
        'data_size': 106496,
        'extra_flags': '-DNRF52833_XXAA {build.flags.usb}',
        'ldscript': 'nrf52833_s140_v7.ld',
        'uf2_family': '0x621E937A'
    },
    52840: {
        'flash_size': 815104,
        'data_size': 237568,
        'extra_flags': '-DNRF52840_XXAA {build.flags.usb}',
        'ldscript': 'nrf52840_s140_v6.ld',
        'uf2_family': '0xADA52840'
    }
}


def build_upload(mcu, name):
    print("# Upload")
    print(f"{name}.bootloader.tool=bootburn")
    print(f"{name}.upload.tool=nrfutil")
    print(f"{name}.upload.protocol=nrfutil")
    if mcu == 52832:
        print(f"{name}.upload.use_1200bps_touch=false")
        print(f"{name}.upload.wait_for_upload_port=false")
        print(f"{name}.upload.native_usb=false")
    else:
        print(f"{name}.upload.use_1200bps_touch=true")
        print(f"{name}.upload.wait_for_upload_port=true")
    print(f"{name}.upload.maximum_size={mcu_dict[mcu]['flash_size']}")
    print(f"{name}.upload.maximum_data_size={mcu_dict[mcu]['data_size']}")
    print()


def build_header(mcu, name, variant, vendor_name, product_name, boarddefine, vid, pid_list):
    prettyname = vendor_name + " " + product_name
    print()
    print("# -----------------------------------")
    print(f"# {prettyname}")
    print("# -----------------------------------")
    print(f"{name}.name={prettyname}")
    print()

    print("# VID/PID for Bootloader, Arduino & CircuitPython")
    for i in range(len(pid_list)):
        print(f"{name}.vid.{i}={vid}")
        print(f"{name}.pid.{i}={pid_list[i]}")
    print()

    build_upload(mcu, name)

    print("# Build")
    print(f"{name}.build.mcu=cortex-m4")
    print(f"{name}.build.f_cpu=64000000")
    print(f"{name}.build.board={boarddefine}")
    print(f"{name}.build.core=nRF5")
    print(f"{name}.build.variant={variant}")
    print(f'{name}.build.usb_manufacturer="{vendor_name}"')
    print(f'{name}.build.usb_product="{product_name}"')

    mcu_info = mcu_dict[mcu]
    print(f"{name}.build.extra_flags={mcu_info['extra_flags']}")
    print(f"{name}.build.ldscript={mcu_info['ldscript']}")
    print(f"{name}.build.openocdscript=scripts/openocd/daplink_nrf52.cfg")
    if mcu != 52832:
        print(f"{name}.build.vid={vid}")
        print(f"{name}.build.pid={pid_list[0]}")
        print(f"{name}.build.uf2_family={mcu_info['uf2_family']}")
    print()


def build_softdevice(mcu, name):
    print("# Menu: SoftDevice")
    if mcu == 52832:
        print(f"{name}.menu.softdevice.s132v6=S132 6.1.1")
        print(f"{name}.menu.softdevice.s132v6.build.sd_name=s132")
        print(f"{name}.menu.softdevice.s132v6.build.sd_version=6.1.1")
        print(f"{name}.menu.softdevice.s132v6.build.sd_fwid=0x00B7")
    elif mcu == 52833:
        print(f"{name}.menu.softdevice.s140v7=S140 7.3.0")
        print(f"{name}.menu.softdevice.s140v7.build.sd_name=s140")
        print(f"{name}.menu.softdevice.s140v7.build.sd_version=7.3.0")
        print(f"{name}.menu.softdevice.s140v7.build.sd_fwid=0x0123")
    elif mcu == 52840:
        print(f"{name}.menu.softdevice.s140v6=S140 6.1.1")
        print(f"{name}.menu.softdevice.s140v6.build.sd_name=s140")
        print(f"{name}.menu.softdevice.s140v6.build.sd_version=6.1.1")
        print(f"{name}.menu.softdevice.s140v6.build.sd_fwid=0x00B6")
    print()


def build_debug(name):
    print("# Menu: Debug Level")
    print(f"{name}.menu.debug.l0=Level 0 (Release)")
    print(f"{name}.menu.debug.l0.build.debug_flags=-DCFG_DEBUG=0")
    print(f"{name}.menu.debug.l1=Level 1 (Error Message)")
    print(f"{name}.menu.debug.l1.build.debug_flags=-DCFG_DEBUG=1")
    print(f"{name}.menu.debug.l2=Level 2 (Full Debug)")
    print(f"{name}.menu.debug.l2.build.debug_flags=-DCFG_DEBUG=2")
    print(f"{name}.menu.debug.l3=Level 3 (Segger SystemView)")
    print(f"{name}.menu.debug.l3.build.debug_flags=-DCFG_DEBUG=3")
    print(f"{name}.menu.debug.l3.build.sysview_flags=-DCFG_SYSVIEW=1")
    print()


def build_debug_output(name):
    print("# Menu: Debug Port")
    print(f"{name}.menu.debug_output.serial=Serial")
    print(f"{name}.menu.debug_output.serial.build.logger_flags=-DCFG_LOGGER=0")
    print(f"{name}.menu.debug_output.serial1=Serial1")
    print(f"{name}.menu.debug_output.serial1.build.logger_flags=-DCFG_LOGGER=1 -DCFG_TUSB_DEBUG=CFG_DEBUG")
    print(f"{name}.menu.debug_output.rtt=Segger RTT")
    print(f"{name}.menu.debug_output.rtt.build.logger_flags=-DCFG_LOGGER=2 -DCFG_TUSB_DEBUG=CFG_DEBUG -DSEGGER_RTT_MODE_DEFAULT=SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL")


def build_global_menu():
    print("menu.softdevice=SoftDevice")
    print("menu.debug=Debug Level")
    print("menu.debug_output=Debug Port")


def make_board(mcu, name, variant, vendor_name, product_name, boarddefine, vid, pid_list):
    build_header(mcu, name, variant, vendor_name, product_name, boarddefine, vid, pid_list)
    build_softdevice(mcu, name)
    build_debug(name)
    build_debug_output(name)


# ------------------------------
# main
# ------------------------------
build_global_menu()

# ------------------------------
# Adafruit Boards
# ------------------------------

adafruit_boards_list = [
    [52832, "feather52832", "feather_nrf52832", "Adafruit", "Feather nRF52832", "NRF52832_FEATHER",
     "0x239A", []],

    [52840, "feather52840", "feather_nrf52840_express", "Adafruit", "Feather nRF52840 Express", "NRF52840_FEATHER",
     "0x239A", ["0x8029", "0x0029", "0x002A", "0x802A"]],

    [52840, "feather52840sense", "feather_nrf52840_sense", "Adafruit", "Feather nRF52840 Sense", "NRF52840_FEATHER_SENSE",
     "0x239A", ["0x8087", "0x0087", "0x0088", "0x8088"]],

    [52840, "feather_nrf52840_sense_tft", "feather_nrf52840_sense_tft", "Adafruit", "Feather nRF52840 Sense TFT", "NRF52840_FEATHER_SENSE_TFT",
     "0x239A", ["0x8087", "0x0087", "0x0088", "0x8088"]], # TODO shared VID with sense for now

    [52840, "itsybitsy52840", "itsybitsy_nrf52840_express", "Adafruit", "ItsyBitsy nRF52840 Express", "NRF52840_ITSYBITSY -DARDUINO_NRF52_ITSYBITSY",
     "0x239A", ["0x8051", "0x0051", "0x0052", "0x8052"]],

    [52840, "cplaynrf52840", "circuitplayground_nrf52840", "Adafruit", "Circuit Playground Bluefruit", "NRF52840_CIRCUITPLAY",
     "0x239A", ["0x8045", "0x0045", "0x8046"]],

    [52840, "cluenrf52840", "clue_nrf52840", "Adafruit", "CLUE", "NRF52840_CLUE",
     "0x239A", ["0x8071", "0x0071", "0x8072"]],

    [52840, "ledglasses_nrf52840", "ledglasses_nrf52840", "Adafruit", "LED Glasses Driver nRF52840", "NRF52840_LED_GLASSES",
     "0x239A", ["0x810D", "0x010D", "0x810E"]],

    [52840, "mdbt50qrx", "raytac_mdbt50q_rx", "Raytac", "nRF52840 Dongle", "MDBT50Q_RX",
     "0x239A", ["0x810B", "0x010B", "0x810C"]],

    [52840, "metro52840", "metro_nrf52840_express", "Adafruit", "Metro nRF52840 Express", "NRF52840_METRO",
     "0x239A", ["0x803F", "0x003F", "0x0040", "0x8040"]],

    # [52833, "feather52833", "feather_nrf52833_express", "Adafruit", "Feather nRF52833 Express", "NRF52833_FEATHER",
    #            "0x239A", ["0x8029", "0x0029", "0x002A", "0x802A"]],
]

for b in adafruit_boards_list:
    make_board(*b)

# ------------------------------
# 3rd Party Boards
# ------------------------------

print()
print()
print("# -------------------------------------------------------")
print("# Boards that aren't made by Adafruit")
print("# and are not officially supported")
print("# -------------------------------------------------------")

thirdparty_boards_list = [
    [52840, "pca10056", "pca10056", "Nordic", "nRF52840 DK", "NRF52840_PCA10056", "0x239A", ["0x80DA", "0x00DA"]],
    [52833, "pca10100", "pca10100", "Nordic", "nRF52833 DK", "NRF52833_PCA10100", "0x239A", ["0x80D8", "0x00D8"]],
    [52840, "particle_xenon", "particle_xenon", "Particle", "Xenon", "PARTICLE_XENON", "0x239A", ["0x80DA", "0x00DA"]],
]

for b in thirdparty_boards_list:
    make_board(*b)
