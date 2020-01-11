# Unsupported Arduino Nano 33 BLE

This directory contains an Adafruit bootloader for the Arduino Nano 33 BLE. This
isn't a supported Adafruit product: its inclusion is a community contribution,
and it's not for newbies.

To use the Bluefruit libraries and board definition on the Arduino Nano 33 BLE,
you need to first install the bootloader and SoftDevice, which currently
requires an SWD debugger such as a J-Link, Black Magic Probe, or ATMEL-ICE (in
the SAM port - operating as a CMSIS-DAP programmer).

## Installing the Adafruit (UF2) bootloader

There are pads (for pogo pins or soldering wires to) on the back of the Nano 33
BLE to access the SWD lines - see the second page of the
[Arduino pinout guide](https://content.arduino.cc/assets/Pinout-NANOble_v1.pdf).
They are spaced at 0.1 inch with each other and with the last row of pins on the
board, so you can use two pieces of perfboard to build a small pogo pin jig that
aligns itself using those last two pins. However you choose to connect, the pads
will need to be connected to the appropriate pins of your debug/programming
tool.

If you have a CMSIS-DAP compatible tool, you can use openocd as included with
the Arduino nRF528xx hardware support to flash the bootloader. Everywhere you
see "$ARDUINODIR" in the following command, replace it with something like
`/home/ryan/.arduino15` (for Linux).

```sh
$ARDUINODIR/packages/arduino/tools/openocd/0.10.0-arduino9/bin/openocd -d2 -s $ARDUINODIR/packages/arduino/tools/openocd/0.10.0-arduino9/share/openocd/scripts/ -f interface/cmsis-dap.cfg -f target/nrf52.cfg -c "telnet_port disabled; init; reset init; halt; adapter_khz 10000; nrf5 mass_erase; program {$ARDUINODIR/packages/adafruit/hardware/nrf52/0.15.1/bootloader/arduino_nano_33_ble/arduino_nano_33_ble_bootloader-0.2.13_s140_6.1.1.hex}; reset run; shutdown "
```

You may need to adjust the version numbers in the various paths there.

Once this is flashed, you'll need to use the Adafruit entry for the Nano 33 BLE
in the Boards menu, instead of the Arduino one, since the Arduino board config
uses a bootloader protocol that is uncommon on the nRF52 (SAM-BA/bossac) and not
the same as what is used by Adafruit (nrfutil/adafruit-nrfutil).

## Returning to Arduino (bossac) bootloader

Re-connect the Nano 33 BLE to your programmer, and load the
`packages/arduino/hardware/mbed/1.1.3/bootloaders/nano33ble/bootloader.hex` file
(from your `~/.arduino15` directory on Linux or equivalent on your platform)
again. If you have a CMSIS-DAP compatible programmer like an ATMEL-ICE, this is
pretty easy and can be done in the Arduino IDE:

- Under Tools, Programmer, choose ARM CMSIS-DAP compatible
- Under Tools, Board, choose the Arduino Nano 33 BLE in the "Arduino nRF528x
  (MBed OS)" section.
- Choose Tools, Burn Bootloader.

This runs the same openocd command described above but with the Arduino
bootloader hex.

## Peripherals

The same basic pins as the Arduino-provided core for this board are provided.

The on-board LSM9DS1 is on the second (onboard-only) I2C bus, called `Wire1`.
The pullup to that bus is `PIN_ENABLE_I2C_PULLUP`, which is enabled (set to
`HIGH`) on startup by default. Power to that sensor is controlled by
`PIN_ENABLE_SENSORS_3V3`, also set `HIGH` on startup by default.

D13 has an LED (yellow, next to USB port) as usual. In addition, there is an
onboard RGB LED: just as in the Arduino-provided core, `LEDR`, `LEDG`, and
`LEDB` are the pins controlling the three color components with `analogWrite`:
Note that 0 is max and 255 is min for these LEDs.

The Power LED is actually software-controlled, so if you need another LED, you
can control `LED_PWR` (active-high).

## Development

This is still a work in progress:

- Bootloader:
  - UF2 mass-storage flashing doesn't appear to work properly.
  - current.uf2 shows up as only 512 bytes.
- Arduino core:
  - Some things appear to result in an extra-long delay or hang, such as:
    - using the onboard LSM9DS1 with `Adafruit_LSM9DS1` never gets past the
      first serial printout, though it works fine with the Arduino-provided
      board config after replacing the constructor line with
      `Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire1);`
    - The bluefruit scan example works, but takes a long time before it gets to
      the first scan result.
