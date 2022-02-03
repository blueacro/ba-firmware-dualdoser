# Reef Volt DualDoser Firmware

This is the firmware for the ReefVolt DualDoser, as sold by
[blueAcro.com](blueacro.com), and part of the [Reef2Reef Reef-Pi
Community](https://www.reef2reef.com/forums/reef-pi-discussion.1296/). It
features two peristaltic dosing pumps which are controllable via a simple USB
endpoint, and two float switch inputs.

# Building and Contributing

In order to build, the following is required:

- A Linux system, VM, or WSL on Windows setup. You can build this straight on a
  Raspberrry Pi if you want.
- Basic build tools, such as GNU make, git (on Debian, `apt-get install build-essential git-core`)
- An installed GCC compiler for `arm-none-eabi-`. This can be fetched from
  multiple sources depending on your distribution.
  - ARM hosts their version at [ARM Developer](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
  - On debian, `apt-get install gcc-arm-none-eabi`
  - The version does matter, as older versions than about GCC 8 will not produce a
    bootloader which fits within the required 1KiB of flash space.
- A copy of `dfu-util` if you want to flash the device over USB.

## Build images

    make clean all

## Flash device

Device can be flashed when in DFU mode. This is done by holding down the button
while attaching the USB cable. A red LED will be on when in DFU mode.

    dfu-util -D build/dualdoser.dfu

A small python script is provided to enter DFU mode when running the
application. It requires Python 3.x and a copy of PyUSB to be installed.

    # Install helpers for the script
    virtualenv venv
    venv/bin/pip install pyusb
    # Use to enter the bootloader
    venv/bin/python app/enter_bl.py

# Communications and USB

The DualDoser firmware uses two Vendor mode endpoints (IN and OUT) which can
transfer a packet of up to 64 bytes at a time. The OUT endpoint receives
messages from the computer, and the IN is used to respond to status.

The first byte of any message identifies the command or response, and is encoded
in `app/commands.h`.

Currently, all bootloader commands are followed by a single `response` frame
which identifies current pump, float switch, and power input states.

# Failsafes

The dual doser is designed to only operate its pumps when actively being told by
the host system. If a pump on command is received, the DualDoser will only keep
the pump selected on for 5 seconds before turning it off. No one wants a flooded
floor or a cooked aquarium.

## Pinout

| Pin  | Function |
|------|----------|
| PA02 | 12V detect |
| PA04 | Analog level |
| PA06 | Driver 2 |
| PA07 | Driver 1 |
| PA08 | Float 1 |
| PA09 | Float 2 |
| PA14 | SDA |
| PA15 | SCL |
| PA16 | Button |
| PA17 | Red LED |
| PA22 | Green LED |
| PA23 | Blue LED |
| PA24 | USBD- |
| PA25 | USBD+ |

