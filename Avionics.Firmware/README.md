# Avionics Firmware

The firmware for the IMU board.

## Requirements

- CMake version 3.2 or greater
- The ARM cross-compiler found [here](<https://launchpad.net/gcc-arm-embedded>)
- Either GNU Make (Linux) or MinGW (Windows)
- Cygwin will **NOT** work as the ARM compiler doesn't understand Cygwin style paths.

## Linux Compilation

Navigate to the root directory of the repository then run

```console
$ cd Avionics.Firmware
$ make
```

## Windows Compilation

Navigate to the root directory of the repository then run

```console
$ cd Avionics.Firmware
$ mingw32-make
```

## Debugging

This document assumes a) you're using a Black magic (or Adam magic) probe and b)
that you've read the [getting
started](https://github.com/blacksphere/blackmagic/wiki/Getting-Started) guide
on the black magic probe wiki.

The IMU firmware is located in the ``Avionics.Firmware/build`` directory
relative to the repository root. Assuming you're in the repo root run the following:

```console
$ cd Avionics.Firmware/build
$ arm-none-eabi-gdb
```

Next connect to the probe via the USB serial link (you may need to use a different
device file).

```
(gdb) target extended-remote /dev/ttyACM0
Remote debugging using /dev/ttyACM0
(gdb) monitor swdp_scan
Target voltage: 3.3V
Available Targets:
No. Att Driver
 1      STM32F4xx
(gdb) attach 1
Attaching to Remote target
0x0800069c in ?? ()
```

Now we're attached, we can load and run the ``IMU.elf`` file (see the Black Magic
[useful gdb
commands](https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands)
wiki page).
