# Uploading firmware on Linux

This document assumes a) you're using a Black magic (or Adam magic) probe and b)
that you've read the [getting
started](https://github.com/blacksphere/blackmagic/wiki/Getting-Started) guide
on the black magic probe wiki.

The IMU firmware is located in the ``Avionics.Firmware/build`` directory
relative to the repository root. Assuming you're in the repo root:

```console
$ pwd
[ check output ends with "/state-estimators" ]
$ cd Avionics.Firmware/build
$ arm-none-eabi-gdb
[...output...]
```

Connect to the probe via the USB serial link. You may need to use a different
device file. (See the Black Magic getting started guide linked above.)

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

Now we're attached, we can load an run the ``IMU.elf`` file. See the Black Magic
[useful gdb
commands](https://github.com/blacksphere/blackmagic/wiki/Useful-GDB-commands)
wiki page.

