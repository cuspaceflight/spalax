#!/bin/bash

# Log commands and exit on error
set -xe

# Test-builds the firmware on Travis.
# Required packages: gcc-arm-none-eabi libnewlib-arm-none-eabi

cd firmware.spalax
make all VERBOSE=1
cd ../firmware.m3imu
make all VERBOSE=1
cd ../gui
mkdir build
cd build
cmake .. -G "Unix Makefiles"
cmake --build . --target spalax.gui.launcher
cmake --build . --target spalax.gui.tests