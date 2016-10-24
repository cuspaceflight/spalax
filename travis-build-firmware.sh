#!/bin/bash

# Log commands and exit on error
set -xe

# Test-builds the firmware on Travis.
# Required packages: gcc-arm-none-eabi libnewlib-arm-none-eabi

mkdir build.firmware.spalax
cd build.firmware.spalax
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/spalax.cmake
make
