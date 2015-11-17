#!/bin/bash

# Log commands and exit on error
set -xe

# Test-builds the firmware on Travis.
# Required packages: gcc-arm-none-eabi libnewlib-arm-none-eabi

mkdir build-fw
cd build-fw
cmake \
	-DBUILD_FIRMWARE=ON \
	-DCMAKE_TOOLCHAIN_FILE=../Toolchain-arm-none-eabi.cmake \
	..
make VERBOSE=1

