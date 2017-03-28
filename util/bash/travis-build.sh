#!/bin/bash

# Log commands and exit on error
set -xe


# Compile firmware for Spalax Board
mkdir -p build.firmware.spalax
cd build.firmware.spalax
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/spalax.cmake
make
cd ..

# Compile firmware for M3IMU Board
mkdir -p build.firmware.m3imu
cd build.firmware.m3imu
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/m3imu.cmake
make
cd ..

# Compile GUI, Extractor and Tests
mkdir -p build.gui.spalax
cd build.gui.spalax
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/std.cmake
make
cd bin
# Tests disabled temporarily
#./spalax.tests
