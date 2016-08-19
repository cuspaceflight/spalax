#!/bin/bash

# Log commands and exit on error
set -xe

# Test-builds the firmware on Travis.
# Required packages: gcc-arm-none-eabi libnewlib-arm-none-eabi

cd firmware.spalax
make all VERBOSE=1