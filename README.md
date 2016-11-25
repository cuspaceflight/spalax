# Spalax

[![Build Status](https://travis-ci.org/cuspaceflight/spalax.svg?branch=master)](https://travis-ci.org/cuspaceflight/state-estimators)

This repository contains the schematics, PCB designs and firmware for a 10DOF IMU system.

## Components

- [Firmware](firmware) - Firmware for the various IMU boards
- [Extractor](extractor) - A CLI to convert telemetry data from a telemetry dump file or from an attached IMU board to CSV 
- [GUI](gui) - A 3D visualisation GUI which renders in realtime data sent from an attached IMU board
- [Shared](shared) - A static library providing common functionality between the various software components
- [PCB](pcb) - The KiCad PCB files for the Spalax IMU boards

## Build Requirements

- GCC v4.8+
- CMake v3.2+
- arm-none-eabi-gcc (firmware build)
- xorg-dev libgl1-mesa-dev libfreetype6-dev libglew-dev (GUI build)
- GLFW v3+ (GUI build) - may need to be compiled from source (see travis.yml)

## Compilation

Having installed all the dependencies simply run from the root directory of the repo

```bash
git submodule update --init --recursive
./travis-build.sh
```

This will configure and compile all the different components. If you only wish to configure a subset, just run the relevant section of travis-build.sh.
