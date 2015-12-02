# state-estimators

[![Build Status](https://travis-ci.org/cuspaceflight/state-estimators.svg)](https://travis-ci.org/cuspaceflight/state-estimators)

## Compilation

* The project can be configured to build in one of two ways: to build the device firmware or to build the GUI.
* The instructions below describe how to perform configuration using the CMake GUI, although using the command line does also work.
* For Windows specific instructions see [windows-setup.md](Documents/windows-setup.md)
* The master branch is re-built on each commit. You can see the current [build status](https://travis-ci.org/cuspaceflight/state-estimators) on Travis.

### GUI Build Configuration

* Open the CMake GUI
* Set the source code to the root directory of this repo
* Set the build directory to ${ROOT_DIR}/Avionics.GUI.Build
* Click Configure - if it asks to create the directory select yes
* Select your desired build system as the generator (only MSVC has been tested)
* Click Finish
* Configuration will fail as you haven't selected the configuration to build
* Select the BUILD_GUI checkbox and click configure again
* Finally click Generate
* Avionics.GUI.Build should now contain the files to build the GUI

### Firmware Build Configuration

* Make sure you have the [ARM GCC toolchain](https://launchpad.net/gcc-arm-embedded) installed and added to your PATH
* Open the CMake GUI
* Set the source code the root directory of this repo
* Set the build directory to ${ROOT_DIR}/Avionics.Firmware.Build
* Click Configure - if it asks to create the directory select yes
* For the generator select the Makefiles appropriate to your system
* Select 'Specify toolchain file for cross-compiling'
* Click Next
* Specify ${ROOT_DIR}/Toolchain-arm-none-eabi.cmake
* Click Finish
* Configuration will fail as you haven't selected the configuration to build
* Select the BUILD_FIRMWARE checkbox and click configure again
* Finally click Generate
* Avionics.Firmware.Build should now contain the files to build the Firmware

## Resources

Information which is of use for electrical design is in the [electrical-resources.md](Documents/electrical-resources.md) file.

Other resources:

* [IMU quadrotor dataset](http://www.sfly.org/mav-datasets)
* [A Collection of Outdoor Robotic Datasets with centimeter-accuracy Ground Truth](http://www.mrpt.org/malaga_dataset_2009)
* [Handheld camera 6dof trajectories](http://webdav.is.mpg.de/pixel/benchmark4camerashake/)

Papers:

* [Stochastic modeling of MEMS sensors](http://www.cit.iit.bas.bg/cit_2010/v10-2/31-40.pdf)
* [Attitude Estimation Using Modified Rodrigues Parameters](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19960035754.pdf)
* [Analog Device's Application Note on the EKF implementation in another of their sensors](http://www.analog.com/media/en/technical-documentation/application-notes/AN-1157.pdf)
* [A Survey of Nonlinear Attitude Estimation Methods](http://ancs.eng.buffalo.edu/pdf/ancs_papers/2007/att_survey07.pdf)

Existing projects:

* https://github.com/sfwa/ukf
* [An existing sensor](https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4) which does sensor fusion whose SPI interface we may take inspiration from.
* https://github.com/kriswiner/BNO-055 - A driver for the above which does
    attitude estimation on an Arduino via a quaternion filter
* http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf - An efficient
    orientation filter for inertial and inertial/magnetic sensor arrays (Some
    existing work on putting this into some efficient embedded C)
