# Windows specific setup instructions

## Environment Setup

Follow the instructions [here](http://www.jann.cc/2013/10/10/embedded_development_with_open_source_tools_on_windows.html#install-the-gcc-arm-embedded-toolchain) to install the GCC ARM toolchain, MinGW and MSYS

## GUI Build Configuration

Follow the instructions in [README.md](../README.md)

## Firmware Build Configuration

* Make sure you have the [ARM GCC toolchain](https://launchpad.net/gcc-arm-embedded) installed and added to your PATH
* Open the CMake GUI
* Set the source code the root directory of this repo
* Set the build directory to ${ROOT_DIR}/Avionics.Firmware.Build
* Click Configure - if it asks to create the directory select yes
* For the generator select 'MinGW Makefiles'
* Select 'Specify toolchain file for cross-compiling'
* Click Next
* Specify $[ROOT_DIR}/Toolchain-arm-none-eabi.cmake
* Click Finish
* Configuration will fail as you haven't selected the configuration to build
* Select the BUILD_FIRMWARE checkbox and click configure again
* Change the MAKE_COMMAND settings to 'mingw32-make'
* Click Configure again
* Finally click Generate
* Avionics.Firmware.Build should now contain the files to build the Firmware

## Firmware Build

* Open mintty and navigate to ${ROOT_DIR}/Avionics.Firmware.Build you just created
* Type 'mingw32-make' to build everything
* CMake suppresses Make standard output and so I recommend adding 'VERBOSE=1'
