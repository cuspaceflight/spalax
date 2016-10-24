cmake_minimum_required (VERSION 3.2)
project(Spalax)

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "Must specify toolchain file - NB: changing the toolchain requires deleting the CMake cache")
else()
    message(STATUS "Using Toolchain File: ${CMAKE_TOOLCHAIN_FILE}")
endif()

if (${MESSAGING_OS} STREQUAL chibios)
    SET(CHIBIOS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/ChibiOS")
    # Full list nil rt hal adc can dac ext gpt i2c i2s icu mac mmc_spi pal pwm rtc sdc serial serial_usb spi st uart usb memstreams nullstreams
    FIND_PACKAGE(ChibiOS 16 COMPONENTS rt hal adc can dac ext gpt i2c i2s icu mac mmc_spi pal pwm rtc sdc serial serial_usb spi st uart usb chprintf memstreams nullstreams evtimer shell syscalls REQUIRED)
    SET(ChibiOS_INCLUDE_DIRS ${ChibiOS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/firmware/config_${SPALAX_CONF})
endif()

add_subdirectory(external/cusf-messaging)
add_subdirectory(shared)

if (SPALAX_BUILD_FIRMWARE)
    add_subdirectory(firmware)
endif()

if (SPALAX_BUILD_GUI)
    add_subdirectory(gui)
endif()


